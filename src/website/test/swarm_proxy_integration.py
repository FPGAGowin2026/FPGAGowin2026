#!/usr/bin/env python3
"""Check the real swarm and ACG720 proxies against separate loopback upstreams."""
import json
import os
from pathlib import Path
import socket
import subprocess
import sys
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.error import HTTPError, URLError
from urllib.parse import urlsplit
from urllib.request import ProxyHandler, Request, build_opener


received = []
acg_received = []
OPENER = build_opener(ProxyHandler({}))

class Upstream(BaseHTTPRequestHandler):
    reobserve_exploring = False

    def log_message(self, *_args):
        pass

    def respond(self, status, payload):
        body = json.dumps(payload).encode()
        self.send_response(status)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        received.append(('GET', self.path, None, None))
        self.respond(200, {'upstream_path': self.path})

    def do_POST(self):
        body = self.rfile.read(int(self.headers.get('Content-Length', '0')))
        data = json.loads(body)
        received.append(('POST', self.path, data, self.headers.get('Content-Type')))
        if self.path == '/api/alignment/reobserve':
            if data != {}:
                self.respond(400, {'error':'empty object required'})
            elif self.reobserve_exploring:
                self.respond(409, {'error':'exploration active'})
            else:
                self.respond(200, {'accepted':True})
            return
        self.respond(202, {'accepted': True})


class Acg720Upstream(Upstream):
    failure = False
    preview = b'\xff\xd8acg720-preview\xff\xd9'

    def do_GET(self):
        acg_received.append(('GET', self.path, None, None))
        if urlsplit(self.path).path == '/frame.jpg':
            self.send_response(200)
            self.send_header('Content-Type', 'image/jpeg')
            self.send_header('Content-Length', str(len(self.preview)))
            self.end_headers()
            self.wfile.write(self.preview)
        else:
            self.respond(200, {'robot_id': 'acg720', 'fpga_connected': True,
                               'upstream_path': self.path})

    def do_POST(self):
        body = self.rfile.read(int(self.headers.get('Content-Length', '0')))
        data = json.loads(body)
        acg_received.append(('POST', self.path, data, self.headers.get('Content-Type')))
        if self.failure:
            self.respond(503, {'error': 'ACG720 bridge is not ready'})
        else:
            self.respond(200, {'accepted': True, 'robot_id': 'acg720',
                               'delivery': 'sent', 'acknowledged': False,
                               'command': 10})


def raw_request(url, data=None, headers=None):
    try:
        response = OPENER.open(Request(url, data=data, headers=headers or {},
                                      method='POST' if data is not None else 'GET'), timeout=7)
    except HTTPError as error:
        response = error
    with response:
        body = response.read()
        assert int(response.headers['Content-Length']) == len(body)
        return response.status, body, response.headers


def request(url, data=None, headers=None):
    status, body, reply_headers = raw_request(url, data, headers)
    return status, json.loads(body) if 'application/json' in reply_headers.get('Content-Type', '') else None


def main():
    upstream = ThreadingHTTPServer(('127.0.0.1', 0), Upstream)
    thread = threading.Thread(target=upstream.serve_forever, daemon=True); thread.start()
    acg_upstream = ThreadingHTTPServer(('127.0.0.1', 0), Acg720Upstream)
    acg_thread = threading.Thread(target=acg_upstream.serve_forever, daemon=True); acg_thread.start()
    with socket.socket() as probe:
        probe.bind(('127.0.0.1', 0)); port = probe.getsockname()[1]
    server = Path(__file__).resolve().parents[1] / 'dashboard_server.py'
    env = dict(os.environ, ROS_DOMAIN_ID='195', ROS_LOCALHOST_ONLY='1')
    process = subprocess.Popen([sys.executable, str(server), '--bind', '127.0.0.1', '--port', str(port),
                                '--swarm-upstream', f'http://127.0.0.1:{upstream.server_port}',
                                '--acg720-upstream', f'http://127.0.0.1:{acg_upstream.server_port}'],
                               env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    base = f'http://127.0.0.1:{port}'
    try:
        end = time.monotonic()+10
        while True:
            try:
                status, own = request(base+'/api/state')
                break
            except URLError:
                if time.monotonic()>end: raise
                time.sleep(.1)
        assert status == 200 and 'upstream_path' not in own
        fleet_status, fleet = request(base+'/api/fleet')
        assert fleet_status == 200
        assert [member['id'] for member in fleet['members']] == ['car', 'acg720']
        assert fleet['members'][1]['type'] == 'hexapod'
        assert request(base+'/acg720/api/status') == (200, {'robot_id':'acg720',
                                                          'fpga_connected':True, 'upstream_path':'/api/status'})
        assert not received, 'ACG720 status must not reach the swarm upstream'
        status, image, headers = raw_request(base+'/acg720/frame.jpg?v=7')
        assert status == 200 and image == Acg720Upstream.preview
        assert headers['Content-Type'] == 'image/jpeg' and headers['Cache-Control'] == 'no-store'
        assert acg_received[-1][1] == '/frame.jpg?v=7'
        acg_payload = json.dumps({'action':'ripple','pressed':True,'client_id':'acg720-test','lease_ms':500}).encode()
        acg_result = {'accepted':True,'robot_id':'acg720','delivery':'sent','acknowledged':False,'command':10}
        assert request(base+'/acg720/api/control', acg_payload) == (200, acg_result)
        assert acg_received[-1][1:] == ('/api/control', json.loads(acg_payload), 'application/json')
        assert not received, 'ACG720 control must not reach the swarm upstream'
        assert request(base+'/swarm/api/state') == (200, {'upstream_path':'/api/state'})
        for view in ('car', 'acg720', 'common'):
            query = f'/api/map?view={view}'
            assert request(base+'/swarm'+query) == (200, {'upstream_path':query})
        before_retired = len(received)
        assert request(base+'/swarm/api/hex/status')[0] == 404
        assert request(base+'/swarm/api/hex/control',b'{}')[0] == 404
        assert len(received) == before_retired, 'retired routes are not forwarded upstream'
        acg_count = len(acg_received)
        swarm_count = len(received)
        with ThreadPoolExecutor(max_workers=2) as pool:
            acg_future = pool.submit(request, base+'/acg720/api/control', acg_payload)
            swarm_future = pool.submit(request, base+'/swarm/api/explore/start', b'{}')
            assert acg_future.result() == (200, acg_result)
            assert swarm_future.result() == (202, {'accepted':True})
        assert len(acg_received) == acg_count + 1 and len(received) == swarm_count + 1
        assert acg_received[-1][2]['client_id'] == 'acg720-test'
        assert received[-1][1:] == ('/api/explore/start',{},'application/json')
        Acg720Upstream.failure = True
        assert request(base+'/acg720/api/control', acg_payload) == (503, {'error':'ACG720 bridge is not ready'})
        assert len(received) == swarm_count + 1, 'ACG720 failure must not retry via the swarm service'
        Acg720Upstream.failure = False
        assert request(base+'/swarm/api/explore/start',b'{}') == (202, {'accepted':True})
        assert request(base+'/swarm/api/explore/confirm-stopped',b'{"confirmed":true}') == (202, {'accepted':True})
        assert request(base+'/swarm/api/alignment/reobserve',b'{}') == (200, {'accepted':True})
        assert received[-1][1:] == ('/api/alignment/reobserve',{},'application/json')
        assert request(base+'/swarm/api/alignment/reobserve',b'{"x":1}') == (400, {'error':'empty object required'})
        Upstream.reobserve_exploring = True
        assert request(base+'/swarm/api/alignment/reobserve',b'{}') == (409, {'error':'exploration active'})
        before = len(received)
        acg_before = len(acg_received)
        assert request(base+'/swarm/api/alignment',b'{}')[0] == 404
        assert request(base+'/swarm/api/unknown',b'{}')[0] == 404
        assert request(base+'/swarm/api/hex/control',b'')[0] == 404
        assert request(base+'/acg720/api/control',b'')[0] == 400
        assert request(base+'/acg720/api/control',b'{}', {'Content-Length':'16385'})[0] == 400
        assert request(base+'/acg720/api/unknown',b'{}')[0] == 404
        assert len(received) == before and len(acg_received) == acg_before
        acg_upstream.shutdown(); acg_upstream.server_close(); acg_thread.join(timeout=2)
        assert request(base+'/acg720/api/status')[0] == 503
        assert request(base+'/swarm/api/state') == (200, {'upstream_path':'/api/state'})
        assert len(acg_received) == acg_before, 'offline ACG720 must not receive or reroute a request'
        print('PASS dashboard proxy: swarm/ACG720 isolation, retired route removal, control results, JPEG and offline isolation')
    finally:
        process.terminate()
        try: process.wait(timeout=3)
        except subprocess.TimeoutExpired: process.kill(); process.wait()
        upstream.shutdown(); upstream.server_close(); thread.join(timeout=2)
        if acg_thread.is_alive():
            acg_upstream.shutdown(); acg_upstream.server_close(); acg_thread.join(timeout=2)


if __name__ == '__main__':
    main()
