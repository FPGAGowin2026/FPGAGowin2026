"""A delayed request cannot undo a confirmed stop or cancel a newer goal."""
from collections import OrderedDict
import importlib.util
import json
from pathlib import Path
import threading
from types import SimpleNamespace


path = Path(__file__).resolve().parents[1] / 'scripts' / 'jetson_global_navigation_node.py'
spec = importlib.util.spec_from_file_location('global_node_safety_test', path)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def test_stop_fences_old_start_and_new_start_fences_old_cancel():
    class Coordinator:
        def __init__(self):
            self.bridge_safety_epochs = {'bridge':0}
            self.bridge_safety_lock = threading.Lock()
            self.bridge_safety_condition = threading.Condition(self.bridge_safety_lock)
            self.bridge_safety_blocked = False
            self.bridge_control_busy = False
            self.bridge_safety_id = 'bridge'
            self.bridge_safety_seq = 0
            self.retired_bridge_ids = set()
            self.request_cache = OrderedDict()
            self.response_lock = threading.Lock()
            self.responses = {}
            self.state = {}
            self.sent = []

        def send(self, value):
            self.sent.append(value['request']['action'])
            pending = self.responses[value['id']]
            pending['result'] = {'success': True, 'message': 'ok', 'state': {}}
            pending['event'].set()
            return True

    node = Coordinator()
    def issue(action, epoch):
        request = SimpleNamespace(request_json=json.dumps(
            {'action': action, '_bridge_id': 'bridge', '_bridge_safety_epoch': epoch,
             '_bridge_control_seq':node.bridge_safety_seq}))
        response = SimpleNamespace(success=False, unknown=False, message='', state_json='')
        return module.GlobalNavigationNode.command(node, request, response)

    assert issue('start', 0).success
    assert issue('cancel', 1).success
    old_false = SimpleNamespace(data=json.dumps({'bridge_id':'bridge', 'sequence':1,
        'safety_epoch':0, 'emergency_stop':False, 'global_cancel_pending':False, 'control_busy':False}))
    module.GlobalNavigationNode.receive_safety_state(node, old_false)
    assert node.bridge_safety_blocked, 'a stale false topic cannot undo a later cancel service'
    assert not issue('start', 0).success
    confirmed_false = SimpleNamespace(data=json.dumps({'bridge_id':'bridge', 'sequence':2,
        'safety_epoch':2, 'emergency_stop':False, 'global_cancel_pending':False, 'control_busy':False}))
    module.GlobalNavigationNode.receive_safety_state(node, confirmed_false)
    assert issue('start', 2).success
    assert issue('cancel', 1).success
    assert node.sent == ['start', 'cancel', 'start']
    def retry_start(seq):
        request = SimpleNamespace(request_json=json.dumps({'action':'start', 'session':'s', 'request_id':'same',
            '_bridge_id':'bridge', '_bridge_safety_epoch':2, '_bridge_control_seq':seq}))
        response = SimpleNamespace(success=False, unknown=False, message='', state_json='')
        return module.GlobalNavigationNode.command(node, request, response)
    assert retry_start(2).success
    newer_status = SimpleNamespace(data=json.dumps({'bridge_id':'bridge', 'sequence':3, 'safety_epoch':2,
        'emergency_stop':False, 'global_cancel_pending':False, 'control_busy':False}))
    module.GlobalNavigationNode.receive_safety_state(node, newer_status)
    assert retry_start(3).success
    assert node.sent == ['start', 'cancel', 'start', 'start'], 'retry must not start a second task'


def test_safety_state_and_direct_ros_start_are_ordered():
    class Coordinator:
        def __init__(self):
            self.bridge_safety_epochs = {}
            self.bridge_safety_lock = threading.Lock()
            self.bridge_safety_condition = threading.Condition(self.bridge_safety_lock)
            self.bridge_safety_blocked = False
            self.bridge_control_busy = False
            self.bridge_safety_id = None
            self.bridge_safety_seq = None
            self.retired_bridge_ids = set()
            self.request_cache = OrderedDict()
            self.response_lock = threading.Lock()
            self.responses = {}
            self.state = {}
            self.sent = []
            self.start_entered = threading.Event()
            self.release_start = threading.Event()

        def send(self, value):
            action = value['request']['action']
            if action == 'start':
                self.start_entered.set()
                assert self.release_start.wait(2)
            self.sent.append(action)
            pending = self.responses.get(value['id'])
            if pending:
                pending['result'] = {'success':True, 'message':'ok', 'state':{}}
                pending['event'].set()
            return True

    safety = SimpleNamespace(data=json.dumps({'bridge_id':'b', 'sequence':1, 'safety_epoch':1,
        'emergency_stop':True, 'global_cancel_pending':True, 'control_busy':False}))
    response = lambda: SimpleNamespace(success=False, unknown=False, message='', state_json='')
    request = SimpleNamespace(request_json=json.dumps({'action':'start'}))
    first = Coordinator()
    module.GlobalNavigationNode.receive_safety_state(first, safety)
    assert not module.GlobalNavigationNode.command(first, request, response()).success
    assert first.sent == ['cancel']

    second = Coordinator()
    result = response()
    starting = threading.Thread(target=module.GlobalNavigationNode.command, args=(second, request, result))
    starting.start()
    assert second.start_entered.wait(2)
    stopping = threading.Thread(target=module.GlobalNavigationNode.receive_safety_state, args=(second, safety))
    stopping.start()
    second.release_start.set()
    starting.join(2)
    stopping.join(2)
    assert not starting.is_alive() and not stopping.is_alive()
    assert second.sent == ['start', 'cancel']
    assert second.bridge_safety_blocked

    newer = SimpleNamespace(data=json.dumps({'bridge_id':'new', 'sequence':1, 'safety_epoch':1,
        'emergency_stop':True, 'global_cancel_pending':True, 'control_busy':False}))
    delayed_old = SimpleNamespace(data=json.dumps({'bridge_id':'b', 'sequence':2, 'safety_epoch':2,
        'emergency_stop':False, 'global_cancel_pending':False, 'control_busy':False}))
    module.GlobalNavigationNode.receive_safety_state(second, newer)
    module.GlobalNavigationNode.receive_safety_state(second, delayed_old)
    assert second.bridge_safety_id == 'new'
    assert second.bridge_safety_blocked
    old_cancel = SimpleNamespace(request_json=json.dumps({'action':'cancel', '_bridge_id':'b',
        '_bridge_safety_epoch':2}))
    assert module.GlobalNavigationNode.command(second, old_cancel, response()).success

    third = Coordinator()
    third.release_start.set()
    def status(seq, busy):
        return SimpleNamespace(data=json.dumps({'bridge_id':'b', 'sequence':seq, 'safety_epoch':0,
            'emergency_stop':False, 'global_cancel_pending':False, 'control_busy':busy}))
    module.GlobalNavigationNode.receive_safety_state(third, status(1, False))
    unknown_bridge = SimpleNamespace(request_json=json.dumps({'action':'start', '_bridge_id':'unseen-old',
        '_bridge_safety_epoch':0, '_bridge_control_seq':1}))
    rejected = module.GlobalNavigationNode.command(third, unknown_bridge, response())
    assert not rejected.success and '不是当前安全状态来源' in rejected.message
    module.GlobalNavigationNode.receive_safety_state(third, status(2, True))
    assert not module.GlobalNavigationNode.command(third, request, response()).success
    assert third.sent == ['cancel'], 'manual takeover cancels any queued global goal'
    same_page_request = SimpleNamespace(request_json=json.dumps({'action':'start', '_bridge_id':'b',
        '_bridge_safety_epoch':0, '_bridge_control_seq':3}))
    same_page_result = response()
    switching = threading.Thread(target=module.GlobalNavigationNode.command,
                                 args=(third, same_page_request, same_page_result))
    switching.start()
    module.GlobalNavigationNode.receive_safety_state(third, status(3, False))
    switching.join(2)
    assert not switching.is_alive()
    assert same_page_result.success, 'same page may start after the release status arrives'
    assert third.sent == ['cancel', 'start']

    fourth = Coordinator()
    fourth.release_start.set()
    module.GlobalNavigationNode.receive_safety_state(fourth, status(1, False))
    coalesced = SimpleNamespace(data=json.dumps({'bridge_id':'b', 'sequence':3, 'safety_epoch':1,
        'emergency_stop':False, 'global_cancel_pending':False, 'control_busy':False}))
    module.GlobalNavigationNode.receive_safety_state(fourth, coalesced)
    assert fourth.sent == ['cancel'], 'a missed brief manual claim still invalidates the old goal'
    after_release = SimpleNamespace(request_json=json.dumps({'action':'start', '_bridge_id':'b',
        '_bridge_safety_epoch':1, '_bridge_control_seq':3}))
    assert module.GlobalNavigationNode.command(fourth, after_release, response()).success
    assert fourth.sent == ['cancel', 'start']
