// 实际 TCP 慢视频客户端：暂停读取、制造回压，同时观察独立控制 UDP。
import assert from 'node:assert/strict';
import { once } from 'node:events';
import { createRequire } from 'node:module';
import { performance } from 'node:perf_hooks';
import { setTimeout as delay } from 'node:timers/promises';
import { createDashboardServer } from '../server.mjs';
import { createSimulator } from '../simulator.mjs';
const require = createRequire(new URL('../package.json', import.meta.url));
const WebSocket = require('ws');

async function until(predicate, label, timeout = 12000) {
  const end = performance.now()+timeout;
  while (performance.now()<end) {
    if (predicate()) return;
    await delay(10);
  }
  throw Error(`等待超时：${label}`);
}
const simulator = await createSimulator({ port: 0, videoFps: 300, quiet: true });
let server, control, slow, heartbeat;
let state, hello, sequence = 0, receivedCommands = 0, maxBuffered = 0;
let lastCommandAt, maxCommandGap = 0;
simulator.on('command', () => {
  const now = performance.now();
  if (lastCommandAt) maxCommandGap = Math.max(maxCommandGap, now-lastCommandAt);
  lastCommandAt = now;
  receivedCommands++;
});
try {
  server = await createDashboardServer({ httpPort: 0, jetsonPort: simulator.socket.address().port, simulation: true, maxVideoQueueBytes: 32768, log: false });
  const port = server.http.address().port;
  const origin = `http://127.0.0.1:${port}`;
  control = new WebSocket(`ws://127.0.0.1:${port}/ws`, { origin });
  control.on('message', bytes => {
    const value = JSON.parse(bytes.toString());
    if (value.type === 'hello' && value.delivery_ack === 1) control.send(JSON.stringify({v:1,type:'delivery_ready'}));
    if (value.type === 'delivery_probe') control.send(JSON.stringify({v:1,type:'delivery_ack',id:value.id}));
    if (value.type === 'state') state = value;
    if (value.type === 'hello') hello = value;
  });
  await once(control, 'open');
  await until(() => hello && state?.fpga?.ready && [...server.clients.values()].some(client => client.deliveryAckAt != null), '监控零速握手与页面消费确认');
  control.send(JSON.stringify({ v: 1, type: 'arm' }));
  heartbeat = setInterval(() => control.send(JSON.stringify({ v: 1, type: 'drive', seq: ++sequence, vx: 60, vy: 0, wz: 0 })), 50);
  await until(() => state?.armed && simulator.command.velocity[0] === 60, '真实UDP控制已生效');
  slow = new WebSocket(`ws://127.0.0.1:${port}/video`, { origin });
  await once(slow, 'open');
  // 测试中暂停真实 TCP 接收；不替换服务端 WebSocket，也不伪造 bufferedAmount。
  slow._socket.pause();
  await until(() => [...server.clients.values()].some(client => client.kind === 'video'), '真实视频连接已建立');
  const started = performance.now();
  const commandsBefore = receivedCommands;
  while (performance.now()-started < 10000) {
    const video = [...server.clients.entries()].find(([, client]) => client.kind === 'video');
    if (!video) break;
    maxBuffered = Math.max(maxBuffered, video[0].bufferedAmount);
    await delay(10);
  }
  assert.equal([...server.clients.values()].filter(client => client.kind === 'video').length, 0, '实际慢视频客户端必须被有界策略清理');
  assert.ok(receivedCommands-commandsBefore >= 3, '拥堵期间控制UDP持续接收');
  assert.equal(server.state().owner, hello.client_id, '视频回压不应解除健康的独立控制连接');
  assert.equal(simulator.command.velocity[0], 60);
  const stopAt = performance.now();
  control.send(JSON.stringify({ v: 1, type: 'stop', reason: 'congestion-acceptance' }));
  clearInterval(heartbeat);
  await until(() => !server.state().armed && simulator.command.velocity.every(value => value === 0), '视频拥堵后停车指令到达模拟Jetson', 500);
  const stopDelay = performance.now()-stopAt;
  assert.ok(stopDelay < 300, '停车请求不应排在旧画面后');
  console.log(JSON.stringify({ result: 'PASS', actual_tcp_backpressure: true, software_simulation_only: true, offered_video_fps: 300, slow_video_client_removed: true, max_observed_video_buffer_bytes: maxBuffered, control_packets_during_congestion: receivedCommands-commandsBefore, max_control_gap_ms: Math.round(maxCommandGap), stop_to_simulator_ms: Math.round(stopDelay) }, null, 2));
} finally {
  clearInterval(heartbeat);
  slow?._socket?.resume();
  slow?.terminate();
  control?.terminate();
  if (server) await server.close();
  await simulator.close();
}
