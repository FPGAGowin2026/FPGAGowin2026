import assert from 'node:assert/strict';
import dgram from 'node:dgram';
import fs from 'node:fs/promises';
import os from 'node:os';
import path from 'node:path';
import test from 'node:test';
import { setTimeout as delay } from 'node:timers/promises';
import { WebSocket, WebSocketServer } from 'ws';
import { DashboardBridge, JpegAssembler, createDashboardServer } from '../server.mjs';
import { createSimulator } from '../simulator.mjs';

const SIM_JPEG = await fs.readFile(new URL('../assets/simulator.jpg', import.meta.url));

function jpegPackets(id, jpeg = SIM_JPEG) {
  const count = Math.ceil(jpeg.length / 1200);
  return Array.from({ length: count }, (_, index) => {
    const header = Buffer.alloc(12);
    header.write('BXJV', 0, 4, 'ascii');
    header.writeUInt32BE(id >>> 0, 4);
    header.writeUInt16BE(index, 8);
    header.writeUInt16BE(count, 10);
    return Buffer.concat([header, jpeg.subarray(index * 1200, (index + 1) * 1200)]);
  });
}

class JsonSocket {
  constructor(socket, {deliveryReady = true} = {}) {
    this.socket = socket;
    this.deliveryReady = deliveryReady;
    this.autoAck = true;
    this.lastProbe = null;
    this.queue = [];
    this.waiters = [];
    socket.on('message', (data, isBinary) => {
      if (isBinary) return;
      let value;
      try { value = JSON.parse(data.toString()); } catch { return; }
      if (value.type === 'hello' && value.delivery_ack === 1 && this.deliveryReady) this.send({type:'delivery_ready'});
      if (value.type === 'delivery_probe') {
        this.lastProbe = value.id;
        if (this.autoAck) this.send({type:'delivery_ack',id:value.id});
      }
      const waiterIndex = this.waiters.findIndex(waiter => waiter.predicate(value));
      if (waiterIndex >= 0) {
        const [waiter] = this.waiters.splice(waiterIndex, 1);
        clearTimeout(waiter.timer);
        waiter.resolve(value);
      } else this.queue.push(value);
    });
  }

  send(value) { this.socket.send(JSON.stringify({ v: 1, ...value })); }
  close() { this.socket.close(); }

  waitFor(predicate, timeoutMs = 1500) {
    const index = this.queue.findIndex(predicate);
    if (index >= 0) return Promise.resolve(this.queue.splice(index, 1)[0]);
    return new Promise((resolve, reject) => {
      const waiter = { predicate, resolve, reject, timer: null };
      waiter.timer = setTimeout(() => {
        this.waiters.splice(this.waiters.indexOf(waiter), 1);
        reject(new Error('WebSocket message wait timed out'));
      }, timeoutMs);
      this.waiters.push(waiter);
    });
  }
}

async function openJsonSocket(port, options) {
  const socket = new WebSocket(`ws://127.0.0.1:${port}/ws`, { origin: `http://127.0.0.1:${port}` });
  const json = new JsonSocket(socket, options);
  await new Promise((resolve, reject) => {
    socket.once('open', resolve);
    socket.once('error', reject);
  });
  return json;
}

async function waitUntil(predicate, labelOrTimeout = 1500, labeledTimeoutMs = 1500) {
  const timeoutMs = typeof labelOrTimeout === 'number' ? labelOrTimeout : labeledTimeoutMs;
  const end = Date.now() + timeoutMs;
  while (Date.now() < end) {
    if (predicate()) return;
    await delay(10);
  }
  assert.fail(typeof labelOrTimeout === 'string' ? labelOrTimeout : 'condition did not become true before timeout');
}

async function rig(options = {}, simulatorOptions = {}) {
  const logDirectory = await fs.mkdtemp(path.join(os.tmpdir(), 'bx72-dashboard-test-'));
  const simulator = await createSimulator({ bind: '127.0.0.1', port: 0, quiet: true, ...simulatorOptions });
  const bridge = await createDashboardServer({
    bindHost: '127.0.0.1', httpPort: 0, udpBindHost: '127.0.0.1',
    jetsonHost: '127.0.0.1', jetsonPort: simulator.socket.address().port,
    simulation: true, logDirectory, linkTimeoutMs: 500, ...options,
  });
  const httpPort = bridge.http.address().port;
  return {
    bridge, simulator, httpPort, logDirectory,
    async close() {
      await bridge.close();
      await simulator.close();
      await fs.rm(logDirectory, { recursive: true, force: true });
    },
  };
}

async function waitReady(json) {
  const state = await json.waitFor(message => message.type === 'state' && message.jetson.online && message.fpga.ready);
  await json.waitFor(message => message.type === 'delivery_probe');
  return state;
}

// 仅替代 Jetson 高层服务回执，不模拟规划和设备执行。
function mockNavigationReplies(simulator, outcome = () => ({success:true,message:'已接收'})) {
  const received=[];
  simulator.socket.on('message', (packet, remote) => {
    let command;
    try { command=JSON.parse(packet.toString()); } catch { return; }
    if (command.type !== 'navigation_task') return;
    received.push(command);
    const response=outcome(command);
    if (!response) return;
    simulator.socket.send(Buffer.from(JSON.stringify({v:1,type:'navigation_task_result',
      session:command.session,bridge_id:simulator.bridgeId,request_id:command.request_id,
      action:command.action,unknown:false,...response})),remote.port,remote.address);
  });
  return received;
}

test('explicit stop and emergency commands work without vision while visual starts and offline requests are rejected', async t => {
  const testRig=await rig(); t.after(()=>testRig.close());
  testRig.bridge.vision = null;
  testRig.bridge.acceptVision = () => {};
  const json=await openJsonSocket(testRig.httpPort); t.after(()=>json.close());
  await waitReady(json);
  assert.equal(testRig.bridge.vision,null);
  for (const action of ['enable_ros','disable_ros','start']) {
    json.send({type:'mission_command',action,behavior:'search',class_id:'39',target_id:null,color:''});
    const result=await json.waitFor(message=>message.type==='mission_result' && message.action===action);
    assert.equal(result.success,false,`${action} still requires a vision interface`);
    assert.match(result.message,/未提供视觉任务接口/);
  }
  json.send({type:'mission_command',action:'stop'});
  const stopped=await json.waitFor(message=>message.type==='mission_result' && message.action==='stop');
  assert.equal(stopped.success,true,'a stop remains available without vision');
  json.send({type:'mission_command',action:'emergency_stop'});
  let result=await json.waitFor(message=>message.type==='mission_result' && message.action==='emergency_stop');
  assert.equal(result.success,true);
  assert.equal(testRig.simulator.emergencyStop,true,'the explicit request reaches the simulated bridge');
  json.send({type:'mission_command',action:'release_emergency_stop'});
  result=await json.waitFor(message=>message.type==='mission_result' && message.action==='release_emergency_stop');
  assert.equal(result.success,true);
  assert.equal(testRig.simulator.emergencyStop,false,'the explicit release reaches the simulated bridge');
  testRig.simulator.statusEnabled=false;
  testRig.simulator.telemetryEnabled=false;
  await waitUntil(()=>!testRig.bridge.jetsonOnline(),'simulated bridge should become offline');
  json.send({type:'mission_command',action:'emergency_stop'});
  result=await json.waitFor(message=>message.type==='mission_result' && message.action==='emergency_stop');
  assert.equal(result.success,true,'an emergency stop still reaches a responding bridge with stale status');
  assert.equal(testRig.simulator.emergencyStop,true);
  json.send({type:'mission_command',action:'release_emergency_stop'});
  result=await json.waitFor(message=>message.type==='mission_result' && message.action==='release_emergency_stop');
  assert.equal(result.success,false,'releasing the stop still requires an online bridge');
  assert.equal(testRig.simulator.emergencyStop,true);
});

test('stop falls back to the bridge when navigation telemetry is stale', async t => {
  const testRig=await rig(); t.after(()=>testRig.close());
  const json=await openJsonSocket(testRig.httpPort); t.after(()=>json.close());
  await waitReady(json);
  await waitUntil(()=>testRig.bridge.navigationState()?.boot_id > 0);
  // 设备任务可由网页以外的执行器创建；停车不依赖网页拥有任务。
  testRig.simulator.navigationTask={...testRig.simulator.navigationTask,status:5,mode:2,task_id:1,last_task_id:1};
  await waitUntil(()=>testRig.simulator.navigationTask.status===5);
  testRig.simulator.telemetryEnabled=false;
  testRig.bridge.vision=null;
  testRig.bridge.acceptVision=()=>{};
  await waitUntil(()=>testRig.bridge.navigationState()?.stale === true && testRig.bridge.jetsonOnline(), 'navigation telemetry expires');
  json.send({type:'stop',reason:'stale-navigation-test'});
  const result=await json.waitFor(message=>message.type==='mission_result' && message.action==='stop');
  assert.equal(result.success,true);
  await waitUntil(()=>testRig.simulator.navigationTask.status===9, 'the stop reaches the simulated bridge');
});

test('semantic search routes to one requester and rejects unknown text', async t => {
  const testRig=await rig(); t.after(()=>testRig.close());
  const requester=await openJsonSocket(testRig.httpPort), observer=await openJsonSocket(testRig.httpPort);
  t.after(()=>{requester.close();observer.close();});
  await waitReady(requester);
  await requester.waitFor(m=>m.type==='state' && m.vision && !m.vision.stale);
  requester.send({type:'semantic_command',text:'寻找红色水瓶'});
  const accepted=await requester.waitFor(m=>m.type==='semantic_result');
  assert.equal(accepted.accepted,true);
  const result=await requester.waitFor(m=>m.type==='mission_result' && m.request_id===accepted.request_id);
  assert.equal(result.success,true);
  await delay(100);
  assert.equal(observer.queue.some(m=>m.type==='mission_result' && m.request_id===accepted.request_id),false);
  await waitUntil(() => testRig.bridge.navigationState()?.boot_id > 0);
  requester.send({type:'local_task',action:'start',mode:'cooperative',allow_disconnect:false,
    goal_x_m:1,goal_y_m:0,hold_yaw_rad:0,final_yaw_rad:0,max_speed_mm_s:100,
    max_accel_mm_s2:200,tolerance_mm:100,yaw_tolerance_mrad:100,duration_s:30});
  const local=await requester.waitFor(m=>m.type==='mission_result' && m.action==='local_start');
  assert.equal(local.success,false,'旧协同航点必须迁移到 Jetson 高层接口');
  assert.match(local.message,/navigation_task/);
  assert.equal(testRig.simulator.rosAuthorized,true,'拒绝旧任务不会更改视觉任务');
  requester.send({type:'semantic_command',text:'飞到火星'});
  const rejected=await requester.waitFor(m=>m.type==='semantic_result');
  assert.equal(rejected.accepted,false);
  assert.match(rejected.error,/不支持/);
});

test('semantic coordinates go to Jetson as a high-level task with the default speed', async t => {
  const testRig=await rig({}, {navigationArchitecture:'grid'}); t.after(()=>testRig.close());
  const requester=await openJsonSocket(testRig.httpPort); t.after(()=>requester.close());
  await waitReady(requester);
  await waitUntil(() => {
    const nav=testRig.bridge.navigationState();
    const map=testRig.bridge.mapping.snapshot(performance.now());
    return nav && map.pose_fresh && map.domain===String(nav.boot_id)+':'+String(nav.odom_domain);
  }, 'fresh simulated pose and navigation domain', 3000);
  const commands=[];
  testRig.simulator.socket.on('message', (packet,remote) => {
    const command=JSON.parse(packet.toString());
    if (command.type==='navigation_task' && command.action==='start') {
      commands.push(command);
      // 模拟网桥高层服务回执，只验证传输，不模拟规划或真实设备执行。
      testRig.simulator.socket.send(Buffer.from(JSON.stringify({v:1,type:'navigation_task_result',
        session:command.session,bridge_id:testRig.simulator.bridgeId,request_id:command.request_id,
        action:'start',success:true,unknown:false,message:'目标已接收'})),remote.port,remote.address);
    }
  });
  requester.send({type:'semantic_command',text:'前往坐标1，2'});
  const accepted=await requester.waitFor(message=>message.type==='semantic_result');
  assert.equal(accepted.accepted,true);
  const result=await requester.waitFor(message=>message.type==='mission_result' && message.request_id===accepted.request_id);
  assert.equal(result.success,true);
  assert.equal(commands.length,1);
  assert.deepEqual([commands[0].frame,commands[0].goals,commands[0].max_speed_mm_s],
    ['odom',[{x:1,y:2}],400]);
});

test('control handoff waits for a fresh matching PC zero ACK and never queues motion', async t => {
  const testRig = await rig();
  t.after(() => testRig.close());
  const json = await openJsonSocket(testRig.httpPort);
  t.after(() => json.close());
  await waitReady(json);
  testRig.simulator.statusEnabled = false;
  json.send({type:'arm'});
  await waitUntil(() => testRig.bridge.pendingArm !== null);
  const bridge = testRig.bridge;
  const pending = {...bridge.pendingArm};
  json.send({type:'drive', seq:1, vx:50, vy:0, wz:0});
  await json.waitFor(message => message.type === 'event' && message.reason === 'drive-handshake-pending');
  assert.deepEqual(bridge.velocity, [0, 0, 0]);
  assert.equal(bridge.state().armed, false);
  const status = {...bridge.status, seq:(bridge.statusSequence + 1) >>> 0, ready:true,
    control_source:'pc', pc_timeout:false, fpga_ack_age:0, jetson_tx_mono_s:10,
    last_ack:{source:'pc', type:3, pc_session:pending.session, pc_seq:(pending.seq - 1) >>> 0,
      velocity:[0,0,0], jetson_rx_mono_s:10}};
  const accept = () => { status.seq = (bridge.statusSequence + 1) >>> 0; bridge.acceptStatus(status, performance.now()); };
  accept();
  assert.ok(bridge.pendingArm, 'old sequence cannot authorize');
  status.last_ack.pc_seq = pending.seq;
  status.last_ack.source = 'ros';
  accept();
  assert.ok(bridge.pendingArm, 'wrong source cannot authorize');
  status.last_ack.source = 'pc';
  status.last_ack.pc_session = 'old-session';
  accept();
  assert.ok(bridge.pendingArm, 'wrong session cannot authorize');
  status.last_ack.pc_session = pending.session;
  status.last_ack.velocity = [1,0,0];
  accept();
  assert.ok(bridge.pendingArm, 'nonzero ACK cannot authorize');
  status.last_ack.velocity = [0,0,0];
  accept();
  assert.equal(bridge.pendingArm, null);
  assert.deepEqual(bridge.velocity, [0,0,0]);
  assert.equal(bridge.state().armed, true);
});

test('control handoff times out safely without a matching ACK', async t => {
  const testRig = await rig();
  t.after(() => testRig.close());
  const json = await openJsonSocket(testRig.httpPort);
  t.after(() => json.close());
  await waitReady(json);
  testRig.simulator.statusEnabled = false;
  json.send({type:'arm'});
  await waitUntil(() => testRig.bridge.pendingArm !== null);
  await waitUntil(() => testRig.bridge.owner === null, 500);
  assert.deepEqual(testRig.bridge.velocity, [0,0,0]);
  assert.equal(testRig.bridge.state().armed, false);
});

test('closing a monitor does not interfere with an active manual owner', () => {
  const bridge=new DashboardBridge({log:false});
  const oldSocket={},manualSocket={};
  bridge.clients.set(oldSocket,{id:'old-task-page',kind:'json',events:[]});
  bridge.clients.set(manualSocket,{id:'manual-page',kind:'json',events:[]});
  bridge.owner={id:'manual-page',socket:manualSocket};
  const at=performance.now();
  bridge.pendingArm={session:bridge.session,seq:10,at};
  const cancels=[];
  bridge.sendMissionCommand=(_socket,action)=>cancels.push(action);
  bridge.acceptStatus({v:1,type:'status',session:bridge.session,bridge_id:'takeover',seq:1,
    ready:true,pc_timeout:false,fpga_ack_age:0,jetson_tx_mono_s:10,control_source:'pc',
    last_ack:{source:'pc',type:3,pc_session:bridge.session,pc_seq:10,
      velocity:[0,0,0],jetson_rx_mono_s:10}},at);
  assert.equal(bridge.pendingArm,null);
  bridge.onClientClose(oldSocket);
  assert.deepEqual(cancels,[],'old task disconnect must not inject a cancel into the new manual source');
  assert.equal(bridge.owner.socket,manualSocket);
});

test('vision telemetry remains valid while Jetson navigation owns motion', () => {
  const bridge = new DashboardBridge({log:false});
  bridge.bridgeId = 'jetson-nav';
  bridge.queueLatest = () => {};
  const message = {session:bridge.session, bridge_id:'jetson-nav', seq:1,
    mission_state:'IDLE', control_source:'navigation', ros_authorized:false,
    emergency_stop:false, stale:false, age_ms:0, width:800, height:480,
    frame_id:'camera', stamp:{sec:0,nanosec:0}, detections:[], selected_target:null};
  bridge.acceptVision(message, performance.now());
  assert.equal(bridge.vision?.control_source, 'navigation');
});

test('separate monitors accept their own status and reassemble shared navigation state', () => {
  const owner = new DashboardBridge({log:false}), viewer = new DashboardBridge({log:false});
  const at=performance.now(), bridgeId='multi-monitor';
  for (const bridge of [owner,viewer]) {
    bridge.queueLatest=()=>{};
    bridge.acceptStatus({v:1,type:'status',session:bridge.session,bridge_id:bridgeId,seq:1,
      ready:true,pc_timeout:false,fpga_ack_age:0,jetson_tx_mono_s:10,
      last_ack:{source:'pc',type:3,pc_session:owner.session,pc_seq:8,
        velocity:[0,0,0],jetson_rx_mono_s:10}},at);
    assert.equal(bridge.fpgaReady(),true);
    const common={v:1,type:'telemetry',bridge_id:bridgeId,epoch:1,valid:true,
      jetson_rx_mono_s:10,jetson_tx_mono_s:10,stream:'navigation_task',parts:2};
    const data={type:'navigation_task',sequence:1,time_us:1};
    bridge.acceptTelemetry({...common,part:1,data:{...data,state:{route:[{x:2,y:0}]}}},at);
    bridge.acceptTelemetry({...common,part:0,data:{...data,session:bridge.session,
      state:{active:true,route:[{x:1,y:0}]}}},at);
    assert.deepEqual(bridge.navigationTask.route,[{x:1,y:0},{x:2,y:0}]);
    assert.equal(bridge.navigationTask.active,true);
  }
  viewer.stopRequest={session:viewer.session,seq:1};
  assert.equal(viewer.stopState(at).fpga_acknowledged,false,
    'the real owner acknowledgement is not attributed to the observer');
});

test('explicit control and navigation requests echo only their own current claim nonce', () => {
  const bridge=new DashboardBridge({log:false}), sent=[];
  bridge.target={address:'127.0.0.1',port:6200};
  bridge.udp={address:()=>({port:1}),send:bytes=>sent.push(JSON.parse(bytes.toString()))};
  bridge.status={session:bridge.session,claim_nonce:'fresh-claim'};
  bridge.owner={id:'owner'};
  bridge.sendControl([0,0,0],false,'manual-zero-handshake');
  assert.equal(sent.at(-1).claim_nonce,'fresh-claim');
  bridge.owner=null;
  bridge.sendNavigationTask(null,{action:'start',claim_nonce:'injected'});
  assert.equal(sent.at(-1).claim_nonce,'fresh-claim');
  bridge.sendMissionCommand(null,'stop',{}, {skipNavigation:true});
  assert.equal(sent.at(-1).claim_nonce,'fresh-claim');
  bridge.status={session:'another-session',claim_nonce:'not-ours'};
  bridge.sendControl([0,0,0],false,'idle-zero');
  assert.equal('claim_nonce' in sent.at(-1),false);
});

test('offline emergency stop is sent, but a missing reply remains unknown', () => {
  const bridge=new DashboardBridge({log:false}), sent=[], results=[];
  bridge.target={address:'127.0.0.1',port:6200};
  bridge.udp={address:()=>({port:1}),send:bytes=>sent.push(JSON.parse(bytes.toString()))};
  bridge.deliverMissionResult=result=>results.push(result);
  bridge.queueLatest=()=>{};
  const request=bridge.sendMissionCommand({},'emergency_stop',{}, {skipNavigation:true});
  assert.ok(request);
  assert.equal(sent[0].action,'emergency_stop');
  assert.equal(results.length,0,'sending alone does not claim the robot stopped');
  const pending=bridge.pendingMission.get(request);
  bridge.tick(pending.at+2501);
  assert.equal(results.at(-1).action,'emergency_stop');
  assert.equal(results.at(-1).unknown,true);
  assert.match(results.at(-1).message,/结果未知/);
});

test('losing the manual owner clears authorization and later heartbeats do not claim', () => {
  const bridge=new DashboardBridge({log:false}), sent=[];
  bridge.target={address:'127.0.0.1',port:6200};
  bridge.udp={address:()=>({port:1}),send:bytes=>sent.push(JSON.parse(bytes.toString()))};
  bridge.queueLatest=()=>{};
  bridge.owner={id:'page',socket:{}};
  bridge.bridgeId='bridge';
  const at=performance.now();
  bridge.acceptStatus({v:1,type:'status',session:bridge.session,bridge_id:'bridge',seq:1,
    jetson_tx_mono_s:10,ready:true,pc_timeout:false,pc_control_owner:true,claim_nonce:'old'},at);
  assert.ok(bridge.owner);
  bridge.acceptStatus({v:1,type:'status',session:bridge.session,bridge_id:'bridge',seq:2,
    jetson_tx_mono_s:10.1,ready:true,pc_timeout:false,pc_control_owner:false,claim_nonce:'new'},at+1);
  assert.equal(bridge.owner,null);
  assert.equal(sent.at(-1).take_control,false);
  assert.notEqual(sent.at(-1).claim,true);
  bridge.sendControl([0,0,0],false,'idle-zero');
  assert.equal(sent.at(-1).take_control,false);
  assert.notEqual(sent.at(-1).claim,true);
});

test('HTTP + WebSocket drive traverses PC UDP, forwards all telemetry, and delivers complete JPEG', async t => {
  const testRig = await rig();
  t.after(() => testRig.close());
  const response = await fetch(`http://127.0.0.1:${testRig.httpPort}/`);
  assert.equal(response.status, 200);
  assert.match(await response.text(), /BX72/i);
  assert.equal((await fetch(`http://127.0.0.1:${testRig.httpPort}/not-allowed`)).status, 404);

  const json = await openJsonSocket(testRig.httpPort);
  t.after(() => json.close());
  const hello = await json.waitFor(message => message.type === 'hello');
  assert.equal(hello.simulation, true);
  assert.equal(hello.limits.speed, 400);
  assert.equal(hello.limits.turn, 800);
  await waitReady(json);

  const telemetry = new Map();
  while (telemetry.size < 6) {
    const envelope = await json.waitFor(message => message.type === 'telemetry');
    assert.ok(envelope.pc_rx_wall_ms > 0);
    assert.ok(envelope.pc_rx_mono_ms >= 0);
    assert.equal(envelope.source_protocol, 'BX72');
    assert.equal(envelope.source_version, 1);
    assert.equal(envelope.valid, true);
    telemetry.set(envelope.stream, envelope.data);
  }
  assert.deepEqual([...telemetry.keys()].sort(), ['fusion', 'health', 'imu', 'lidar', 'motor', 'navigation']);
  assert.ok(Number.isSafeInteger(telemetry.get('navigation').now_us));
  assert.equal(telemetry.get('navigation').status, 0);
  assert.equal(telemetry.get('navigation').last_task_id, 0);
  assert.equal(telemetry.get('imu').flags, 15);
  assert.equal(telemetry.get('imu').who_am_i, 0x70);
  assert.equal(telemetry.get('health').imu_who_am_i, 0x70);
  assert.equal(telemetry.get('motor').motor_channel_map, 0xd2);
  assert.equal(telemetry.get('motor').encoder_channel_map, 0xd2);
  assert.equal(telemetry.get('motor').motor_invert, 0x0b);
  assert.equal(telemetry.get('motor').encoder_invert, 0x0a);
  const motorTicks = telemetry.get('motor').count_ticks;
  assert.deepEqual(
    [1, 2, 3, 4].map(index => telemetry.get('motor').physical_count_ticks[`MOTOR${index}`]),
    [motorTicks.B, motorTicks.C, motorTicks.A, motorTicks.D],
  );
  assert.equal(telemetry.get('fusion').flags & (1 << 11), 0);

  const video = new WebSocket(`ws://127.0.0.1:${testRig.httpPort}/video`, { origin: `http://127.0.0.1:${testRig.httpPort}` });
  t.after(() => video.close());
  await new Promise((resolve, reject) => { video.once('open', resolve); video.once('error', reject); });
  const jpeg = await new Promise((resolve, reject) => {
    const timer = setTimeout(() => reject(new Error('JPEG frame wait timed out')), 1500);
    video.once('message', data => { clearTimeout(timer); resolve(Buffer.from(data)); });
  });
  assert.equal(jpeg[0], 0xff);
  assert.equal(jpeg[1], 0xd8);
  assert.equal(jpeg.at(-2), 0xff);
  assert.equal(jpeg.at(-1), 0xd9);

  json.send({ type: 'arm' });
  await json.waitFor(message => message.type === 'event' && message.event === 'armed');
  json.send({ type: 'drive', seq: 1, vx: 100, vy: 100, wz: 200 });
  const command = await testRig.simulator.waitForCommand(snapshot => snapshot.command.velocity.some(Boolean));
  assert.deepEqual(command.command.velocity, [100, 100, 200]);
  const movingMotor = await json.waitFor(message => message.type === 'telemetry' && message.stream === 'motor' && message.data.odometry.wz_mrad_s === 200);
  assert.deepEqual(movingMotor.data.wheel_mm_s, { A: -35, B: 235, C: 165, D: 35 });
  json.send({ type: 'drive', seq: 2, vx: 400, vy: 0, wz: 800 });
  const highCommand = await testRig.simulator.waitForCommand(snapshot => snapshot.command.velocity[0] === 400 && snapshot.command.velocity[2] === 800);
  assert.deepEqual(highCommand.command.velocity, [400, 0, 800]);
});

test('compatible legacy point goes via the Jetson goal interface and does not claim execution', async t => {
  const testRig = await rig(); t.after(() => testRig.close());
  const json = await openJsonSocket(testRig.httpPort); t.after(() => json.close());
  await waitReady(json);
  await waitUntil(() => testRig.bridge.navigationState()?.boot_id > 0);
  const commands=mockNavigationReplies(testRig.simulator);
  json.send({type:'local_task',action:'start',mode:'independent',position_only:true,
    allow_disconnect:true,goal_x_m:1.25,goal_y_m:-.5,max_speed_mm_s:80,
    max_accel_mm_s2:200,tolerance_mm:64,yaw_tolerance_mrad:100,duration_s:30});
  const result=await json.waitFor(message => message.type==='mission_result' && message.action==='local_start');
  assert.equal(result.success,true);
  assert.equal(result.message,'已接收');
  assert.equal(result.task_id,undefined,'goal receipt does not mean device execution');
  assert.equal(commands.length,1);
  assert.deepEqual(commands[0].goals,[{x:1.25,y:-.5}]);
  assert.equal(commands[0].type,'navigation_task');
  assert.equal(commands[0].frame,'odom');
  assert.equal(commands[0].task_id,undefined,'only Jetson assigns local transaction IDs');
  assert.equal(testRig.simulator.navigationTask.status,0);
});

test('stalled page consumption closes only the page; Jetson navigation remains independent', async t => {
  const testRig=await rig(); t.after(()=>testRig.close());
  const owner=await openJsonSocket(testRig.httpPort), observer=await openJsonSocket(testRig.httpPort,{deliveryReady:false});
  t.after(()=>{owner.close();observer.close();});
  await waitReady(owner);
  await observer.waitFor(message=>message.type==='state' && message.fpga.ready);
  const commands=mockNavigationReplies(testRig.simulator);
  owner.send({type:'navigation_task',action:'start',frame:'odom',domain:'1:1',generation:null,
    goals:[{x:1,y:0}],max_speed_mm_s:100,max_accel_mm_s2:200,tolerance_mm:100,
    yaw_tolerance_mrad:100,duration_s:30});
  assert.equal((await owner.waitFor(message=>message.type==='mission_result' && message.action==='navigation_start')).success,true);
  testRig.bridge.navigationTask={active:true,status:'following',goals:[{x:1,y:0}],route:[]};
  testRig.bridge.navigationTaskAt=performance.now();
  const serverSocket=[...testRig.bridge.clients].find(([,client])=>client.id===owner.queue.find(m=>m.type==='hello')?.client_id)?.[0];
  assert.ok(serverSocket);
  owner.autoAck=false;
  const oldId=owner.lastProbe;
  await waitUntil(()=>{const pending=testRig.bridge.clients.get(serverSocket)?.deliveryPending;return pending && pending.id!==oldId;});
  const pending=testRig.bridge.clients.get(serverSocket).deliveryPending;
  owner.send({type:'delivery_ack',id:oldId});
  await delay(100);
  assert.equal(testRig.bridge.clients.get(serverSocket).deliveryPending.id,pending.id);
  const closed=await new Promise(resolve=>owner.socket.once('close',(code,reason)=>resolve({code,reason:reason.toString()})));
  assert.deepEqual(closed,{code:1013,reason:'client consumption stalled'});
  await delay(100);
  assert.deepEqual(commands.map(row=>row.action),['start'],'closing a page never schedules a navigation cancel');
  assert.equal(testRig.bridge.navigationTask.active,true);
  assert.equal(testRig.bridge.clients.size,1);
  const reconnected=await openJsonSocket(testRig.httpPort); t.after(()=>reconnected.close());
  await waitReady(reconnected);
  assert.equal(testRig.bridge.owner,null,'reconnection never re-arms manual control');
});

test('legacy observer cannot change shared state or interrupt a confirmed simulator owner', async t => {
  const testRig=await rig(); t.after(()=>testRig.close());
  const owner=await openJsonSocket(testRig.httpPort), observer=await openJsonSocket(testRig.httpPort,{deliveryReady:false});
  t.after(()=>{owner.close();observer.close();});
  await waitReady(owner);
  await observer.waitFor(message=>message.type==='state' && message.fpga.ready);
  owner.send({type:'arm'});
  let sequence=0;
  const heartbeat=setInterval(()=>owner.send({type:'drive',seq:++sequence,vx:0,vy:0,wz:0}),50);
  t.after(()=>clearInterval(heartbeat));
  await owner.waitFor(message=>message.type==='event' && message.event==='armed');
  const ownerId=testRig.bridge.owner.id, target=testRig.bridge.target;
  const generation=testRig.bridge.mapping.snapshot(performance.now()).generation;
  observer.send({type:'mission_command',action:'enable_ros'});
  assert.equal((await observer.waitFor(message=>message.type==='mission_result' && message.action==='enable_ros')).success,false);
  observer.send({type:'map_reset'});
  assert.equal((await observer.waitFor(message=>message.event==='map_reset_rejected')).reason,'fpga-map-reset-unavailable');
  owner.send({type:'map_reset'});
  assert.equal((await owner.waitFor(message=>message.event==='map_reset_rejected')).reason,'fpga-map-reset-unavailable',
    'an authorized page also cannot rebuild the actual FPGA map');
  observer.send({type:'configure',jetson:'127.0.0.1',port:1});
  assert.equal((await observer.waitFor(message=>message.event==='configure_rejected')).reason,'page-consumption-unconfirmed');
  assert.equal(testRig.bridge.owner.id,ownerId);
  assert.equal(testRig.bridge.target,target);
  assert.equal(testRig.bridge.mapping.snapshot(performance.now()).generation,generation);
});

test('legacy observer can still issue explicit safety stops without consumption acknowledgement', async t => {
  const testRig=await rig(); t.after(()=>testRig.close());
  const owner=await openJsonSocket(testRig.httpPort), observer=await openJsonSocket(testRig.httpPort,{deliveryReady:false});
  t.after(()=>{owner.close();observer.close();});
  await waitReady(owner);
  await observer.waitFor(message=>message.type==='state' && message.fpga.ready && message.vision);
  for (const action of ['stop','cancel']) {
    testRig.simulator.navigationTask={...testRig.simulator.navigationTask,status:5,mode:2,task_id:1,last_task_id:1};
    observer.send({type:'local_task',action});
    const result=await observer.waitFor(message=>message.type==='mission_result' && message.action==='local_'+action);
    assert.equal(result.success,true,'generic stop reaches the device even without a page lease');
    await waitUntil(()=>testRig.simulator.navigationTask.status===9);
  }
  for (const action of ['disable_ros','stop','emergency_stop']) {
    observer.send({type:'mission_command',action});
    const result=await observer.waitFor(message=>message.type==='mission_result' && message.action===action);
    assert.equal(result.success,true);
  }
  assert.equal(testRig.simulator.emergencyStop,true);
  const sequence=testRig.bridge.seq;
  observer.send({type:'stop',reason:'legacy-safety-stop'});
  await waitUntil(()=>testRig.bridge.stopRequest?.seq>sequence);
  assert.deepEqual(testRig.bridge.velocity,[0,0,0]);
  assert.equal(observer.socket.readyState,WebSocket.OPEN);
});

test('Jetson validates compatible goals and legacy unsupported semantics never bypass it', async t => {
  const testRig=await rig(); t.after(()=>testRig.close());
  const json=await openJsonSocket(testRig.httpPort); t.after(()=>json.close());
  await waitReady(json);
  await waitUntil(()=>testRig.bridge.navigationState()?.boot_id > 0);
  const commands=mockNavigationReplies(testRig.simulator, command =>
    command.max_speed_mm_s > 400 ? {success:false,message:'max_speed_mm_s 超出范围'} : {success:true,message:'已接收'});
  const fields={type:'local_task',action:'start',mode:'independent',position_only:true,
    goal_x_m:1,goal_y_m:0,max_accel_mm_s2:200,tolerance_mm:100,yaw_tolerance_mrad:100,duration_s:30};
  json.send({...fields,hold_yaw_rad:0,max_speed_mm_s:100});
  const unsupported=await json.waitFor(item=>item.type==='mission_result' && item.action==='local_start');
  assert.equal(unsupported.success,false); assert.match(unsupported.message,/navigation_task/);
  assert.equal(commands.length,0);
  json.send({...fields,max_speed_mm_s:401});
  const invalid=await json.waitFor(item=>item.type==='mission_result' && item.action==='local_start');
  assert.equal(invalid.success,false); assert.match(invalid.message,/max_speed_mm_s/);
  assert.equal(commands[0].max_speed_mm_s,401,'parameters are forwarded to the Jetson entry');
  json.send({...fields,max_speed_mm_s:400});
  const accepted=await json.waitFor(item=>item.type==='mission_result' && item.action==='local_start');
  assert.equal(accepted.success,true); assert.equal(commands[1].max_speed_mm_s,400);
});

test('navigation watermark is displayed from device telemetry without allocating transaction IDs', () => {
  const bridge=new DashboardBridge({log:false});
  bridge.jetsonOnline=()=>true;
  const at=performance.now();
  const nav={boot_id:77,odom_domain:4,task_id:12,last_task_id:41,now_us:1000};
  bridge.telemetryLatest.set('navigation',{data:nav,pc_rx_mono_ms:at});
  assert.equal(bridge.navigationState(at).task_id_watermark_known,true);
  assert.equal(bridge.navigationState(at).last_task_id,41);
  const legacy={...nav}; delete legacy.last_task_id;
  bridge.telemetryLatest.set('navigation',{data:legacy,pc_rx_mono_ms:at});
  assert.equal(bridge.navigationState(at).task_id_watermark_known,false);
  assert.equal(typeof bridge.nextNavigationTaskId,'undefined');
});

test('polar terminal state is displayed without mutating local task ownership', () => {
  const bridge=new DashboardBridge({log:false});
  const base={v:1,type:'telemetry',bridge_id:'board',stream:'navigation',epoch:1,valid:true,
    jetson_rx_mono_s:1,jetson_tx_mono_s:1.01};
  const nav={type:'navigation',sequence:1,time_us:1000,protocol:'nav_polar_v2',boot_id:77,
    odom_domain:4,task_id:41,last_task_id:41,now_us:1000,status:7,active_task:false};
  bridge.acceptTelemetry({...base,data:nav},performance.now());
  assert.equal(bridge.navigationState().status,7);
  assert.equal(bridge.navigationState().last_task_id,41);
  assert.equal('localTask' in bridge,false);
});

test('grid wait-auto and cancellation are shown directly from Jetson device telemetry', () => {
  const bridge=new DashboardBridge({log:false});
  const base={v:1,type:'telemetry',bridge_id:'board',stream:'navigation',epoch:1,valid:true,
    jetson_rx_mono_s:1,jetson_tx_mono_s:1.01};
  const nav={type:'navigation',sequence:1,time_us:1000,protocol:'nav_grid_v1',boot_id:77,
    odom_domain:4,task_id:12,last_task_id:12,now_us:1000,status:9,active_task:true};
  bridge.acceptTelemetry({...base,data:nav},performance.now());
  assert.equal(bridge.navigationState().active_task,true);
  bridge.acceptTelemetry({...base,data:{...nav,sequence:2,task_id:13,last_task_id:13,status:7,active_task:false}},performance.now());
  assert.equal(bridge.navigationState().active_task,false);
  assert.equal(bridge.navigationState().last_task_id,13);
});

test('grid simulator sends architecture and direction preview through the existing server streams', async t => {
  const testRig=await rig({}, {navigationArchitecture:'grid'});t.after(()=>testRig.close());
  const json=await openJsonSocket(testRig.httpPort);t.after(()=>json.close());
  await json.waitFor(message=>message.type==='hello');await waitReady(json);
  await waitUntil(()=>testRig.bridge.navigationState()?.protocol==='nav_grid_v1');
  await waitUntil(()=>testRig.bridge.mapping.snapshot(performance.now()).available===true);
  const map=await (await fetch('http://127.0.0.1:'+testRig.httpPort+'/map.json')).json();
  assert.equal(map.source,'fpga'); assert.equal(map.protocol,'fpga_grid_view_v1');
  assert.equal(map.width,64); assert.equal(map.height,64); assert.ok(map.counts.free>0);
  // 真实规划位于设备；模拟器直接发布已经存在的设备状态。
  testRig.simulator.navigationTask={...testRig.simulator.navigationTask,status:3,mode:3,
    task_id:1,last_task_id:1,goal_x_q16:2*65536,goal_y_q16:0};
  await waitUntil(()=>testRig.bridge.navigationState()?.status===3);
  const plan=await json.waitFor(message=>message.type==='telemetry' && message.stream==='local_plan' && message.data.valid);
  assert.equal(plan.data.grid_direction_preview,true); assert.equal(plan.data.map_version,1);
  assert.equal(plan.data.points.length,2); assert.equal(testRig.bridge.navigationState().candidates_evaluated,7);
  json.send({type:'stop',reason:'test-stop'});
  assert.equal((await json.waitFor(message=>message.type==='mission_result' && message.action==='stop')).success,true);
  await waitUntil(()=>testRig.bridge.navigationState()?.status===7);
});

test('simulated Jetson rejects a zero-velocity legacy auto request', async t => {
  const simulator = await createSimulator({
    bind: '127.0.0.1', port: 0, quiet: true,
    statusIntervalMs: 1000, telemetryIntervalMs: 1000, videoFps: 1,
  });
  const sender = dgram.createSocket('udp4');
  await new Promise((resolve, reject) => {
    sender.once('error', reject);
    sender.bind(0, '127.0.0.1', resolve);
  });
  t.after(async () => {
    await simulator.close();
    await new Promise(resolve => sender.close(resolve));
  });
  const send = value => new Promise((resolve, reject) => {
    sender.send(JSON.stringify(value), simulator.socket.address().port, '127.0.0.1', error => error ? reject(error) : resolve());
  });
  const rejected = new Promise(resolve => simulator.on('event', event => {
    if (event.event === 'command_rejected' && event.reason === 'legacy-auto-disabled') resolve(event);
  }));
  const session = 'sim-auto-timeout-test';
  await send({ v: 1, type: 'control', session, seq: 0, vx: 0, vy: 0, wz: 0, auto: false });
  await waitUntil(() => simulator.session === session);
  await send({ v: 1, type: 'control', session, seq: 1, vx: 0, vy: 0, wz: 0, auto: true });
  await rejected;
  assert.equal(simulator.autoRequested, false);
  assert.equal(simulator.command.auto, false);
  assert.deepEqual(simulator.command.velocity, [0, 0, 0]);
  assert.equal(simulator.commandSequence, 0,'the rejected mode request does not refresh the control lease');
});

test('stop evidence needs a matching fresh zero ACK and fresh four-wheel telemetry', async t => {
  const testRig = await rig();
  t.after(() => testRig.close());
  const json = await openJsonSocket(testRig.httpPort);
  t.after(() => json.close());
  await json.waitFor(message => message.type === 'hello');
  await waitReady(json);
  json.send({ type: 'arm' });
  await json.waitFor(message => message.type === 'event' && message.event === 'armed');
  json.send({ type: 'drive', seq: 1, vx: 40, vy: 0, wz: 0 });
  await testRig.simulator.waitForCommand(snapshot => snapshot.command.velocity[0] === 40);
  json.send({ type: 'stop', reason: 'stop-evidence-test' });
  await waitUntil(() => testRig.bridge.stopRequest !== null);

  const request = testRig.bridge.stopRequest;
  const status = testRig.bridge.status;
  testRig.bridge.lastStatusAt = performance.now();
  testRig.bridge.status = {
    ...status,
    jetson_tx_mono_s: 12,
    last_ack: {
      type: 3, fpga_seq: 23, pc_seq: request.seq, pc_session: request.session,
      velocity: [1, 0, 0], jetson_rx_mono_s: 12,
    },
  };
  testRig.bridge.telemetryLatest.set('motor', {
    pc_rx_mono_ms: performance.now(),
    data: { wheel_mm_s: { A: 0, B: 0, C: 0, D: 0 } },
  });
  assert.equal(testRig.bridge.state().stop.fpga_acknowledged, false, 'a nonzero ACK cannot prove stopping');
  assert.equal(testRig.bridge.state().stop.wheels_low, true);

  testRig.bridge.status.last_ack.velocity = [0, 0, 0];
  testRig.bridge.status.last_ack.pc_seq = (request.seq + 1) >>> 0;
  testRig.bridge.status.last_ack.pc_session = 'another-monitor';
  assert.equal(testRig.bridge.state().stop.fpga_acknowledged, false,
    'another monitor or controller cannot acknowledge this viewer stop');
  testRig.bridge.status.last_ack.pc_session = request.session;
  assert.equal(testRig.bridge.state().stop.fpga_acknowledged, true);
  testRig.bridge.status.last_ack.jetson_rx_mono_s = 11.6;
  assert.equal(testRig.bridge.state().stop.fpga_acknowledged, null, 'a stale ACK is unknown');

  testRig.bridge.telemetryLatest.get('motor').data.wheel_mm_s.C = 6;
  assert.equal(testRig.bridge.state().stop.wheels_low, false);
  testRig.bridge.telemetryLatest.get('motor').pc_rx_mono_ms -= 1001;
  assert.equal(testRig.bridge.state().stop.wheels_low, null, 'stale motor telemetry is unknown');
});

test('ping does not extend the 200 ms drive lease; stale input zeros and releases control', async t => {
  const testRig = await rig();
  t.after(() => testRig.close());
  const json = await openJsonSocket(testRig.httpPort);
  t.after(() => json.close());
  await json.waitFor(message => message.type === 'hello');
  await waitReady(json);
  json.send({ type: 'arm' });
  await json.waitFor(message => message.type === 'event' && message.event === 'armed');
  json.send({ type: 'drive', seq: 10, vx: 50, vy: 0, wz: 0 });
  const moving = await testRig.simulator.waitForCommand(snapshot => snapshot.command.velocity[0] === 50);
  for (let index = 0; index < 7; index += 1) {
    await delay(35);
    json.send({ type: 'ping' });
  }
  await testRig.simulator.waitForCommand(snapshot => snapshot.command.velocity.every(value => value === 0) && snapshot.command.seq > moving.command.seq);
  await waitUntil(() => testRig.bridge.owner === null);
});

test('page close clears control and a new page cannot inherit the old authorization', async t => {
  const testRig = await rig();
  t.after(() => testRig.close());
  const first = await openJsonSocket(testRig.httpPort);
  await first.waitFor(message => message.type === 'hello');
  await waitReady(first);
  first.send({ type: 'arm' });
  await first.waitFor(message => message.type === 'event' && message.event === 'armed');
  first.send({ type: 'drive', seq: 1, vx: 65, vy: 0, wz: 0 });
  await testRig.simulator.waitForCommand(snapshot => snapshot.command.velocity[0] === 65);
  first.close();
  await testRig.simulator.waitForCommand(snapshot => snapshot.command.velocity.every(value => value === 0));
  await waitUntil(() => testRig.bridge.owner === null);

  const reconnected = await openJsonSocket(testRig.httpPort);
  t.after(() => reconnected.close());
  await reconnected.waitFor(message => message.type === 'hello');
  const state = await reconnected.waitFor(message => message.type === 'state' && message.jetson.online);
  assert.equal(state.armed, false);
  assert.equal(state.owner, null);
});

test('UDP stream loss and a new bridge id clear authorization across reconnection', async t => {
  const testRig = await rig();
  t.after(() => testRig.close());
  const json = await openJsonSocket(testRig.httpPort);
  t.after(() => json.close());
  await json.waitFor(message => message.type === 'hello');
  await waitReady(json);
  json.send({ type: 'arm' });
  await json.waitFor(message => message.type === 'event' && message.event === 'armed');
  json.send({ type: 'drive', seq: 1, vx: 40, vy: 0, wz: 0 });
  await testRig.simulator.waitForCommand(snapshot => snapshot.command.velocity[0] === 40);

  testRig.simulator.stopStreams();
  await waitUntil(() => testRig.bridge.owner === null && !testRig.bridge.jetsonOnline(), 1200);
  const previousBridge = testRig.simulator.bridgeId;
  testRig.simulator.restartBridge();
  await json.waitFor(message => message.type === 'state' && message.jetson.online && message.fpga.ready, 2000);
  assert.notEqual(testRig.simulator.bridgeId, previousBridge);
  assert.equal(testRig.bridge.owner, null);
  const command = testRig.simulator.getSnapshot().command;
  assert.deepEqual(command.velocity, [0, 0, 0]);
});

test('only one page can arm, while a monitor page can stop the controller', async t => {
  const testRig = await rig();
  t.after(() => testRig.close());
  const one = await openJsonSocket(testRig.httpPort);
  const two = await openJsonSocket(testRig.httpPort);
  t.after(() => { one.close(); two.close(); });
  await Promise.all([one.waitFor(message => message.type === 'hello'), two.waitFor(message => message.type === 'hello')]);
  await Promise.all([waitReady(one), waitReady(two)]);
  one.send({ type: 'arm' });
  const armed = await one.waitFor(message => message.type === 'event' && message.event === 'armed');
  two.send({ type: 'arm' });
  const denied = await two.waitFor(message => message.type === 'event' && message.event === 'arm_rejected');
  assert.match(denied.reason, /another-page/);
  assert.equal(testRig.bridge.owner.id, armed.owner);
  two.send({ type: 'stop', reason: 'monitor-stop' });
  await testRig.simulator.waitForCommand(snapshot => snapshot.command.velocity.every(value => value === 0));
  await waitUntil(() => testRig.bridge.owner === null);
});

test('websocket legacy auto is explicitly rejected while local task mode owns autonomy', async t => {
  const testRig = await rig();
  t.after(() => testRig.close());
  const json = await openJsonSocket(testRig.httpPort);
  t.after(() => json.close());
  await json.waitFor(message => message.type === 'hello');
  await waitReady(json);
  json.send({ type: 'arm' });
  await json.waitFor(message => message.type === 'event' && message.event === 'armed');
  json.send({ type: 'drive', seq: 1, vx: 0, vy: 0, wz: 0 });
  json.send({ type: 'auto', enabled: true });
  const rejected = await json.waitFor(message => message.type === 'event' && message.event === 'auto_rejected');
  assert.match(rejected.reason,/局部导航/);
  assert.equal(testRig.simulator.command.auto,false);
  assert.equal(testRig.simulator.autoRequested,false);
  json.send({ type: 'drive', seq: 2, vx: 1, vy: 0, wz: 0 });
  const manual = await testRig.simulator.waitForCommand(snapshot => snapshot.command.velocity[0] === 1);
  assert.equal(manual.command.auto,false,'a rejected automatic request does not change manual packet semantics');
  json.send({ type: 'stop', reason: 'legacy-auto-rejected' });
  await waitUntil(() => testRig.bridge.owner === null);
});

test('equal vx and wz values still use the lower translation hard limit', async t => {
  const testRig = await rig();
  t.after(() => testRig.close());
  const json = await openJsonSocket(testRig.httpPort);
  t.after(() => json.close());
  await json.waitFor(message => message.type === 'hello');
  await waitReady(json);
  json.send({ type: 'arm' });
  await json.waitFor(message => message.type === 'event' && message.event === 'armed');
  json.send({ type: 'drive', seq: 1, vx: 450, vy: 0, wz: 450 });
  await waitUntil(() => testRig.bridge.owner === null, 1000);
  const snapshot = testRig.simulator.getSnapshot();
  assert.deepEqual(snapshot.command.velocity, [0, 0, 0]);
});

test('UDP packets from a different source port cannot replace Jetson state', async t => {
  const testRig = await rig();
  t.after(() => testRig.close());
  const json = await openJsonSocket(testRig.httpPort);
  t.after(() => json.close());
  await json.waitFor(message => message.type === 'hello');
  await waitReady(json);
  const originalBridge = testRig.bridge.bridgeId;
  const spoof = dgram.createSocket('udp4');
  t.after(() => spoof.close());
  const packet = Buffer.from(JSON.stringify({
    v: 1, type: 'status', session: testRig.bridge.session, bridge_id: 'forged-bridge', seq: 1,
    jetson_tx_mono_s: 1, ready: true, pc_timeout: false, fpga_ack_age: 0,
  }));
  const rejectedBefore = testRig.bridge.rejections;
  await new Promise((resolve, reject) => spoof.send(packet, testRig.bridge.udp.address().port, '127.0.0.1', error => error ? reject(error) : resolve()));
  await waitUntil(() => testRig.bridge.rejections > rejectedBefore);
  assert.equal(testRig.bridge.bridgeId, originalBridge);
});

test('WebSocket control connections reject a non-local Origin', async t => {
  const testRig = await rig();
  t.after(() => testRig.close());
  const remoteHost = `192.168.1.239:${testRig.httpPort}`;
  const sameOrigin = new WebSocket(`ws://127.0.0.1:${testRig.httpPort}/ws`,
    {origin:`http://${remoteHost}`,headers:{Host:remoteHost}});
  t.after(() => sameOrigin.terminate());
  await new Promise((resolve,reject) => { sameOrigin.once('open',resolve); sameOrigin.once('error',reject); });
  const socket = new WebSocket(`ws://127.0.0.1:${testRig.httpPort}/ws`, { origin: 'http://evil.example' });
  t.after(() => socket.terminate());
  await new Promise(resolve => {
    socket.once('unexpected-response', (_request, response) => { assert.equal(response.statusCode, 403); resolve(); });
    socket.once('error', () => resolve());
  });
});

test('configure DNS completion rechecks ownership before changing the shared target', async t => {
  const testRig = await rig({ simulation: false });
  t.after(() => testRig.close());
  const json = await openJsonSocket(testRig.httpPort);
  t.after(() => json.close());
  await json.waitFor(message => message.type === 'hello');
  await waitReady(json);

  let release;
  let entered;
  const enteredPromise = new Promise(resolve => { entered = resolve; });
  const gate = new Promise(resolve => { release = resolve; });
  const originalTarget = { ...testRig.bridge.target };
  testRig.bridge.resolveTarget = async host => {
    entered();
    await gate;
    return { host, address: '127.0.0.1', port: originalTarget.port + 1 };
  };
  json.send({ type: 'configure', jetson: 'localhost', port: originalTarget.port + 1 });
  await enteredPromise;
  json.send({ type: 'arm' });
  await json.waitFor(message => message.type === 'event' && message.event === 'armed');
  release();
  const rejected = await json.waitFor(message => message.type === 'event' && message.event === 'configure_rejected');
  assert.match(rejected.reason, /owner-changed/);
  assert.equal(testRig.bridge.target.port, originalTarget.port);
});

test('simulation mode refuses a configure request outside its startup endpoint', async t => {
  const testRig = await rig();
  t.after(() => testRig.close());
  const json = await openJsonSocket(testRig.httpPort);
  t.after(() => json.close());
  await json.waitFor(message => message.type === 'hello');
  await waitReady(json);

  const startupTarget = { ...testRig.bridge.target };
  json.send({ type: 'configure', jetson: '127.0.0.1', port: startupTarget.port === 65535 ? 65534 : startupTarget.port + 1 });
  const rejected = await json.waitFor(message => message.type === 'event' && message.event === 'configure_rejected');
  assert.match(rejected.reason, /simulation-target-locked/);
  assert.equal(testRig.bridge.target.address, startupTarget.address);
  assert.equal(testRig.bridge.target.port, startupTarget.port);
  assert.equal(testRig.bridge.options.simulation, true);
});

test('BXJV reassembly tolerates order and duplicates, rejects old frames, and expires incomplete frames', () => {
  const assembler = new JpegAssembler({ timeoutMs: 250 });
  const packets = jpegPackets(0xffff_fffe);
  assert.ok(packets.length > 1);
  let result = null;
  for (const packet of [...packets].reverse()) result = assembler.feed(packet, 100) ?? result;
  assert.equal(result.jpeg.equals(SIM_JPEG), true);
  for (const packet of packets) assembler.feed(packet, 110);
  assert.equal(assembler.completed, 1);
  assert.ok(assembler.duplicates > 0);

  const incomplete = jpegPackets(0xffff_ffff);
  assembler.feed(incomplete[0], 200);
  assembler.prune(451);
  assert.equal(assembler.dropped, 1);
  let wrapped = null;
  for (const packet of jpegPackets(0)) wrapped = assembler.feed(packet, 460) ?? wrapped;
  assert.ok(wrapped);

  for (const packet of jpegPackets(0xffff_ffff)) assembler.feed(packet, 470);
  assert.equal(assembler.completed, 2);
});

test('telemetry JSON part reassembly concatenates arrays without losing lidar points', () => {
  const bridge = new DashboardBridge({ log: false });
  const base = {
    v: 1, type: 'telemetry', bridge_id: 'sim-bridge', stream: 'lidar', epoch: 4,
    valid: true, jetson_rx_mono_s: 3, jetson_tx_mono_s: 3,
  };
  const points = Array.from({ length: 16 }, (_, index) => ({ angle_cdeg: index * 2200, distance_mm: 1000 + index, strength: 100 + index }));
  const first = JSON.parse(JSON.stringify({ ...base, part: 0, parts: 2, data: { type: 'lidar', sequence: 9, time_us: 1234, points: points.slice(0, 8) } }));
  const second = JSON.parse(JSON.stringify({ ...base, part: 1, parts: 2, data: { type: 'lidar', sequence: 9, time_us: 1234, points: points.slice(8) } }));
  bridge.acceptTelemetry(first, 10);
  bridge.acceptTelemetry(second, 11);
  assert.equal(bridge.telemetryLatest.get('lidar').data.points.length, 16);
  assert.deepEqual(bridge.telemetryLatest.get('lidar').data.points, points);
});

test('video has one replaceable pending frame and disconnects a slow video socket', async t => {
  const testRig = await rig({ maxVideoQueueBytes: 1 });
  t.after(() => testRig.close());
  const slow = {
    readyState: WebSocket.OPEN,
    bufferedAmount: 2,
    closed: null,
    close(code, reason) { this.closed = { code, reason }; },
  };
  testRig.bridge.clients.set(slow, { id: 'slow', kind: 'video', events: [], pendingState: null, pendingTelemetry: new Map(), pendingVideo: null });
  testRig.bridge.tick();
  assert.equal(slow.closed.code, 1013);
  assert.equal(testRig.bridge.clients.has(slow), false);

  const one = jpegPackets(20)[0];
  const secondFrame = jpegPackets(21)[0];
  // 完整帧由所有分片组装，客户端只保留最新帧。
  for (const packet of jpegPackets(20)) testRig.bridge.acceptVideoDatagram(packet);
  for (const packet of jpegPackets(21)) testRig.bridge.acceptVideoDatagram(packet);
  const client = { id: 'latest-only', kind: 'video', events: [], pendingState: null, pendingTelemetry: new Map(), pendingVideo: null };
  testRig.bridge.clients.set(slow, client);
  for (const packet of jpegPackets(22)) testRig.bridge.acceptVideoDatagram(packet);
  assert.equal(client.pendingVideo.equals(SIM_JPEG), true);
  assert.ok(one.length > 0 && secondFrame.length > 0);
});

test('HTTP static routes serve only the whitelisted public files', async t => {
  const testRig = await rig();
  t.after(() => testRig.close());
  const base = `http://127.0.0.1:${testRig.httpPort}`;

  const script = await fetch(`${base}/js/dashboard_network.js`);
  assert.equal(script.status, 200);
  assert.match(script.headers.get('content-type'), /^text\/javascript/);
  assert.match(await script.text(), /BX72DashboardNetwork/);

  const css = await fetch(`${base}/css/tokens.css`);
  assert.equal(css.status, 200);
  assert.equal(css.headers.get('content-type'), 'text/css; charset=utf-8');

  const image = await fetch(`${base}/assets/simulator.jpg`);
  assert.equal(image.status, 200);
  assert.equal(image.headers.get('content-type'), 'image/jpeg');
  const imageBytes = Buffer.from(await image.arrayBuffer());
  assert.deepEqual([imageBytes[0], imageBytes[1]], [0xff, 0xd8]);

  for (const pathname of [
    '/dashboard_network.js', '/index.html', '/not-allowed', '/js/not-a-script.txt',
    '/css/tokens.js', '/assets/..%2fpackage.json', '/js/%2e%2e/server.mjs',
    '/assets/missing.jpg', '/css/missing.css',
  ]) {
    assert.equal((await fetch(`${base}${pathname}`)).status, 404, `${pathname} must be 404`);
  }
  assert.equal((await fetch(`${base}/`, { method: 'POST' })).status, 405);
});

test('event JSONL rolls within a small bounded file set using asynchronous writes', async t => {
  const directory = await fs.mkdtemp(path.join(os.tmpdir(), 'bx72-dashboard-log-test-'));
  const bridge = new DashboardBridge({ logDirectory: directory, logOptions: { maxBytes: 500, backups: 2, maxQueue: 64 }, log: true });
  t.after(async () => { await bridge.log.flush(); await fs.rm(directory, { recursive: true, force: true }); });
  for (let index = 0; index < 20; index += 1) bridge.emitEvent('control_test', `event-${index}`, { index });
  await bridge.log.flush();
  const names = (await fs.readdir(directory)).filter(name => name.startsWith('dashboard.jsonl'));
  assert.ok(names.length <= 3);
  for (const name of names) assert.ok((await fs.stat(path.join(directory, name))).size <= 500);
});

test('mission admission reports stale vision, explicit emergency, and offline separately', async t => {
  const directory = await fs.mkdtemp(path.join(os.tmpdir(), 'bx72-mission-reasons-'));
  const bridge = new DashboardBridge({logDirectory:directory, log:true});
  t.after(async () => { await bridge.log.flush(); await fs.rm(directory, {recursive:true,force:true}); });
  const requester = {readyState:WebSocket.OPEN, bufferedAmount:0, messages:[],
    send(value) { this.messages.push(JSON.parse(value)); }};
  bridge.clients.set(requester,{kind:'json',events:[]});
  bridge.target={address:'127.0.0.1',port:6200};
  bridge.udp.address=()=>({address:'127.0.0.1',port:1});
  let sent=0;
  bridge.udp.send=()=>{ ++sent; };
  bridge.lastJetsonRxAt=performance.now();
  bridge.lastStatusAt=performance.now();
  bridge.status={emergency_stop:false,control_source:'none'};
  bridge.visionAt=performance.now();
  bridge.vision={age_ms:600,stale:false,emergency_stop:false,detections:[],selected_target:null};
  const payload={target_id:'private-target',class_id:'39',behavior:'approach',color:''};
  bridge.sendMissionCommand(requester,'start',payload);
  assert.match(requester.messages.at(-1).message,/视觉观测已过期/);
  assert.doesNotMatch(requester.messages.at(-1).message,/急停/);
  bridge.vision={...bridge.vision,age_ms:0,emergency_stop:true};
  bridge.status={...bridge.status,emergency_stop:true};
  bridge.sendMissionCommand(requester,'start',payload);
  assert.match(requester.messages.at(-1).message,/软件急停已激活/);
  bridge.lastJetsonRxAt=null;
  bridge.sendMissionCommand(requester,'start',payload);
  assert.match(requester.messages.at(-1).message,/网桥当前不在线/);
  assert.doesNotMatch(requester.messages.at(-1).message,/急停|视觉观测/);
  assert.equal(sent,0,'rejected requests never send a mission or motion');
  await bridge.log.flush();
  const records=(await fs.readFile(path.join(directory,'dashboard.jsonl'),'utf8')).trim().split('\n').map(JSON.parse);
  assert.equal(records.length,3);
  assert.ok(records.every(record=>record.type==='mission_result' && record.success===false &&
    record.unknown===false && record.request_id===null));
  assert.doesNotMatch(JSON.stringify(records),/private-target/);

  bridge.lastJetsonRxAt=bridge.lastStatusAt=performance.now();
  bridge.status={emergency_stop:false,control_source:'none'};
  bridge.vision={...bridge.vision,age_ms:600,emergency_stop:true};
  bridge.clientConsumptionReady=()=>true;
  bridge.bridgeId='jetson';
  const local=bridge.sendNavigationTask(requester,{action:'start',frame:'odom',domain:'7:3',generation:null,
    goals:[{x:1,y:0}],max_speed_mm_s:100,max_accel_mm_s2:200,
    tolerance_mm:100,yaw_tolerance_mrad:100,duration_s:30});
  assert.ok(local,'vision state does not veto a Jetson navigation request');
  assert.equal(sent,1);
});

test('mission failure and timeout are logged privately without request payload or automatic retry', async t => {
  const directory = await fs.mkdtemp(path.join(os.tmpdir(), 'bx72-mission-log-'));
  const bridge = new DashboardBridge({logDirectory:directory, log:true});
  t.after(async () => { await bridge.log.flush(); await fs.rm(directory, {recursive:true,force:true}); });
  function fakeSocket() {
    return {readyState:WebSocket.OPEN,bufferedAmount:0,messages:[],
      send(value) { this.messages.push(JSON.parse(value)); }};
  }
  const requester=fakeSocket(), observer=fakeSocket();
  bridge.clients.set(requester,{kind:'json',events:[]});
  bridge.clients.set(observer,{kind:'json',events:[]});
  let sent=0;
  bridge.udp.send=()=>{ ++sent; };
  bridge.pendingMission.set('failed-id',{action:'start',session:bridge.session,at:1,socket:requester});
  bridge.acceptMissionResult({v:1,type:'mission_result',request_id:'failed-id',action:'start',
    session:bridge.session,success:false,unknown:false,message:'已有 LOST 任务，请先结束原任务',
    payload:{target_id:'private-target'},secret:'private-token'});
  assert.equal(requester.messages.at(-1).message,'已有 LOST 任务，请先结束原任务');
  bridge.pendingMission.set('timeout-id',{action:'start',session:bridge.session,at:1,socket:requester});
  bridge.lastSendAt=3000;
  bridge.tick(3000);
  assert.equal(requester.messages.at(-1).unknown,true);
  assert.match(requester.messages.at(-1).message,/超时.*未自动重试/);
  assert.equal(observer.messages.length,0,'another page receives no private result');
  assert.equal(sent,0,'failure and timeout never issue a retry or movement command');
  await bridge.log.flush();
  const records=(await fs.readFile(path.join(directory,'dashboard.jsonl'),'utf8')).trim().split('\n').map(JSON.parse);
  assert.deepEqual(records.map(({action,request_id,success,unknown,message})=>
    ({action,request_id,success,unknown,message})),[
    {action:'start',request_id:'failed-id',success:false,unknown:false,message:'已有 LOST 任务，请先结束原任务'},
    {action:'start',request_id:'timeout-id',success:false,unknown:true,message:'命令超时，结果未知；未自动重试，请核对实际状态'},
  ]);
  assert.doesNotMatch(JSON.stringify(records),/private-target|private-token/);
  assert.ok(records.every(record=>!Object.hasOwn(record,'session') && !Object.hasOwn(record,'payload')));
});
