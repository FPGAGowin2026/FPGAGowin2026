import assert from 'node:assert/strict';
import {EventEmitter} from 'node:events';
import {spawn} from 'node:child_process';
import {createInterface} from 'node:readline';
import {fileURLToPath} from 'node:url';
import test from 'node:test';
import {GlobalNavigation} from '../scripts/global_planner/global_navigation.mjs';
import {checkGlobalSegment,decodeGlobalMap,planGlobalRoute} from '../scripts/global_planner/global_route.mjs';
import {runWorker} from '../scripts/global_planner/route_runner.mjs';

function fixture(frame='map') {
  let now = 1000, next = 0;
  const sent = [], planned = [];
  const map = {status:'building',domain:'7:3',generation:'g1',revision:1,
    resolution_m:.05,width:64,height:64,origin:{x:-1,y:-1},
    map_from_odom:{x:0,y:0,yaw:0},map_from_odom_age_ms:0,pose:{x:0,y:0,yaw:0}};
  const context = {online:true,ready:true,emergency:false,ownerReady:true,
    nav:{boot_id:7,odom_domain:3,task_id:0,last_task_id:0,status:0,active_task:false},
    localMap:{available:true,status:'building',domain:'7:3',pose_fresh:true,
      pose_odom:{x:0,y:0,yaw:0},origin:{x:-1.6,y:-1.6},
      grid_from_odom:{map_anchor_x_mm:0,map_anchor_y_mm:0,odom_anchor_x_mm:0,
        odom_anchor_y_mm:0,yaw_offset_mrad:0},cells_rle:[1,4096]}};
  let planner=async input=>({path:[input.start,input.goal],reason:null});
  let checker=async()=>({segmentClear:true});
  let fetcher=async()=>structuredClone(map);
  const controller = new GlobalNavigation({getContext:()=>context,fetchMap:async()=>fetcher(),
    plan:async input=>{planned.push(input);return planner(input);},
    checkSegment:async input=>checker(input),now:()=>now,
    sendLocal:(action,fields)=>{
      const result={requestId:`r${++next}`,taskId:next};
      sent.push({action,fields,...result}); return result;
    }});
  const fields = {frame,domain:'7:3',generation:frame === 'map' ? 'g1' : null,
    goals:[{x:.6,y:0}],max_speed_mm_s:200,max_accel_mm_s2:200,
    tolerance_mm:100,yaw_tolerance_mrad:100,duration_s:30};
  const ack = entry => controller.acceptResult({request_id:entry.requestId,action:entry.action,
    success:true,...(entry.action === 'local_start' ? {task_id:entry.taskId} :
      {transaction_task_id:entry.taskId})});
  return {controller,fields,map,context,sent,planned,ack,advance:ms=>{now+=ms;},
    setPlanner:fn=>{planner=fn;},setChecker:fn=>{checker=fn;},setFetcher:fn=>{fetcher=fn;}};
}

test('map goal is reprojected only after cancellation is confirmed', async () => {
  const f=fixture();
  const accepted=await f.controller.start('ros',f.fields);
  assert.equal(accepted.status,'planning');
  assert.equal(f.sent.length,0,'acceptance must not claim motion was dispatched');
  await f.controller.tick();
  assert.equal(f.sent.length,1);
  assert.equal(f.sent[0].action,'local_start');
  assert.equal(f.sent[0].fields.mode,'independent');
  assert.equal(f.sent[0].fields.allow_disconnect,true);
  assert.equal(f.planned[0].radius,.191);
  f.ack(f.sent[0]);
  f.context.nav={...f.context.nav,task_id:1,last_task_id:1,status:3,active_task:true};
  f.context.ready=false;
  f.map.map_from_odom.x=.08;
  f.advance(1100);
  await f.controller.tick();
  assert.equal(f.sent[1].action,'local_cancel');
  assert.equal(f.sent.length,2,'a corrected goal cannot race the prior segment');
  f.ack(f.sent[1]);
  f.context.nav={...f.context.nav,task_id:1,last_task_id:2,status:7,active_task:false};
  f.context.ready=true;
  await f.controller.tick();
  assert.equal(f.sent[2].action,'local_start');
  assert.ok(Math.abs(f.sent[2].fields.goal_x_m-.52)<1e-9);
  assert.deepEqual(f.controller.snapshot().goals,[{x:.6,y:0}]);
});

test('odom goals remain fixed and transport ACK is not waypoint arrival', async () => {
  const f=fixture('odom');
  await f.controller.start('ros',f.fields);
  await f.controller.tick();
  assert.equal(f.sent[0].fields.goal_x_m,.6);
  f.ack(f.sent[0]);
  f.context.nav={...f.context.nav,task_id:1,last_task_id:1,status:3,active_task:true};
  f.context.ready=false;
  f.map.map_from_odom.x=.2;
  f.advance(1500);
  await f.controller.tick();
  assert.equal(f.sent.length,1);
  assert.equal(f.controller.snapshot().active,true);
  assert.equal(f.controller.snapshot().goal_index,0);
  f.context.nav={...f.context.nav,task_id:1,last_task_id:1,status:4,active_task:false};
  f.context.localMap.pose_odom={x:.6,y:0,yaw:0};
  f.context.ready=true;
  await f.controller.tick();
  assert.equal(f.controller.snapshot().status,'arrived');
});

test('transport ACK without the matching device state times out and cancels', async () => {
  const f=fixture('odom');
  await f.controller.start('ros',f.fields);
  await f.controller.tick();
  f.ack(f.sent[0]);
  assert.equal(f.controller.snapshot().status,'planning');
  f.advance(2100);
  await f.controller.tick();
  assert.equal(f.sent[1].action,'local_cancel');
  assert.equal(f.controller.snapshot().active,true);
  assert.match(f.controller.snapshot().message,/停止/);
  f.ack(f.sent[1]);
  f.context.nav={...f.context.nav,last_task_id:2,active_task:false};
  await f.controller.tick();
  assert.equal(f.controller.snapshot().active,false);
  assert.equal(f.controller.snapshot().status,'failed');
  assert.match(f.controller.snapshot().message,/设备未确认/);
});

test('an observed task is not timed out when its transport ACK arrives later', async () => {
  const f=fixture('odom');
  await f.controller.start('ros',f.fields);
  await f.controller.tick();
  f.context.nav={...f.context.nav,task_id:1,last_task_id:1,status:3,active_task:true};
  f.context.ready=false;
  await f.controller.tick();
  assert.equal(f.controller.snapshot().status,'following');
  f.ack(f.sent[0]);
  f.advance(2100);
  await f.controller.tick();
  assert.equal(f.sent.length,1);
  assert.equal(f.controller.snapshot().active,true);
});

test('arrival before transport ACK waits for the transaction before advancing', async () => {
  const f=fixture('odom');
  f.fields.goals.push({x:1.2,y:0});
  await f.controller.start('ros',f.fields); await f.controller.tick();
  f.context.nav={...f.context.nav,task_id:1,last_task_id:1,status:4,active_task:false};
  f.context.localMap.pose_odom={x:.6,y:0,yaw:0};
  await f.controller.tick();
  assert.equal(f.sent.length,1);
  assert.equal(f.controller.snapshot().goal_index,0);
  f.ack(f.sent[0]);
  await f.controller.tick();
  assert.equal(f.sent[1].fields.goal_x_m,1.2);
  assert.equal(f.controller.snapshot().goal_index,1);
});

test('odom device terminal state is checked against actual pose before advancing', async () => {
  const f=fixture('odom');
  await f.controller.start('ros',f.fields);
  await f.controller.tick();
  f.ack(f.sent[0]);
  f.context.nav={...f.context.nav,task_id:1,last_task_id:1,status:4,active_task:false};
  f.context.localMap.pose_odom={x:.3,y:0,yaw:0};
  await f.controller.tick();
  assert.equal(f.controller.snapshot().goal_index,0);
  assert.equal(f.controller.snapshot().active,true);
  assert.equal(f.sent[1].action,'local_start');
  assert.equal(f.sent[1].fields.goal_x_m,.6);
});

test('map route executes successive segments and checks final position', async () => {
  const f=fixture();
  f.fields.goals=[{x:1.4,y:0}];
  await f.controller.start('ros',f.fields);
  await f.controller.tick();
  assert.ok(Math.abs(f.sent[0].fields.goal_x_m-.75)<1e-9);
  f.ack(f.sent[0]);
  f.context.nav={...f.context.nav,task_id:1,last_task_id:1,status:4,active_task:false};
  f.context.localMap.pose_odom={x:.75,y:0,yaw:0};
  await f.controller.tick();
  assert.equal(f.sent[1].fields.goal_x_m,1.4);
  assert.equal(f.controller.snapshot().goal_index,0);
  f.ack(f.sent[1]);
  f.context.nav={...f.context.nav,task_id:2,last_task_id:2,status:4,active_task:false};
  f.context.localMap.pose_odom={x:1.2,y:0,yaw:0};
  await f.controller.tick();
  assert.equal(f.controller.snapshot().active,true);
  assert.equal(f.sent[2].fields.goal_x_m,1.4);
  f.ack(f.sent[2]);
  f.context.nav={...f.context.nav,task_id:3,last_task_id:3,status:4,active_task:false};
  f.context.localMap.pose_odom={x:1.4,y:0,yaw:0};
  await f.controller.tick();
  assert.equal(f.controller.snapshot().status,'arrived');
});

test('new obstruction cancels and requires ACK plus matching inactive state', async () => {
  for (const telemetryFirst of [false,true]) {
    const f=fixture(); let clear=true;
    f.setChecker(async()=>({segmentClear:clear}));
    await f.controller.start('ros',f.fields);
    await f.controller.tick();
    f.ack(f.sent[0]);
    f.context.nav={...f.context.nav,task_id:1,last_task_id:1,status:3,active_task:true};
    f.context.ready=false;
    clear=false; f.advance(1100); await f.controller.tick();
    assert.equal(f.sent[1].action,'local_cancel');
    clear=true;
    const stopped=()=>{f.context.nav={...f.context.nav,task_id:1,last_task_id:2,status:7,active_task:false};
      f.context.ready=true;};
    if (telemetryFirst) stopped(); else f.ack(f.sent[1]);
    await f.controller.tick();
    assert.equal(f.sent.length,2);
    if (telemetryFirst) f.ack(f.sent[1]); else stopped();
    await f.controller.tick();
    assert.equal(f.sent[2].action,'local_start');
    assert.deepEqual(f.controller.snapshot().goals,f.fields.goals);
  }
});

test('stale transform cancels active segment and map generation changes end the task', async () => {
  const f=fixture();
  await f.controller.start('ros',f.fields); await f.controller.tick();
  f.ack(f.sent[0]);
  f.context.nav={...f.context.nav,task_id:1,last_task_id:1,status:3,active_task:true};
  f.context.ready=false;
  f.map.map_from_odom_age_ms=751;
  f.advance(1100); await f.controller.tick();
  assert.equal(f.sent[1].action,'local_cancel');
  f.ack(f.sent[1]);
  f.context.nav={...f.context.nav,task_id:1,last_task_id:2,status:7,active_task:false};
  f.context.ready=true; f.map.map_from_odom_age_ms=0;
  await f.controller.tick();
  assert.equal(f.sent[2].action,'local_start');
  f.ack(f.sent[2]);
  f.context.nav={...f.context.nav,task_id:3,last_task_id:3,status:3,active_task:true};
  f.context.ready=false; f.map.generation='g2';
  f.advance(1100); await f.controller.tick();
  assert.equal(f.sent[3].action,'local_cancel');
  f.ack(f.sent[3]);
  f.context.nav={...f.context.nav,task_id:3,last_task_id:4,status:7,active_task:false};
  await f.controller.tick();
  assert.equal(f.controller.snapshot().status,'cancelled');
  assert.equal(f.controller.snapshot().active,false);
});

test('planning completion after a user cancellation cannot dispatch a stale segment', async () => {
  const f=fixture(); let release;
  f.setPlanner(input=>new Promise(resolve=>{release=()=>resolve({path:[input.start,input.goal]});}));
  await f.controller.start('ros',f.fields);
  const pending=f.controller.tick();
  while (!release) await new Promise(resolve=>setImmediate(resolve));
  f.controller.stop('用户停止');
  release(); await pending;
  assert.equal(f.sent.length,0);
  assert.equal(f.controller.snapshot().status,'cancelled');
});

test('a new obstacle during the async sweep is rechecked before dispatch', async () => {
  const f=fixture(); let checks=0;
  f.setChecker(async()=>{
    if (++checks===1) {
      f.context.localMap={...f.context.localMap,cells_rle:[1,10,2,1,1,4085]};
      return {segmentClear:true};
    }
    return {segmentClear:false};
  });
  await f.controller.start('ros',f.fields); await f.controller.tick();
  assert.equal(checks,2);
  assert.equal(f.sent.length,0);
  assert.equal(f.controller.snapshot().status,'blocked');
});

test('an oversized route is reported without dispatching or displaying a truncated path', async () => {
  const f=fixture();
  f.setPlanner(async input=>({path:Array.from({length:257},(_,i)=>
    ({x:input.start.x+i*.001,y:input.start.y})),reason:null}));
  await f.controller.start('ros',f.fields); await f.controller.tick();
  assert.equal(f.sent.length,0);
  assert.equal(f.controller.snapshot().status,'blocked');
  assert.match(f.controller.snapshot().message,/过于复杂/);
  assert.deepEqual(f.controller.snapshot().route,[]);
});

test('real planner cancels an active segment when live local occupancy blocks the vehicle footprint', async () => {
  const f=fixture();
  Object.assign(f.map,{v:1,type:'global_mapping',protocol:'slam_toolbox_view_v1',source:'slam_toolbox',
    available:true,width:120,height:120,resolution_m:.05,origin:{x:-3,y:-3},cells_rle:[1,14400]});
  Object.assign(f.context.localMap,{width:64,height:64,resolution_m:.05});
  f.setFetcher(async()=>decodeGlobalMap(structuredClone(f.map)));
  f.setPlanner(async args=>planGlobalRoute(args));
  f.setChecker(async args=>checkGlobalSegment(args));
  await f.controller.start('ros',f.fields); await f.controller.tick();
  assert.equal(f.sent[0].action,'local_start');
  f.ack(f.sent[0]);
  f.context.nav={...f.context.nav,task_id:1,last_task_id:1,status:3,active_task:true};
  f.context.ready=false;
  const obstacle=35*64+40;
  f.context.localMap.cells_rle=[1,obstacle,2,1,1,4096-obstacle-1];
  f.advance(1100); await f.controller.tick();
  assert.equal(f.sent[1].action,'local_cancel');
  assert.equal(f.controller.snapshot().active,true);
});

test('a 350 mm passage falls back to a 135 mm proposal and retains its clearance during execution', async () => {
  const f=fixture(), width=80, height=40;
  const cells=new Uint8Array(width*height).fill(2);
  for (let y=17;y<=23;y++) cells.fill(1,y*width,(y+1)*width);
  Object.assign(f.map,{width,height,cells,origin:{x:-1,y:-1}});
  Object.assign(f.context.localMap,{width:64,height:64,resolution_m:.05,cells_rle:[0,4096]});
  f.context.localMap.pose_odom.y=.025;
  f.fields.goals=[{x:1.4,y:.025}];
  f.setPlanner(async args=>runWorker(args,'plan'));
  const checkedRadii=[];
  f.setChecker(async args=>{checkedRadii.push(args.radius);return runWorker(args,'segment');});
  await f.controller.start('ros',f.fields); await f.controller.tick();
  assert.deepEqual(f.planned.map(p=>p.radius),[.191,.135]);
  assert.equal(f.sent[0].action,'local_start');
  assert.ok(f.sent[0].fields.goal_x_m>0);
  assert.ok(f.controller.snapshot().route.length>=2);
  assert.deepEqual(checkedRadii,[.135]);
  f.ack(f.sent[0]);
  f.context.nav={...f.context.nav,task_id:1,last_task_id:1,status:3,active_task:true};
  f.context.ready=false;
  f.advance(1100); await f.controller.tick();
  assert.equal(f.sent.length,1,'the valid narrow segment is not cancelled by rechecking at 191 mm');
  assert.equal(checkedRadii.at(-1),.135);
});

test('invalid planning inputs do not trigger a narrower search', async () => {
  const f=fixture();
  f.setPlanner(async()=>({path:[],reason:'invalid_local_map'}));
  await f.controller.start('ros',f.fields); await f.controller.tick();
  assert.deepEqual(f.planned.map(p=>p.radius),[.191]);
  assert.equal(f.sent.length,0);
});

test('cancellation during the narrow-pass search cannot dispatch its result', async () => {
  const f=fixture(); let release;
  f.setPlanner(input=>input.radius===.191 ? Promise.resolve({path:[],reason:'no_path'}) :
    new Promise(resolve=>{release=()=>resolve({path:[input.start,input.goal],reason:null});}));
  await f.controller.start('ros',f.fields);
  const pending=f.controller.tick();
  while (!release) await new Promise(resolve=>setImmediate(resolve));
  f.controller.stop('用户停止');
  release(); await pending;
  assert.equal(f.sent.length,0);
  assert.equal(f.controller.snapshot().status,'cancelled');
});

test('sustained blocking chooses a detour only after stopping the previous segment', async () => {
  const f=fixture(); let blockedPlans=0;
  f.setPlanner(async input=>{
    const around=(input.currentSegment && f.context.nav.status===5 && ++blockedPlans>=3) ||
      (!input.currentSegment && f.controller.detourPending);
    return {path:around ? [input.start,{x:input.start.x+.4,y:input.start.y+.25},input.goal] :
      [input.start,input.goal],reason:null};
  });
  await f.controller.start('ros',f.fields); await f.controller.tick();
  f.ack(f.sent[0]);
  f.context.nav={...f.context.nav,task_id:1,last_task_id:1,status:5,active_task:true};
  f.context.ready=false;
  f.advance(1); await f.controller.tick();
  f.advance(2100); await f.controller.tick();
  assert.equal(f.sent[1].action,'local_cancel');
  f.ack(f.sent[1]);
  f.context.nav={...f.context.nav,task_id:1,last_task_id:2,status:7,active_task:false};
  f.context.ready=true;
  await f.controller.tick();
  assert.ok(f.sent[2].fields.goal_y_m>.2);
  assert.deepEqual(f.controller.snapshot().goals,f.fields.goals);
});

test('worker decodes map data and retains domain information on invalid snapshots', async () => {
  const snapshot={v:1,type:'global_mapping',protocol:'slam_toolbox_view_v1',source:'slam_toolbox',
    available:true,status:'building',domain:'7:3',generation:'g1',width:2,height:2,resolution_m:.05,
    origin:{x:0,y:0},map_from_odom:{x:0,y:0,yaw:0},pose:{x:0,y:0,yaw:0},cells_rle:[1,4]};
  const bytes=Uint8Array.from(Buffer.from(JSON.stringify(snapshot)));
  const decoded=await runWorker(bytes.buffer,'decode');
  assert.deepEqual(Array.from(decoded.cells),[1,1,1,1]);
  snapshot.width=0;
  const invalid=Uint8Array.from(Buffer.from(JSON.stringify(snapshot)));
  await assert.rejects(runWorker(invalid.buffer,'decode'),error=>
    error.domain==='7:3' && error.generation==='g1');
});

test('a hanging or prematurely exited worker rejects instead of blocking the scheduler', async () => {
  class HangingWorker extends EventEmitter { terminate(){this.terminated=true;} }
  await assert.rejects(runWorker({},'plan',{timeoutMs:5,WorkerClass:HangingWorker}),/搜索超时/);
  class EmptyWorker extends EventEmitter {
    constructor(){super();queueMicrotask(()=>this.emit('exit',0));}
    terminate(){}
  }
  await assert.rejects(runWorker({},'plan',{WorkerClass:EmptyWorker}),/未返回结果/);
});

test('real coordinator waits for a valid task watermark and accepts zero as a known watermark', async t => {
  const script = fileURLToPath(new URL('../scripts/global_planner/jetson_global_planner.mjs', import.meta.url));
  const child = spawn(process.execPath,[script],{stdio:['pipe','pipe','pipe']});
  t.after(() => child.kill());
  const lines = createInterface({input:child.stdout});
  const waiting = new Map();
  lines.on('line', line => {
    let message;
    try { message=JSON.parse(line); } catch { return; }
    if (message.kind !== 'command_result' || !waiting.has(message.id)) return;
    waiting.get(message.id)(message);
    waiting.delete(message.id);
  });
  const write = message => child.stdin.write(`${JSON.stringify(message)}\n`);
  const command = (id,request) => new Promise((resolve,reject) => {
    const timer = setTimeout(() => {waiting.delete(id); reject(new Error('coordinator response timeout'));},3000);
    waiting.set(id,value=>{clearTimeout(timer);resolve(value);});
    write({kind:'command',id,request});
  });
  const idleCancel = await command('initial-cancel', {action:'cancel'});
  assert.equal(idleCancel.success, true);
  assert.equal(idleCancel.state?.active, false);
  assert.equal(idleCancel.state?.status, 'idle');
  const nav = {protocol:'nav_grid_v1',boot_id:7,odom_domain:1,now_us:1000,
    active_task:false,pose_valid:true,status:0};
  const request = {action:'start',frame:'odom',domain:'7:1',generation:null,
    goals:[{x:1,y:0}],max_speed_mm_s:200,max_accel_mm_s2:200,
    tolerance_mm:100,yaw_tolerance_mrad:100,duration_s:30};
  write({kind:'executor_ready',ready:true});
  write({kind:'navigation',data:nav});
  assert.equal((await command('missing',request)).success,false);
  write({kind:'navigation',data:{...nav,now_us:1001,last_task_id:0}});
  assert.equal((await command('zero',request)).success,true);
});
