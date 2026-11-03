import assert from 'node:assert/strict';
import test from 'node:test';
import {DashboardBridge} from '../server.mjs';

function fixture() {
  const bridge = new DashboardBridge({log:false});
  const sent = [], results = [], socket = {};
  bridge.target = {address:'127.0.0.1',port:6200};
  bridge.bridgeId = 'jetson-1';
  bridge.udp.address = () => ({address:'127.0.0.1',port:50000});
  bridge.udp.send = data => sent.push(JSON.parse(data));
  bridge.deliverMissionResult = (result, owner) => results.push({result,owner});
  bridge.queueLatest = () => {};
  return {bridge,sent,results,socket};
}

test('航点原样提交 Jetson，只有收到匹配请求的真实结果才通知页面', () => {
  const {bridge,sent,results,socket} = fixture();
  const goals = [{x:1,y:2},{x:3,y:-4}];
  const id = bridge.sendNavigationTask(socket,{action:'start',frame:'map',domain:'7:3',
    generation:'map-1',goals,max_speed_mm_s:200});
  assert.equal(sent.length,1);
  assert.equal(sent[0].type,'navigation_task');
  assert.equal(sent[0].session,bridge.session);
  assert.equal(sent[0].request_id,id);
  assert.deepEqual(sent[0].goals,goals);
  assert.equal(sent[0].frame,'map');
  assert.equal(results.length,0);
  const reply = {request_id:id,session:bridge.session,bridge_id:bridge.bridgeId,
    action:'start',success:true,message:'已接收'};
  bridge.acceptNavigationTaskResult({...reply,bridge_id:'wrong'});
  bridge.acceptNavigationTaskResult({...reply,session:'old'});
  bridge.acceptNavigationTaskResult({...reply,action:'stop'});
  assert.equal(results.length,0);
  bridge.acceptNavigationTaskResult(reply);
  assert.deepEqual(results[0],{owner:socket,result:{v:1,type:'mission_result',request_id:id,
    action:'navigation_start',success:true,unknown:false,message:'已接收'}});
  bridge.acceptNavigationTaskResult(reply);
  assert.equal(results.length,1);
});

test('断开页面不取消 Jetson 任务；停止仍作为高层请求转发', () => {
  const {bridge,sent,socket} = fixture();
  bridge.clients.set(socket,{kind:'json'});
  bridge.sendNavigationTask(socket,{action:'start',frame:'odom',domain:'7:3',goals:[{x:1,y:0}]});
  bridge.onClientClose(socket);
  assert.equal(sent.length,1);
  bridge.sendNavigationTask(null,{action:'stop'});
  assert.equal(sent[1].action,'stop');
  assert.equal(sent[1].type,'navigation_task');
});

test('首个真实结果确认网桥身份，来源错误或超大航点不会被当作成功', () => {
  const {bridge,sent,results,socket} = fixture();
  bridge.bridgeId = null;
  const id=bridge.sendNavigationTask(socket,{action:'start',frame:'odom',domain:'7:3',generation:null,goals:[{x:1,y:2}]});
  assert.equal(sent.length,1);
  assert.equal(results.length,0);
  const reply={v:1,type:'navigation_task_result',request_id:id,session:bridge.session,
    bridge_id:'jetson-1',action:'start',success:true,message:'目标已接收'};
  bridge.onUdpMessage(Buffer.from(JSON.stringify(reply)),{address:'127.0.0.2',port:6200});
  assert.equal(bridge.bridgeId,null,'wrong UDP source cannot establish identity');
  bridge.retiredBridgeIds.add('old-bridge');
  bridge.onUdpMessage(Buffer.from(JSON.stringify({...reply,bridge_id:'old-bridge'})),{address:'127.0.0.1',port:6200});
  assert.equal(bridge.bridgeId,null,'retired bridge identity cannot be restored');
  bridge.onUdpMessage(Buffer.from(JSON.stringify({...reply,session:'old'})),{address:'127.0.0.1',port:6200});
  assert.equal(bridge.bridgeId,null,'response must match current session');
  bridge.onUdpMessage(Buffer.from(JSON.stringify(reply)),{address:'127.0.0.1',port:6200});
  assert.equal(bridge.bridgeId,'jetson-1');
  assert.equal(results.at(-1).result.success,true);
  bridge.sendNavigationTask(socket,{action:'start',goals:Array.from({length:2000},(_,x)=>({x,y:0}))});
  assert.equal(results.at(-1).result.success,false);
  assert.match(results.at(-1).result.message,/容量/);
  assert.equal(sent.length,1,'oversized request is not truncated and sent');
});

test('切换导航只释放请求页的人工控制，零速先于目标发送', () => {
  const {bridge,sent,socket}=fixture();
  bridge.owner={id:'owner',socket};
  bridge.velocity=[100,0,0];
  bridge.sendNavigationTask(socket,{action:'start',frame:'odom',domain:'7:3',generation:null,goals:[{x:1,y:0}]});
  assert.equal(bridge.owner,null);
  assert.deepEqual(sent.map(item=>item.type),['control','navigation_task']);
  assert.equal(sent[0].vx,0);
  assert.ok(sent[1].seq>sent[0].seq);
  const other={};bridge.owner={id:'other',socket:other};
  bridge.sendNavigationTask(socket,{action:'start',frame:'odom',domain:'7:3',generation:null,goals:[{x:1,y:0}]});
  assert.equal(bridge.owner.socket,other,'another page retains its control until Jetson arbitrates');
  assert.equal(sent.at(-1).type,'navigation_task');
});

test('乱序分片按原顺序恢复 Jetson 路线，混合状态和旧会话不刷新显示', () => {
  const {bridge} = fixture();
  const base = {v:1,type:'telemetry',bridge_id:'jetson-1',stream:'navigation_task',epoch:1,
    valid:true,jetson_rx_mono_s:1,jetson_tx_mono_s:1,parts:2};
  const data = {type:'navigation_task',sequence:5,time_us:100,session:bridge.session};
  const piece = (part,route,extras={}) => ({...base,part,
    data:{...data,state:{active:true,id:'task-1',route,...extras}}});
  bridge.acceptTelemetry(piece(1,[{x:2,y:0}]),100);
  assert.equal(bridge.navigationTask,null);
  bridge.acceptTelemetry(piece(0,[{x:1,y:0}]),101);
  assert.deepEqual(bridge.navigationTask.route,[{x:1,y:0},{x:2,y:0}]);
  assert.equal(bridge.navigationTaskAt,101);
  const badData = {...data,sequence:6};
  bridge.acceptTelemetry({...piece(0,[{x:3,y:0}]),data:{...badData,state:{active:true,id:'one',route:[{x:3,y:0}]}}},102);
  bridge.acceptTelemetry({...piece(1,[{x:4,y:0}]),data:{...badData,state:{active:true,id:'two',route:[{x:4,y:0}]}}},103);
  assert.equal(bridge.navigationTaskAt,101);
  bridge.acceptTelemetry({...base,part:undefined,parts:undefined,
    data:{...data,sequence:7,session:'old',state:{active:false}}},104);
  assert.equal(bridge.navigationTaskAt,101);
  bridge.navigationTaskAt = performance.now() - 2000;
  assert.equal(bridge.state().navigation_task.stale,true);
  assert.deepEqual(bridge.state().navigation_task.route,[]);
});

test('可等价的旧 odom 点转发至 Jetson，回执保留旧客户端动作名',()=>{
  const {bridge,sent,results,socket}=fixture();
  bridge.telemetryLatest.set('navigation',{data:{boot_id:77,odom_domain:4},pc_rx_mono_ms:0});
  bridge.clients.set(socket,{kind:'json'});
  const payload={action:'start',mode:'independent',position_only:true,allow_disconnect:true,
    goal_x_m:1.25,goal_y_m:-.5,max_speed_mm_s:80,max_accel_mm_s2:200,
    tolerance_mm:64,yaw_tolerance_mrad:100,duration_s:30};
  bridge.onClientMessage(socket,Buffer.from(JSON.stringify({v:1,type:'local_task',...payload})),false);
  assert.equal(sent.length,1);
  assert.equal(sent[0].type,'navigation_task');
  assert.equal(sent[0].frame,'odom');
  assert.equal(sent[0].domain,'77:4');
  assert.deepEqual(sent[0].goals,[{x:1.25,y:-.5}]);
  assert.equal('task_id' in sent[0],false,'网页不得分配局部事务号');
  assert.equal(results.length,0);
  bridge.acceptNavigationTaskResult({request_id:sent[0].request_id,session:bridge.session,
    bridge_id:bridge.bridgeId,action:'start',success:true,message:'仅接收目标'});
  assert.equal(results[0].result.action,'local_start');
  assert.equal(results[0].result.success,true);
  assert.equal('task_id' in results[0].result,false,'目标接收不是设备执行或到达');
});

test('语义坐标使用已缓存的坐标域交给 Jetson，不要求页面位姿新鲜',()=>{
  const {bridge,sent,socket}=fixture();
  const replies=[];
  bridge.clients.set(socket,{kind:'json'});
  bridge.safeSend=(_socket,message)=>replies.push(message);
  const send=()=>bridge.onClientMessage(socket,Buffer.from(JSON.stringify({v:1,type:'semantic_command',text:'前往坐标40000，2'})),false);
  send();
  assert.equal(replies.at(-1).accepted,false);
  assert.match(replies.at(-1).error,/坐标域/);
  bridge.telemetryLatest.set('navigation',{data:{boot_id:77,odom_domain:4},pc_rx_mono_ms:0});
  bridge.mapping.snapshot=()=>({status:'paused',pose_fresh:false});
  send();
  assert.equal(replies.at(-1).accepted,true,'this only reports forwarding');
  assert.equal(sent.at(-1).domain,'77:4');
  assert.deepEqual(sent.at(-1).goals,[{x:40000,y:2}],'range validation belongs to Jetson');
});

test('旧协同或带航向目标明确拒绝，不能绕开 Jetson 调度',()=>{
  const {bridge,sent,results,socket}=fixture();
  bridge.clients.set(socket,{kind:'json'});
  for (const payload of [
    {mode:'cooperative',position_only:true},
    {mode:'independent',position_only:true,hold_yaw_rad:0},
    {mode:'independent',position_only:false},
    {mode:'independent',position_only:true,allow_disconnect:false},
  ]) {
    bridge.onClientMessage(socket,Buffer.from(JSON.stringify({v:1,type:'mission_command',
      action:'local_start',goal_x_m:1,goal_y_m:2,...payload})),false);
    assert.equal(results.at(-1).result.action,'local_start');
    assert.equal(results.at(-1).result.success,false);
    assert.match(results.at(-1).result.message,/navigation_task/);
  }
  assert.equal(sent.length,0);
});

test('旧停止优先交给 Jetson；状态未知时仍发送通用停车',()=>{
  const {bridge,sent,results,socket}=fixture();
  bridge.navigationTask={active:true};
  bridge.navigationTaskAt=performance.now();
  bridge.sendLegacyLocalTask(socket,'local_cancel');
  assert.deepEqual(sent.map(item=>[item.type,item.action]),[['navigation_task','cancel']]);
  const request=sent[0];
  bridge.acceptNavigationTaskResult({request_id:request.request_id,session:bridge.session,
    bridge_id:bridge.bridgeId,action:'cancel',success:true,message:'已请求取消'});
  assert.equal(results.at(-1).result.action,'local_cancel');
  assert.equal(results.at(-1).result.success,true);
  bridge.navigationTask=null;
  bridge.sendLegacyLocalTask(socket,'local_stop');
  assert.deepEqual(sent.slice(1).map(item=>[item.type,item.action]),
    [['navigation_task','stop'],['mission_command','stop']]);
  const fallback=sent[2];
  bridge.acceptMissionResult({session:bridge.session,request_id:fallback.request_id,
    action:'stop',success:false,unknown:true,message:'停车结果未知'});
  assert.equal(results.at(-1).result.action,'local_stop');
  assert.equal(results.at(-1).result.unknown,true);
});

test('Jetson 停止回执失败时尝试通用停车，仍向请求者报告未知',()=>{
  const {bridge,sent,results,socket}=fixture();
  bridge.navigationTask={active:true};
  bridge.navigationTaskAt=performance.now();
  bridge.sendLegacyLocalTask(socket,'local_cancel');
  const cancel=sent[0];
  bridge.acceptNavigationTaskResult({request_id:cancel.request_id,session:bridge.session,
    bridge_id:bridge.bridgeId,action:'cancel',success:false,unknown:true,message:'结果未知'});
  assert.deepEqual(sent.map(item=>[item.type,item.action]),
    [['navigation_task','cancel'],['mission_command','stop']]);
  assert.equal(results.at(-1).result.action,'local_cancel');
  assert.equal(results.at(-1).result.unknown,true);
});

test('过期全局状态不能阻挡通用停车',()=>{
  const {bridge,sent,socket}=fixture();
  bridge.navigationTask={active:true};
  bridge.navigationTaskAt=performance.now()-2000;
  bridge.sendLegacyLocalTask(socket,'local_stop');
  assert.deepEqual(sent.map(item=>[item.type,item.action]),
    [['navigation_task','stop'],['mission_command','stop']]);
});

test('目标接收与首帧全局状态之间的停车仍通知 Jetson 协调器',()=>{
  const {bridge,sent,socket}=fixture();
  bridge.clients.set(socket,{kind:'json'});
  bridge.disarm=()=>{};
  const start=bridge.sendNavigationTask(socket,{action:'start',frame:'odom',domain:'7:3',goals:[{x:1,y:0}]});
  bridge.acceptNavigationTaskResult({request_id:start,session:bridge.session,bridge_id:bridge.bridgeId,
    action:'start',success:true,message:'目标已接收'});
  assert.equal(bridge.navigationTask,null);
  bridge.onClientMessage(socket,Buffer.from(JSON.stringify({v:1,type:'stop'})),false);
  assert.deepEqual(sent.slice(1).map(item=>[item.type,item.action]),
    [['navigation_task','stop'],['mission_command','stop']]);
});
