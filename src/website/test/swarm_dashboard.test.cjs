const assert = require('node:assert/strict');
const test = require('node:test');
const {dashboard,robot,stateFor,html,flush} = require('./swarm_fixture.cjs');

test('current members render their maps and backend readiness without submitting commands',async()=>{
  const state=stateFor(['car','acg720'],{readiness:['等待 AprilTag 自动对齐','小车协同目标未连接']});
  const d=await dashboard(state), e=d.elements;
  assert.equal(d.commands.length,0,'loading the page sends no commands');
  assert.equal(e.swarmCommonMap,undefined);
  assert.equal(e.commonMapDetails,undefined);
  assert.ok(d.requests.every(url=>!url.includes('view=common')));
  assert.equal(e.hexRemote,undefined);
  assert.equal(e.swarmMemberHexapod,undefined);
  assert.doesNotMatch(html,/Tang Primer|Orange Pi|data-hex-action/);
  assert.match(e.swarmReadiness.textContent,/^未就绪：\n• 等待 AprilTag 自动对齐\n• 小车协同目标未连接$/);
  assert.equal(e.swarmExploreStart.disabled,true);
  assert.equal(e.carSwarmLocalMap.hidden,false);
  assert.equal(e.acg720LocalMap.hidden,false);
  assert.deepEqual(Array.from(d.images[0].data.slice(0,16)),
    [211,231,238,255,143,162,166,255,252,255,251,255,33,68,81,255],
    'ROS rows and unknown, partial, free and occupied colors remain consistent');
  assert.ok(d.requests.every(url=>!url.includes('/api/hex/')));
});

test('reobservation remains explicit, reports upstream errors and blocks repeated or non-idle requests',async()=>{
  const state=stateFor(['car','acg720']);
  const d=await dashboard(state), e=d.elements;
  assert.equal(e.swarmReobserve.disabled,false);
  await e.swarmReobserve.onclick();
  assert.deepEqual(d.commands.at(-1),{url:'/swarm/api/alignment/reobserve',body:{}});
  assert.match(e.swarmReobserveResult.textContent,/重新识别请求已接收/);
  assert.match(e.swarmMemberAcg720Alignment.textContent,/等待相机内参与安装变换/,
    'acceptance does not invent an alignment result');
  d.network.postStatus=409;d.network.postBody={error:'请先停止协同探索'};
  await e.swarmReobserve.onclick();
  assert.match(e.swarmReobserveResult.textContent,/重新识别请求失败：请先停止协同探索/);
  d.network.postStatus=200;d.network.postBody={accepted:true};d.network.holdPost=true;
  const before=d.commands.length, pending=e.swarmReobserve.onclick();
  await flush();assert.equal(e.swarmReobserve.disabled,true);
  await e.swarmReobserve.onclick();
  assert.equal(d.commands.length,before+1,'a pending request prevents duplicate submission');
  d.network.resolvePost();await pending;d.network.holdPost=false;
  for(const exploring of [true,undefined]){
    await d.update({...state,exploring});
    assert.equal(e.swarmReobserve.disabled,true);
    const count=d.commands.length;await e.swarmReobserve.onclick();assert.equal(d.commands.length,count);
  }
  await d.disconnect();assert.equal(e.swarmReobserve.disabled,true);
});

test('exploration and car cancellation confirmation retain their existing explicit interactions',async()=>{
  const state=stateFor(['car'],{readiness:[],mapping_readiness:[],car:{mode:'idle'},
    robots:{car:robot('car',{aligned:true,auto_navigation_available:true})}});
  const d=await dashboard(state), e=d.elements;
  assert.equal(e.swarmExploreStart.disabled,false);
  await e.swarmExploreStart.onclick();
  assert.deepEqual(d.commands.at(-1),{url:'/swarm/api/explore/start',body:{}});
  e.swarmAutoDetails.open=false;
  await d.update({...state,exploring:true});
  assert.equal(e.swarmAutoDetails.open,true);
  assert.equal(e.swarmExploreStop.disabled,false);
  await e.swarmExploreStop.onclick();
  assert.deepEqual(d.commands.at(-1),{url:'/swarm/api/explore/stop',body:{}});
  const cancelling={...state,readiness:['等待上一条小车目标的取消结果']};
  await d.update(cancelling);
  assert.equal(e.swarmStopConfirm.hidden,false);
  assert.equal(e.swarmConfirmStopped.disabled,true,'on-site confirmation remains explicit');
  const before=d.commands.length;await e.swarmConfirmStopped.onclick();assert.equal(d.commands.length,before);
  e.swarmStopConfirmed.checked=true;e.swarmStopConfirmed.onchange();
  await d.update({...cancelling,car:{mode:'navigating'}});
  assert.equal(e.swarmConfirmStopped.disabled,true,'a moving task cannot be confirmed stopped');
  await d.update(cancelling);assert.equal(e.swarmConfirmStopped.disabled,false);
  await e.swarmConfirmStopped.onclick();
  assert.deepEqual(d.commands.at(-1),{url:'/swarm/api/explore/confirm-stopped',body:{confirmed:true}});
  assert.equal(e.swarmStopConfirmed.checked,false);
  await d.update(state);assert.equal(e.swarmStopConfirm.hidden,true);
});

test('transient failures preserve recent state but disconnection or a stalled poll hides stale maps',async()=>{
  const state=stateFor(['car','acg720'],{readiness:[]});
  const d=await dashboard(state), e=d.elements;
  d.network.available=false;await d.poll(300);
  assert.equal(e.swarmReobserve.disabled,false,'a brief status failure retains recent state');
  await d.poll(2600);
  assert.equal(e.swarmConnection.textContent,'协同服务不可用');
  assert.equal(e.carSwarmLocalMap.hidden,true);
  assert.equal(e.acg720LocalMap.hidden,true);
  assert.equal(e.swarmExploreStart.disabled,true);
  assert.equal(d.commands.length,0,'losing state does not submit a motion or stop request');
  d.network.available=true;await d.poll(2100);
  assert.equal(e.carSwarmLocalMap.hidden,false);
  d.network.suspend=true;d.poll(1000);await flush();
  await d.poll(2600);
  assert.equal(e.swarmConnection.textContent,'协同服务不可用');
  assert.equal(e.swarmReobserve.disabled,true);
  assert.equal(d.commands.length,0);
});

const peerState=()=>stateFor(['car','acg720'],{
  anchor:'acg720',frame:'acg720_map',anchor_locked:true,mapping_status:'aligned',
  alignments:{car:{transform:[3,-2,Math.PI/2]}},
  robots:{car:robot('car',{aligned:true}),
    acg720:robot('acg720',{aligned:true,pose_common:{x:1,y:1,yaw:-3*Math.PI/4}})}});
const peerDetail=app=>app.events.filter(event=>event.type==='bx72:swarm-pose').at(-1).detail;
const close=(actual,expected)=>assert.ok(Math.abs(actual-expected)<1e-9,`${actual} != ${expected}`);

test('peer pose uses the full SE2 inverse and heading calibration while reusing unchanged maps',async()=>{
  const state=peerState();
  state.robots.car.map_version='session:1';
  state.robots.acg720.map_version='session:2';
  const d=await dashboard(state);
  const detail=peerDetail(d);
  assert.equal(detail.frame,'map');assert.equal(detail.name,'六足');
  close(detail.x,3);close(detail.y,2);close(detail.yaw,3*Math.PI/4);
  assert.ok(d.events.every(event=>event.type==='bx72:swarm-pose'));
  assert.equal(d.events[0].detail,null,'the initial disconnected render clears any peer');
  assert.equal(d.requests.filter(url=>url.endsWith('/api/state')).length,1);
  const requests=d.requests.length, marker=d.paint('acg720LocalMap').markers[0];
  assert.equal(d.images.length,2);
  const peer={...state.robots.acg720,heading_offset_rad:Math.PI/2,
    pose_common:{x:2,y:2,yaw:0},pose_local:[1,.5,0]};
  await d.update({...state,robots:{...state.robots,acg720:peer}});
  close(peerDetail(d).x,4);close(peerDetail(d).y,1);close(peerDetail(d).yaw,0);
  assert.match(d.elements.swarmMemberAcg720Pose.textContent,/朝向按机头.*1\.57 rad/);
  assert.notEqual(d.paint('acg720LocalMap').markers[0].x,marker.x);
  assert.equal(d.requests.length,requests+1,'the pose poll reuses both bottom maps');
  assert.equal(d.images.length,2);
  await d.update({...state,robots:{...state.robots,acg720:{...peer,map_version:'session:3'}}});
  assert.equal(d.requests.filter(url=>url.includes('/api/map?view=')).length,3);
  assert.equal(d.images.length,3);
  assert.equal(d.commands.length,0);
});

test('a lost car hides its own current marker without swallowing a valid aligned ACG pose',async()=>{
  const state=peerState(), d=await dashboard(state);
  await d.update({...state,car:{...state.car,robot:null,
    localization:{...state.car.localization,state:'lost',reason:'laser_timeout'}}},null,1000);
  assert.equal(d.paint('carSwarmLocalMap').markers.length,0);
  assert.match(d.elements.swarmMemberCarPose.textContent,/定位已丢失/);
  assert.equal(peerDetail(d).name,'六足');close(peerDetail(d).x,3);close(peerDetail(d).y,2);
  assert.equal(d.commands.length,0);
});

test('peer events clear immediately on invalid alignment, membership, frame, pose or a failed state GET',async()=>{
  const state=peerState(), d=await dashboard(state);
  const changes=[
    {...state,anchor_locked:false},
    {...state,participants:['car']},
    {...state,participants:['acg720']},
    {...state,robots:{...state.robots,car:{...state.robots.car,selected:false}}},
    {...state,robots:{...state.robots,acg720:{...state.robots.acg720,selected:false}}},
    {...state,robots:{...state.robots,car:{...state.robots.car,aligned:false}}},
    {...state,robots:{...state.robots,acg720:{...state.robots.acg720,aligned:false}}},
    {...state,robots:{...state.robots,acg720:{...state.robots.acg720,online:false}}},
    {...state,robots:{...state.robots,car:{...state.robots.car,map_frame:'other_map'}}},
    {...state,robots:{...state.robots,acg720:{...state.robots.acg720,pose_common:{x:1,y:1,yaw:Infinity}}}},
    {...state,alignments:{car:{transform:[3,-2,NaN]}}},
    {...state,alignments:{car:{transform:[3,-2]}}},
    {...state,alignments:{}},
  ];
  for(const next of changes) {
    await d.update(next,null,1000);assert.equal(peerDetail(d),null);
    await d.update(state,null,1000);assert.notEqual(peerDetail(d),null);
  }
  d.network.available=false;
  await d.poll(300);
  assert.equal(d.elements.swarmConnection.textContent,'协同服务在线','recent state may still be displayed');
  assert.equal(peerDetail(d),null,'a failed current GET never republishes a cached peer as current');
  await d.poll(2600);assert.equal(peerDetail(d),null);
  assert.equal(d.elements.swarmConnection.textContent,'协同服务不可用');
  assert.equal(d.commands.length,0);
});
