const assert = require('node:assert/strict');
const test = require('node:test');
const {dashboard,robot,stateFor} = require('./swarm_fixture.cjs');

test('car + uncalibrated ACG720 keeps both local maps and independent manual controls available',async()=>{
  const state=stateFor(['car','acg720']);
  state.robots.car.camera_intrinsics_preliminary=true;
  const app=await dashboard(state);
  const e=app.elements;
  assert.equal(e.carSwarmLocalMap.hidden,false);
  assert.equal(e.acg720LocalMap.hidden,false,'camera calibration does not gate the laser map');
  assert.match(e.swarmMemberAcg720Calibration.textContent,/内参：待标定.*安装变换：待配置/);
  assert.match(e.swarmMemberCarCalibration.textContent,/初步参数.*待验证/);
  assert.match(e.swarmMemberAcg720Alignment.textContent,/等待相机内参与安装变换/);
  assert.match(e.swarmMemberAcg720Pose.textContent,/acg720_lidar/,'the laser reference is not labelled as a body pose');
  assert.equal(e.swarmExploreStart.disabled,true);
  assert.equal(e.acg720Arm.disabled,false,'this page does not change the separate ACG720 control state');
  assert.equal(app.commands.length,0,'loading mapping status never sends motion or starts a task');
});

test('a single ACG720 can supply a local map without claiming camera calibration or cross-robot alignment',async()=>{
  const state=stateFor(['acg720'],{anchor_locked:false,
    robots:{acg720:robot('acg720',{aligned:false,pose_common:null})},mapping_readiness:[]});
  const app=await dashboard(state);
  const e=app.elements;
  assert.equal(e.carSwarmLocalMapCard.hidden,true);
  assert.equal(e.acg720LocalMap.hidden,false);
  assert.match(e.swarmMemberAcg720State.textContent,/临时局部参考，未跨机对齐/);
  assert.match(e.swarmMemberAcg720Alignment.textContent,/等待相机内参与安装变换/);
  assert.match(e.swarmMemberAcg720Pose.textContent,/共同 未对齐/);
  assert.match(e.swarmMemberAcg720Calibration.textContent,/待标定/);
  assert.equal(e.swarmExploreStart.disabled,true);
  await app.update(state,null,1000);
  assert.equal(app.commands.length,0);
});

test('aligned car + ACG720 maps retain historical data when a member goes offline',async()=>{
  let state=stateFor(['car','acg720'],{mapping_status:'aligned',anchor_locked:true,
    mapping_readiness:[],
    robots:{car:robot('car',{aligned:true}),
      acg720:robot('acg720',{aligned:true,camera_model_ready:true,camera_transform_ready:true})}});
  const app=await dashboard(state), e=app.elements;
  assert.match(e.swarmMappingStatus.textContent,/地图已对齐/);
  assert.equal(e.swarmExploreStart.disabled,true,'automatic navigation capability is a separate concern');
  assert.equal(e.swarmReobserve.disabled,false);
  state={...state,robots:{...state.robots,acg720:{...state.robots.acg720,online:false,map_age_s:31}}};
  await app.update(state);
  assert.match(e.acg720LocalMapStatus.textContent,/保留上次地图.*保留最近地图/);
  assert.match(e.swarmMemberAcg720State.textContent,/当前离线/);
  assert.match(e.swarmParticipantsSummary.textContent,/离线：ACG720/);
  state={...state,robots:{...state.robots,acg720:{...state.robots.acg720,aligned:false}}};
  await app.update(state,null,1000);
  assert.equal(e.acg720LocalMap.hidden,false,'resetting visual alignment does not clear a local laser map');
  await app.disconnect();
  assert.equal(e.acg720LocalMap.hidden,true);
  assert.equal(e.carSwarmLocalMap.hidden,true);
  assert.equal(app.commands.length,0,'no readiness change sends a motion or stop request');
});

test('waiting map services do not invent a grid, and automatic exploration is enabled only by its own backend readiness',async()=>{
  const waiting=stateFor(['car','acg720'],{mapping_status:'waiting_maps',
    robots:{car:robot('car',{map_ready:false}),acg720:robot('acg720',{map_ready:false})}});
  const app=await dashboard(waiting), e=app.elements;
  assert.match(e.swarmMappingStatus.textContent,/等待 SLAM 地图/);
  assert.equal(e.acg720LocalMap.hidden,true);
  assert.equal(e.carSwarmLocalMap.hidden,true);
  assert.equal(app.requests.some(url=>url.includes('view=acg720')),false);
  const ready=stateFor(['car','acg720'],{mapping_status:'aligned',anchor_locked:true,mapping_readiness:[],readiness:[],
    robots:{car:robot('car',{aligned:true,auto_navigation_available:true}),
      acg720:robot('acg720',{aligned:true,camera_model_ready:true,camera_transform_ready:true})}});
  await app.update(ready);
  assert.equal(e.acg720LocalMap.hidden,false);
  assert.equal(e.swarmExploreStart.disabled,false,'ACG720 need not have an automatic navigation interface itself');
  assert.equal(e.acg720Arm.disabled,false);
  assert.equal(app.commands.length,0,'backend readiness does not start automatic exploration');
  e.swarmAutoDetails.open=false;
  await app.update({...ready,exploring:true});
  assert.equal(e.swarmAutoDetails.open,true,'an existing automatic task exposes its stop controls');
  assert.equal(e.swarmExploreStop.disabled,false);
  assert.equal(app.commands.length,0);
});

const moveCar=(state,local,common=null,frame=state.robots.car.map_frame)=>({...state,
  robots:{...state.robots,car:{...state.robots.car,map_frame:frame,pose_local:local,pose_common:common}},
  car:{...state.car,robot:local?{x:local[0],y:local[1],yaw:local[2]}:null}});
const carPaths=(app,id)=>app.paint(id).strokes.filter(line=>line.color==='#2074bb' && !line.dash.length).map(line=>line.path);
const carDashes=(app,id)=>app.paint(id).strokes.filter(line=>line.color==='#2074bb' && line.dash.length).map(line=>line.path);
const carDots=(app,id)=>app.paint(id).markers.filter(point=>point.color==='#2074bb');
const segments=path=>path.filter(point=>point.command==='lineTo').length;
const near=(actual,expected)=>assert.ok(Math.abs(actual-expected)<1e-9,`${actual} != ${expected}`);

test('car local map uses 3 cm samples while current dots update between grid fetches',async()=>{
  let state=stateFor(['car','acg720']);
  const app=await dashboard(state);
  assert.equal(segments(carPaths(app,'carSwarmLocalMap')[0]),0,'one stationary sample is not a travel line');
  const first=carDots(app,'carSwarmLocalMap')[0], requests=app.requests.length;
  state=moveCar(state,[.52,.5,0]);await app.update(state,null,1000);
  assert.equal(app.requests.length,requests+1,'a pose-only update reuses the last map grid');
  assert.equal(segments(carPaths(app,'carSwarmLocalMap')[0]),0,'movement under 3 cm is not resampled');
  near(carDots(app,'carSwarmLocalMap')[0].x-first.x,.02);
  state=moveCar(state,[.55,.5,0]);await app.update(state,null,1000);
  assert.equal(segments(carPaths(app,'carSwarmLocalMap')[0]),1);
  assert.equal(app.commands.length,0);
});

test('missing or cached car poses remove current dots immediately and split the resumed history',async()=>{
  let state=stateFor(['car','acg720']);
  const app=await dashboard(state);
  state=moveCar(state,[.6,.5,0]);await app.update(state);
  assert.equal(segments(carPaths(app,'carSwarmLocalMap')[0]),1);
  const lost=moveCar(state,null);
  await app.update(lost,null,1000);
  for(const id of ['carSwarmLocalMap']) {
    assert.equal(carDots(app,id).length,0,'no stale point remains while the grid is reused');
    assert.equal(segments(carPaths(app,id)[0]),1,'historical segments remain visible');
  }
  state=moveCar(state,[.9,.5,0]);await app.update(state,null,1000);
  const resumed=carPaths(app,'carSwarmLocalMap')[0];
  assert.deepEqual(resumed.map(p=>p.command),['moveTo','lineTo','moveTo']);
  const gap=carDashes(app,'carSwarmLocalMap')[0];
  assert.deepEqual(gap.map(p=>p.command),['moveTo','lineTo']);
  assert.deepEqual([gap[0].x,gap[0].y],[resumed[1].x,resumed[1].y]);
  assert.deepEqual([gap[1].x,gap[1].y],[resumed[2].x,resumed[2].y]);
  assert.match(app.elements.carSwarmLocalMapStatus.textContent,/路径中断/);
  assert.equal(carDots(app,'carSwarmLocalMap').length,1);
  await app.update({...state,car:{...state.car,robot:null}},null,1000);
  assert.equal(carDots(app,'carSwarmLocalMap').length,0,'a coordinator-cached pose_local is not a current Mission pose');
  state=moveCar(state,[1.,.5,0]);await app.update(state,null,1000);
  assert.deepEqual(carPaths(app,'carSwarmLocalMap')[0].map(p=>p.command),['moveTo','lineTo','moveTo','moveTo']);
  assert.equal(app.commands.length,0);
});

test('changing the car map frame starts a new frame without connecting old coordinates',async()=>{
  let state=stateFor(['car','acg720']);
  const app=await dashboard(state);
  state=moveCar(state,[.6,.5,0]);await app.update(state);
  state=moveCar({...state,frame:'new_map'},[100.,-50.,0],null,'new_map');
  await app.update(state);
  for(const id of ['carSwarmLocalMap']) {
    assert.equal(segments(carPaths(app,id)[0]),0);
    assert.equal(carPaths(app,id)[0].length,1,'the old map frame is not drawn in the new grid');
    assert.equal(carDashes(app,id).length,0,'different map frames never receive an unknown-path connector');
    assert.equal(carDots(app,id).length,1);
  }
  state=moveCar(state,[100.05,-50.,0]);await app.update(state,null,1000);
  assert.equal(segments(carPaths(app,'carSwarmLocalMap')[0]),1);
  assert.equal(app.commands.length,0);
});

test('off-grid car positions and trails receive display margin without changing map geometry or cells',async()=>{
  let state=moveCar(stateFor(['car','acg720']),[-.5,.5,0]);
  const app=await dashboard(state);
  state=moveCar(state,[-.55,.55,0]);await app.update(state);
  for(const id of ['carSwarmLocalMap']) {
    const dot=carDots(app,id)[0], canvas=app.elements[id];
    assert.ok(dot,'a valid body point outside the raster is still drawn');
    assert.ok(dot.x>=dot.radius && dot.x<=canvas.width-dot.radius);
    assert.ok(dot.y>=dot.radius && dot.y<=canvas.height-dot.radius);
    assert.equal(segments(carPaths(app,id)[0]),1);
    for(const point of carPaths(app,id)[0])
      assert.ok(point.x>0 && point.x<canvas.width && point.y>0 && point.y<canvas.height);
    assert.equal(app.paint(id).images[0].sourceWidth,2);
    assert.equal(app.paint(id).images[0].sourceHeight,2);
  }
  for(const grid of app.mapResponses) {
    assert.deepEqual([grid.width,grid.height,grid.resolution,...grid.origin],[2,2,1,0,0,0]);
    assert.equal(grid.cells,Buffer.from([1,101,0,51]).toString('base64'));
  }
  assert.equal(app.commands.length,0);
});

test('localization status explains lost and confirming car markers while keeping historical paths',async()=>{
  let state=stateFor(['car','acg720']);
  const app=await dashboard(state), e=app.elements;
  state=moveCar(state,[.6,.5,0]);await app.update(state);
  const lost={...state,car:{...state.car,
    localization:{state:'lost',reason:'invalid_scan_plane',confirmations:0,required_confirmations:3}}};
  await app.update(lost,null,1000);
  for(const id of ['carSwarmLocalMap']) {
    assert.equal(carDots(app,id).length,0,'even cached numeric body/local poses cannot create a current lost marker');
    assert.equal(segments(carPaths(app,id)[0]),1);
  }
  for(const id of ['carSwarmLocalMapStatus','swarmMemberCarPose']) {
    assert.match(e[id].textContent,/定位已丢失.*正在重定位.*历史轨迹已保留/);
    assert.doesNotMatch(e[id].textContent,/倾斜|扫描平面|invalid_scan_plane/);
  }
  assert.doesNotMatch(e.swarmMemberCarPose.textContent,/局部 0\.60/,'cached coordinates are not labelled as the current position');
  const confirming={...lost,car:{...lost.car,
    localization:{...lost.car.localization,state:'recovering',confirmations:2}}};
  await app.update(confirming,null,1000);
  for(const id of ['carSwarmLocalMapStatus','swarmMemberCarPose'])
    assert.match(e[id].textContent,/正在重定位.*确认 2\/3/);
  assert.equal(carDots(app,'carSwarmLocalMap').length,0);
  state=moveCar({...confirming,car:{...confirming.car,
    localization:{...confirming.car.localization,state:'tracking',reason:'laser_recovered'}}},[.7,.5,0]);
  await app.update(state,null,1000);
  assert.equal(carDots(app,'carSwarmLocalMap').length,1);
  assert.match(e.swarmMemberCarPose.textContent,/局部 0\.70, 0\.50 m/);
  assert.doesNotMatch(e.carSwarmLocalMapStatus.textContent,/重定位|历史轨迹已保留/);
  assert.deepEqual(carPaths(app,'carSwarmLocalMap')[0].map(p=>p.command),['moveTo','lineTo','moveTo']);
  assert.equal(app.commands.length,0);
});

test('recovery counters split solid samples without classifying sparse valid travel by distance',async()=>{
  let state=stateFor(['car','acg720']);
  const app=await dashboard(state);
  state=moveCar(state,[1.2,.5,0]);await app.update(state);
  assert.equal(segments(carPaths(app,'carSwarmLocalMap')[0]),1);
  assert.equal(carDashes(app,'carSwarmLocalMap').length,0,'distance alone does not classify a valid sample as a carry gap');
  state=moveCar({...state,car:{...state.car,localization:{...state.car.localization,reset_counter:1}}},[1.21,.5,0]);
  await app.update(state,null,1000);
  for(const id of ['carSwarmLocalMap']) {
    const solid=carPaths(app,id)[0], dashed=carDashes(app,id)[0];
    assert.deepEqual(solid.map(p=>p.command),['moveTo','lineTo','moveTo']);
    assert.equal(solid.length,3,'only actual observed positions are stored');
    assert.equal(dashed.length,2);
    assert.deepEqual([dashed[0].x,dashed[0].y],[solid[1].x,solid[1].y]);
    assert.deepEqual([dashed[1].x,dashed[1].y],[solid[2].x,solid[2].y]);
    assert.equal(carDots(app,id).length,1);
  }
  state=moveCar(state,[1.25,.5,0]);await app.update(state,null,1000);
  assert.deepEqual(carPaths(app,'carSwarmLocalMap')[0].map(p=>p.command),['moveTo','lineTo','moveTo','lineTo']);
  assert.equal(carDashes(app,'carSwarmLocalMap')[0].length,2,'new confirmed samples resume the solid path');
  assert.equal(app.commands.length,0);
});

test('network interruption uses a labelled unknown connector without asserting the car was carried',async()=>{
  let state=stateFor(['car','acg720']);
  const app=await dashboard(state);
  state=moveCar(state,[.6,.5,0]);await app.update(state);
  await app.disconnect();
  assert.equal(app.elements.carSwarmLocalMap.hidden,true);
  app.network.available=true;
  state=moveCar(state,[1.1,.5,0]);await app.update(state,null,1000);
  for(const id of ['carSwarmLocalMap']) {
    assert.equal(carPaths(app,id)[0].length,3);
    assert.equal(segments(carPaths(app,id)[0]),1,'the missing interval is not a measured solid segment');
    assert.equal(carDashes(app,id)[0].length,2);
  }
  for(const id of ['carSwarmLocalMapStatus']) {
    assert.match(app.elements[id].textContent,/路径中断/);
    assert.doesNotMatch(app.elements[id].textContent,/搬运/);
  }
  assert.equal(app.commands.length,0);
});

test('local robot markers have names, fixed dots, white outlines and real heading directions',async()=>{
  const state=stateFor(['car','acg720'],{anchor_locked:true,mapping_status:'aligned',
    robots:{car:robot('car',{aligned:true,pose_local:[0,0,0],pose_common:{x:0,y:0,yaw:0}}),
      acg720:robot('acg720',{aligned:true,pose_local:[0,0,Math.PI/2],pose_common:{x:0,y:0,yaw:Math.PI/2}})}});
  const app=await dashboard(state);
  const checkMarker=(id,color,name,heading)=>{
    const paint=app.paint(id), canvas=app.elements[id], dot=paint.markers.find(p=>p.color===color);
    assert.equal(dot.radius,5,'dot size does not shrink with the map resolution or extent');
    assert.ok(paint.strokes.some(s=>s.color==='#fff' && s.width===2 && s.path.some(p=>p.command==='arc' && p.radius===5)));
    const label=paint.texts.find(t=>t.operation==='fill' && t.text===name);
    assert.ok(label);assert.equal(label.color,color);assert.equal(label.align,'center');assert.equal(label.baseline,'middle');
    assert.ok(paint.texts.some(t=>t.operation==='stroke' && t.text===name && t.color==='#fff'));
    const halfWidth=Math.min(label.width,label.maxWidth)/2, size=Number(label.font.match(/([\d.]+)px/)[1]);
    assert.ok(label.x-halfWidth>=2 && label.x+halfWidth<=canvas.width-2,'the name fits even at a raster edge');
    assert.ok(label.y-size/2>=2 && label.y+size/2<=canvas.height-2);
    const arrow=paint.fills.find(f=>f.color===color && f.path.filter(p=>p.command==='lineTo').length===2);
    assert.ok(arrow,'a filled direction arrow accompanies the named dot');
    const tip=arrow.path[0];
    if(heading==='right') {assert.ok(tip.x>dot.x);near(tip.y,dot.y);}
    if(heading==='up') {near(tip.x,dot.x);assert.ok(tip.y<dot.y);}
    if(heading==='down') {near(tip.x,dot.x);assert.ok(tip.y>dot.y);}
    for(const point of arrow.path.filter(p=>p.command!=='closePath'))
      assert.ok(point.x>=1 && point.y>=1 && point.x<=canvas.width-1 && point.y<=canvas.height-1,
        'the white outlined heading also remains inside the canvas');
    return label;
  };
  checkMarker('carSwarmLocalMap','#2074bb','小车','right');
  checkMarker('acg720LocalMap','#208b63','六足','up');
  await app.update(state,{car:{origin:[0,0,Math.PI/2]},acg720:{origin:[0,0,Math.PI/2]}});
  checkMarker('carSwarmLocalMap','#2074bb','小车','down');
  checkMarker('acg720LocalMap','#208b63','六足','right');
  assert.equal(app.commands.length,0);
});
