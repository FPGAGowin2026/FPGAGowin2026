const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const test = require('node:test');
const vm = require('node:vm');

test('dashboard starts without motion and sends a final stop after a released command', async () => {
  const root = path.join(__dirname, '..', 'public');
  const html = fs.readFileSync(path.join(root, 'index.html'), 'utf8');
  const visionModes = [...html.match(/<select id="visionBehavior">([\s\S]*?)<\/select>/)[1].matchAll(/value="([^"]+)"/g)].map(match => match[1]);
  assert.deepEqual(visionModes, ['follow', 'approach', 'search']);
  const source = fs.readFileSync(path.join(root, 'js', 'robot_app.js'), 'utf8');
  const palette = fs.readFileSync(path.join(root, 'js', 'map_palette.js'), 'utf8');
  const manual = fs.readFileSync(path.join(root, 'js', 'hexapod_manual.js'), 'utf8');
  const ids = [...html.matchAll(/\bid="([^"]+)"/g)].map(match => match[1]);
  const noop = () => {};
  const mapDraws = [], strokes = [], mapMarkers = [], mapLabels = [], mapFills = [];
  let pathPoints=[], lineDash=[];
  const context2d = {fillRect:noop, clearRect:noop, beginPath:()=>{pathPoints=[];},
    lineTo:(x,y)=>pathPoints.push(['line',x,y]), moveTo:(x,y)=>pathPoints.push(['move',x,y]), setLineDash:value=>{lineDash=[...value];},
    stroke:()=>strokes.push({color:context2d.strokeStyle,points:[...pathPoints],dash:[...lineDash]}),
    arc:(x,y)=>mapMarkers.push({x,y,color:context2d.fillStyle}),
    fill:()=>{if(pathPoints.length)mapFills.push({color:context2d.fillStyle,path:[...pathPoints]});},
    drawImage:image=>mapDraws.push(image.source || image), strokeRect:noop, strokeText:noop,
    measureText:text=>({width:String(text).length*8}),
    fillText:(text,x,y)=>mapLabels.push({text,x,y,color:context2d.fillStyle})};
  const make = id => ({id, dataset:{}, style:{}, textContent:'', hidden:false, disabled:false,
    children:[], append(...children){this.children.push(...children);}, replaceChildren(...children){this.children=children;},
    clientWidth:400, clientHeight:300, width:0, height:0,
    value:id === 'networkSpeed' ? '100' : id === 'networkTurn' ? '300' : '',
    classList:{toggle:noop, add:noop, remove:noop}, setAttribute:noop, removeAttribute:noop,
    addEventListener:noop, setPointerCapture:noop, scrollIntoView:noop,
    getContext:() => context2d, getBoundingClientRect:() => ({left:0, top:0, width:400, height:300})});
  const elements = Object.fromEntries(ids.map(id => [id,make(id)]));
  elements.localMaps.open=true;
  elements.localMapRobot.value='car';
  elements.localMapPreview.hidden=elements.localMapDownload.hidden=true;
  const keys = 'qweasd'.split('').map(key => Object.assign(make(key),{dataset:{networkKey:key}}));
  const intervals = [];
  const buffer = () => {
    const c=make('buffer');
    c.getContext=()=>({drawImage:image=>{c.source=image;},
      getImageData:()=>({data:new Uint8ClampedArray(c.width*c.height*4)}),putImageData:noop});
    return c;
  };
  const documentListeners={};
  const document = {hidden:false, getElementById:id=>elements[id], createElement:tag=>tag==='canvas'?buffer():make('new'),
    addEventListener:(type,fn)=>{(documentListeners[type] ||= []).push(fn);}, querySelector:selector=>keys.find(k=>selector.includes(`"${k.dataset.networkKey}"`)),
    querySelectorAll:selector=>selector==='.network-key'?keys:[]};
  const posts = [], reads = [];
  let releaseManual;
  let frameId = null, stopReady = true, stopAge = 0, hangState = false, hangManual = false;
  const timeouts = new Map();
  let nextTimeout = 0;
  const untilAbort = signal => new Promise((_, reject) =>
    signal.addEventListener('abort', () => reject(new Error('request aborted')), {once:true}));
  const telemetry = {
    navigation:{age_ms:0,data:{status:0,error:76,mode:1,boot_id:1,odom_domain:1,
      flags:166,vx_mm_s:0,vy_mm_s:0,wz_mrad_s:0}},
    motor:{age_ms:0,data:{measured_mm_s:[0,0,0,0],target_mm_s:[0,0,0,0],ticks:[0,0,0,0],vx_mm_s:0,vy_mm_s:0,wz_mrad_s:0}},
    imu:{age_ms:0,data:{accel:[0,0,0],gyro:[0,0,0]}},
    odom:{age_ms:0,data:{vx:0}}, scan:{age_ms:0,data:{front_min_m:1,valid_points:100}},
    fusion:{age_ms:0,data:{flags:0,max_control_gap_us:20000}},
  };
  const status = {last_jpeg_age_ms:0, fpga_health_age_ms:0, map_age_ms:0, local_map_age_ms:0,
    video_instance:'1', complete_frames:1, vision_age_ms:0, detections:[]};
  const mission = {mode:'idle',reason:'',robot:{x:0,y:0,yaw:0},route:[],map_session:'first',
    localization:{state:'tracking',reason:'laser_tracking',confirmations:0,required_confirmations:3,reset_counter:0},
    map:{width:10,height:10,resolution:1,origin_x:0,origin_y:0,origin_yaw:0}};
  let mapHeader = '10,10,1,0,0,0', mapOverlay = 'active';
  let versionedMap = true;
  const mapVersion = () => versionedMap ? mapHeader + ':' + mapOverlay : null;
  let missionAvailable = true, carStatusAvailable = true;
  const acgMap={age_ms:80,data:{version:'scan-1'}};
  const response = value => ({ok:true, status:200, json:async()=>value, text:async()=>String(value)});
  const fetch = (url, options={}) => {
    if (options.method === 'POST') {
      posts.push(url);
      if (url.startsWith('/manual?')) return hangManual ? untilAbort(options.signal) :
        new Promise(resolve => { releaseManual=()=>resolve(response('ok')); });
      if (url === '/stop') {
        telemetry.command={age_ms:stopAge,data:{state:stopReady?'stop_requested':'stopping'}};
      }
      return Promise.resolve(response('ok'));
    }
    reads.push(url);
    if (url.startsWith('/map.png')) {
      const geometry = mapHeader;
      const overlay = mapOverlay;
      const version = mapVersion();
      return Promise.resolve({ok:true, headers:{get:name=>name === 'X-BX72-Map' ? geometry :
        name === 'X-BX72-Map-Overlay' ? overlay : name === 'X-BX72-Map-Version' ? version : null},
        blob:async()=>({geometry})});
    }
    if (url === '/frame.jpg') return Promise.resolve(frameId
      ? {ok:true, headers:{get:name=>name === 'X-BX72-Frame' ? frameId : null}, blob:async()=>({geometry:frameId})}
      : {ok:false});
    if (url === '/api/state' && hangState) return Promise.resolve({ok:true,json:()=>untilAbort(options.signal)});
    if (url === '/api/state') return Promise.resolve(response({
      ...telemetry, map_version:mapVersion(), acg720_local_map:acgMap,
      mission:{age_ms:missionAvailable?0:2000,data:mission}
    }));
    if (url === '/status.json' && !carStatusAvailable) return Promise.resolve({ok:false,text:async()=>'offline'});
    return Promise.resolve(response(status));
  };
  const images = [], localImages = [];
  class Image {
    constructor(){this.complete=true;this.naturalWidth=0;this.naturalHeight=0;}
    set src(value){
      this.url=value;this.complete=false;this.naturalWidth=0;this.naturalHeight=0;
      (value.includes('/local_map.png?')?localImages:images).push(this);
    }
    finish(width=10,height=10){this.complete=true;this.naturalWidth=width;this.naturalHeight=height;this.onload?.();}
    fail(){this.complete=true;this.onerror?.();}
  }
  let now = 10000;
  class Clock extends Date { static now(){return now;} }
  const revoked = [];
  const URL = {createObjectURL:blob=>`blob:${blob.geometry}`, revokeObjectURL:value=>revoked.push(value)};
  const windowListeners={};
  const sandbox = {document, window:{addEventListener:(type,fn)=>{windowListeners[type]=fn;}}, Image, URL, fetch, console, Math, Date:Clock,
    performance:{now:()=>now},
    AbortController,
    Promise, Number, String, Object,
    setTimeout:(fn,ms)=>{const id=++nextTimeout;timeouts.set(id,{fn,ms});return id;},
    clearTimeout:id=>timeouts.delete(id),
    setInterval:fn=>{intervals.push(fn);return intervals.length;}, clearInterval:noop};
  vm.runInNewContext(palette, sandbox, {filename:'map_palette.js'});
  vm.runInNewContext(manual, sandbox, {filename:'hexapod_manual.js'});
  vm.runInNewContext(source, sandbox, {filename:'robot_app.js'});
  const flush = async () => { for (let i=0;i<32;i++) await Promise.resolve(); };
  await flush();
  assert.ok(!reads.includes('/mission.json'),'mission status reuses the state request');
  assert.deepEqual(posts, [], 'loading telemetry must not issue a motion command');
  assert.equal(images.length, 1);
  assert.match(localImages.at(-1).url,/^\/local_map\.png\?/);
  localImages.at(-1).finish(64,64);
  assert.equal(elements.localMapPreview.hidden,false,'the FPGA local grid is colored and displayed');
  await intervals[0]();
  const pendingCar=localImages.at(-1);
  elements.localMapRobot.value='acg720';elements.localMapRobot.onchange();
  const hexapodMap=localImages.at(-1);
  assert.equal(hexapodMap.url,'/acg720/local_map.png?v=scan-1');
  pendingCar.finish(32,32);
  assert.equal(elements.localMapPreview.hidden,true,'the old robot image cannot appear under the new selection');
  hexapodMap.finish(64,64);
  assert.equal(elements.localMapPreview.hidden,false);
  assert.match(elements.localMapStatus.textContent,/实时/);
  assert.equal(elements.localMapDownload.href,'/acg720/local_map.png');
  const localReads=localImages.length;
  await intervals[0]();
  assert.equal(localImages.length,localReads,'the same ACG720 scan is not fetched again');
  acgMap.age_ms=1800;
  await intervals[0]();
  assert.equal(elements.localMapPreview.hidden,true);
  assert.match(elements.localMapStatus.textContent,/六足局部地图已过期/);
  elements.localMapRobot.value='car';elements.localMapRobot.onchange();
  await intervals[0]();
  assert.equal(elements.localMapDownload.href,'/local_map.png');
  localImages.at(-1).finish(64,64);
  images[0].finish();
  assert.match(elements.mapStatus.textContent,/已叠加局部观测/,'fresh local overlay is visible in the map status');
  windowListeners['bx72:swarm-pose']({detail:{frame:'map',name:'六足',x:2,y:3,yaw:Math.PI/2}});
  const peer=mapMarkers.filter(point=>point.color==='#208b63').at(-1);
  assert.equal(peer.x,125,'peer uses the existing car-map geometry');
  assert.equal(peer.y,200,'peer position respects the canvas Y direction');
  assert.ok(mapLabels.some(label=>label.text==='六足' && label.color==='#208b63'));
  assert.deepEqual(mapFills.filter(fill=>fill.color==='#208b63').at(-1).path[0],['move',125,185],
    'positive yaw points upward in the navigation canvas');
  assert.match(elements.mapPeerPose.textContent,/2\.00, 3\.00 m.*90\.0°/);
  const peerMarkerCount=mapMarkers.filter(point=>point.color==='#208b63').length;
  windowListeners['bx72:swarm-pose']({detail:{frame:'acg720_map',name:'六足',x:2,y:3,yaw:0}});
  windowListeners['bx72:swarm-pose']({detail:null});
  assert.equal(mapMarkers.filter(point=>point.color==='#208b63').length,peerMarkerCount,
    'unprojected or unavailable peer positions are not shown as current');
  assert.match(elements.mapPeerPose.textContent,/等待有效协同定位/);
  assert.deepEqual(posts,[],'displaying peer poses does not send control commands');
  now += 200;
  await intervals[0]();
  await flush();
  assert.equal(reads.filter(url=>url==='/map.png').length,1,'an unchanged map skips the image request');
  assert.equal(images.length,1,'an unchanged map skips image decoding');
  mapOverlay='unavailable'; versionedMap=false;
  const drawn = mapDraws.length;
  now += 200;
  await intervals[0]();
  await flush();
  assert.equal(images.length, 1, 'the raw SLAM fallback never replaces a decoded navigation map');
  assert.equal(mapDraws.length, drawn + 1, 'the previous map stays visible while loading');
  assert.equal(mapDraws.at(-1), images[0]);
  assert.match(elements.mapStatus.textContent,/融合图暂不可用.*上次导航地图/);
  assert.equal(elements.mapStatus.dataset.state,'stale');
  const fallbackReads=reads.filter(url=>url==='/map.png').length;
  now+=200;await intervals[0]();await flush();
  assert.equal(reads.filter(url=>url==='/map.png').length,fallbackReads);
  versionedMap=true;carStatusAvailable=false;
  now+=200;await intervals[0]();await flush();
  assert.equal(images.length,2,'a new navigation version is fetched even when the video service is unavailable');
  images[1].fail();
  assert.match(elements.mapStatus.textContent,/解码失败.*上次导航地图/);
  assert.equal(elements.mapStatus.dataset.state,'stale');
  now += 1000;
  await intervals[0]();
  await flush();
  assert.equal(images.length, 3, 'failed map downloads can be retried');
  assert.equal(mapDraws.at(-1), images[0], 'a failed download keeps the last map');
  images[2].finish();
  assert.equal(mapDraws.at(-1), images[2], 'the new map is drawn after loading');
  assert.match(elements.mapStatus.textContent,/局部未叠加/,'unavailable local observation is reported');
  carStatusAvailable=true;
  mission.robot.x=.5;
  await intervals[0]();await flush();
  assert.match(elements.mapTrackStatus.textContent,/小车 2 点/,'fresh map poses accumulate the car track');
  assert.ok(strokes.some(s=>s.color==='#0891b2' && s.points.some(p=>p[0]==='line')),'the car track is drawn on the map');
  const currentMarkers=()=>mapMarkers.filter(point=>point.color==='#0891b2').length;
  const markersBeforeLoss=currentMarkers();
  telemetry.fusion.data.scan_plane_valid=true;
  mission.localization={state:'lost',reason:'invalid_scan_plane',confirmations:0,required_confirmations:3};
  await intervals[0]();await flush();
  assert.match(elements.mapStatus.textContent,/等待重新定位后的融合图/);
  assert.match(elements.mapPose.textContent,/定位已丢失.*正在重定位/);
  assert.doesNotMatch(elements.mapPose.textContent,/倾斜|扫描平面|invalid_scan_plane/,
    'a retained failure reason must not claim the currently valid scan plane is tilted');
  assert.equal(elements.metricValueX.textContent,'—');
  assert.equal(currentMarkers(),markersBeforeLoss,'a cached robot object is not drawn as the current location while lost');
  assert.match(elements.mapTrackStatus.textContent,/小车 2 点.*历史轨迹已保留/);
  mission.localization={...mission.localization,state:'recovering',confirmations:2};
  await intervals[0]();await flush();
  assert.match(elements.mapPose.textContent,/正在重定位.*确认 2\/3/);
  assert.equal(currentMarkers(),markersBeforeLoss);
  assert.match(elements.mapTrackStatus.textContent,/小车 2 点/);
  mission.localization={...mission.localization,state:'tracking',reason:'laser_recovered'};
  mission.robot.x=.75;
  await intervals[0]();await flush();
  assert.equal(elements.mapPose.textContent,'0.75, 0.00 m');
  assert.ok(currentMarkers()>markersBeforeLoss);
  assert.match(elements.mapTrackStatus.textContent,/小车 3 点/);
  assert.doesNotMatch(elements.mapTrackStatus.textContent,/重定位/);
  const resumedCarPath=strokes.filter(line=>line.color==='#0891b2' && !line.dash.length).at(-1).points;
  assert.deepEqual(resumedCarPath.map(point=>point[0]),['move','line','move'],
    'position recovery preserves the old history and starts a separate segment');
  const gapPath=strokes.filter(line=>line.color==='#0891b2' && line.dash.length).at(-1);
  assert.deepEqual(gapPath.dash,[6,4]);
  assert.deepEqual(gapPath.points.map(point=>point[0]),['move','line']);
  assert.deepEqual(gapPath.points[0].slice(1),resumedCarPath[1].slice(1));
  assert.deepEqual(gapPath.points[1].slice(1),resumedCarPath[2].slice(1),'unknown path connects only the two observed endpoints');
  assert.match(elements.mapTrackStatus.textContent,/轨迹记录有缺口/);
  mission.localization={...mission.localization,reset_counter:1};mission.robot.x=.76;
  await intervals[0]();await flush();
  const resetPath=strokes.filter(line=>line.color==='#0891b2' && !line.dash.length).at(-1).points;
  assert.deepEqual(resetPath.map(point=>point[0]),['move','line','move','move'],
    'a changed recovery counter splits the solid path even if tracking was never absent from the page');
  assert.match(elements.mapTrackStatus.textContent,/小车 4 点/,'a reset records its observed endpoint without interpolated samples');
  assert.equal(posts.length,0,'localization display changes never submit a control request');
  assert.equal(elements.networkArm.disabled, false);
  await elements.networkArm.onclick();
  assert.equal(posts.at(-1), '/stop');
  keys.find(key=>key.dataset.networkKey==='w').onpointerdown({preventDefault:noop});
  await flush();
  assert.match(posts.at(-1), /^\/manual\?x=0\.1&y=0&yaw=0$/);
  now += 125;
  releaseManual();
  await flush();
  const nextManual = [...timeouts].find(([,timer])=>timer.ms===0);
  assert.ok(nextManual, 'a slow response should trigger the next send without waiting for an interval');
  timeouts.delete(nextManual[0]);nextManual[1].fn();
  await flush();
  assert.equal(posts.filter(path=>path.startsWith('/manual?')).length,2);
  keys.find(key=>key.dataset.networkKey==='w').onpointerup();
  await flush();
  assert.equal(posts.at(-1), '/stop', 'release requests an immediate stop');
  releaseManual();
  await flush();
  assert.equal(posts.at(-1), '/stop', 'in-flight motion is followed by another stop');
  assert.equal(posts.filter(item=>item==='/stop').length, 3);

  elements.radarCloud.onpointerdown({clientX:125,clientY:225,pointerId:1});
  elements.radarCloud.onpointerup({clientX:125,clientY:225,pointerId:1});
  assert.equal(elements.localTaskStart.disabled, false);
  const beforeToggle=posts.length;
  elements.radarCloud.onpointerdown({clientX:128,clientY:227,pointerId:1});
  elements.radarCloud.onpointerup({clientX:128,clientY:227,pointerId:1});
  assert.equal(elements.waypointList.children.length,0,'clicking the selected marker removes the draft waypoint');
  assert.equal(elements.localTaskStart.disabled,true);
  assert.equal(posts.length,beforeToggle,'deselecting a draft does not issue a stop or motion request');
  elements.radarCloud.onpointerdown({clientX:125,clientY:225,pointerId:1});
  elements.radarCloud.onpointerup({clientX:125,clientY:225,pointerId:1});
  elements.radarCloud.onpointerdown({clientX:150,clientY:200,pointerId:1});
  elements.radarCloud.onpointerup({clientX:150,clientY:200,pointerId:1});
  assert.equal(elements.waypointCount.textContent,'待提交航点：2');
  assert.equal(elements.localTaskStart.disabled,true,'multiple points wait for the navigation interface');
  const beforeQueueSubmit=posts.length;
  await elements.localTaskStart.onclick();
  assert.equal(posts.length,beforeQueueSubmit,'an unsupported queue never dispatches individual goals');
  elements.radarCloud.onpointerdown({clientX:125,clientY:225,pointerId:1});
  elements.radarCloud.onpointerup({clientX:125,clientY:225,pointerId:1});
  assert.equal(elements.waypointList.children.length,1);
  assert.equal(elements.waypointList.children[0].children[0].textContent,'1');
  assert.equal(elements.waypointList.children[0].children[1].textContent,'3.000, 3.000 m',
    'removing one waypoint preserves and renumbers the others');
  elements.radarCloud.onpointerdown({clientX:125,clientY:225,pointerId:1});
  elements.radarCloud.onpointerup({clientX:125,clientY:225,pointerId:1});
  elements.waypointList.children[0].children[2].children[1].onclick();
  assert.equal(posts.length,beforeQueueSubmit,'editing multiple points sends no command');
  elements.waypointList.children[1].children[2].children[0].onclick();
  assert.equal(elements.waypointList.children[0].children[1].textContent,'3.000, 3.000 m','waypoint order can be changed');
  elements.waypointList.children[0].children[2].children[2].onclick();
  assert.equal(elements.waypointList.children[0].children[1].textContent,'2.000, 2.000 m','deleting leaves the other point intact');
  await elements.localTaskStart.onclick();
  assert.equal(posts.at(-1), '/goal?x=2&y=2&yaw=0', 'map click becomes a map-frame goal');
  assert.equal(posts.at(-2), '/stop', 'manual mode is stopped before goal submission');

  mission.map.origin_x = 1;
  mission.planning_local_map=true;
  mapHeader = '10,10,1,1,0,0';
  now += 1000;
  await intervals[0]();
  await flush();
  assert.equal(mapDraws.at(-1), images[2], 'old image remains paired with its old origin');
  elements.waypointClear.onclick();
  elements.radarCloud.onpointerdown({clientX:125,clientY:225,pointerId:1});
  elements.radarCloud.onpointerup({clientX:125,clientY:225,pointerId:1});
  await elements.localTaskStart.onclick();
  assert.equal(posts.at(-1), '/goal?x=2&y=2&yaw=0', 'old pixels keep their original coordinates');
  images[3].finish();
  elements.waypointClear.onclick();
  elements.radarCloud.onpointerdown({clientX:125,clientY:225,pointerId:1});
  elements.radarCloud.onpointerup({clientX:125,clientY:225,pointerId:1});
  await elements.localTaskStart.onclick();
  assert.equal(posts.at(-1), '/goal?x=3&y=2&yaw=0', 'new image and new origin switch together');

  mission.map.width = 11;
  mapHeader = '11,10,1,1,0,0';
  now += 1000;
  await intervals[0]();
  await flush();
  assert.equal(mapDraws.at(-1), images[3], 'a resize keeps the previous map while loading');
  images[4].finish(11,10);
  assert.equal(mapDraws.at(-1), images[4], 'the resized image switches with matching geometry');
  mission.map.width = 12;
  mapHeader = '12,10,1,1,0,0';
  now += 1000;
  await intervals[0]();
  await flush();
  images[5].finish(11,10);
  assert.equal(mapDraws.at(-1), images[4], 'mismatched image dimensions keep the last valid pair');
  assert.match(elements.mapStatus.textContent,/图像尺寸无效.*上次导航地图/);
  mapHeader = 'invalid';
  now += 1000;
  await intervals[0]();
  await flush();
  assert.equal(images.length, 6, 'invalid geometry never replaces the map');
  assert.equal(revoked.length, 6, 'object URLs are released on success and failure');

  const beforeOutage = mapDraws.length;
  const beforePosts = posts.length;
  missionAvailable = false;
  now += 100;
  await intervals[0]();
  assert.equal(mapDraws.length, beforeOutage + 1, 'temporary mission outage keeps the map visible');
  assert.equal(mapDraws.at(-1), images[4]);
  assert.equal(elements.localTaskStart.disabled, true, 'goal requires a current robot heading');
  await elements.localTaskStart.onclick();
  assert.equal(posts.length, beforePosts, 'missing robot heading must not send a zero-yaw goal');
  missionAvailable = true;
  now += 100;
  await intervals[0]();
  assert.equal(elements.localTaskStart.disabled, false, 'goal becomes available after mission recovery');

  mission.waypoint_task={state:'idle',total:0,current_index:0,completed:0,points:[]};
  await intervals[0]();
  elements.radarCloud.onpointerdown({clientX:150,clientY:200,pointerId:1});
  elements.radarCloud.onpointerup({clientX:150,clientY:200,pointerId:1});
  assert.equal(elements.localTaskStart.disabled,false,'the advertised waypoint interface enables the queue');
  await elements.localTaskStart.onclick();
  const queuedRequest=new (require('node:url').URL)('http://example.test'+posts.at(-1));
  assert.equal(queuedRequest.pathname,'/goal');
  assert.equal(queuedRequest.searchParams.get('waypoints'),'3,2;4.5,3','the complete ordered queue is sent once');
  assert.equal(queuedRequest.searchParams.get('yaw'),'0');
  assert.equal(elements.waypointCount.textContent,'待提交航点：2','HTTP acceptance alone does not clear the draft');
  mission.waypoint_task={state:'running',total:2,current_index:1,completed:0,points:[{x:3,y:2},{x:4.5,y:3}]};
  await intervals[0]();
  assert.equal(elements.waypointCount.textContent,'待提交航点：0','navigation confirmation clears the matching draft');
  assert.match(elements.waypointProgress.textContent,/正在前往第 1\/2 个航点/);
  assert.equal(elements.localTaskStart.disabled,true,'an active waypoint task is not replaced');
  mission.waypoint_task={...mission.waypoint_task,state:'failed',failed_index:2,completed:1,failure:'路线受阻'};
  await intervals[0]();
  assert.match(elements.waypointProgress.textContent,/第 2\/2 个航点失败：路线受阻/);
  mission.waypoint_task={...mission.waypoint_task,state:'completed',completed:2};
  await intervals[0]();
  assert.match(elements.waypointProgress.textContent,/多航点任务已完成 · 2\/2/);
  mission.waypoint_task={...mission.waypoint_task,state:'cancelled',completed:1};
  await intervals[0]();
  assert.match(elements.waypointProgress.textContent,/多航点任务已取消 · 已完成 1\/2/);

  telemetry.vision_mission={age_ms:0,data:{state:'idle',reason:'',target_id:''}};
  status.detections=[{id:'session:7',class:'0',x:300,y:180,w:100,h:150,score:0.9}];
  frameId='camera:1';
  const frameRequest=intervals[1]();
  await flush();
  images.at(-1).finish(640,360);
  await frameRequest;
  await intervals[0]();
  elements.visionDraftSelect.value='0';elements.visionDraftSelect.onchange();
  for (const behavior of ['follow', 'approach']) {
    elements.visionBehavior.value=behavior;elements.visionBehavior.onchange();
    assert.equal(elements.visionStart.disabled,false,'fresh decoded frame and selected target enable the remaining task');
    await elements.visionStart.onclick();
    assert.equal(posts.at(-1), `/vision/start?behavior=${behavior}&class_id=0&target_id=session%3A7&color=`);
  }
  elements.visionBehavior.value='search';elements.visionClass.value='0';elements.visionColor.value='red';
  elements.visionDraftSelect.value='';elements.visionDraftSelect.onchange();
  assert.equal(elements.visionStart.disabled,false,'search-and-approach does not require a selected target ID');
  await elements.visionStart.onclick();
  assert.equal(posts.at(-1), '/vision/start?behavior=search&class_id=0&target_id=&color=red');
  await elements.visionStop.onclick();
  assert.equal(posts.at(-1),'/vision/stop');
  now+=1600;frameId=null;
  await intervals[0]();
  assert.equal(elements.visionStart.disabled,true,'stale browser image cannot be selected for a new task');
  assert.equal(elements.networkVideo.hidden,true);
  frameId='camera:2';
  const retry=intervals[1]();await flush();images.at(-1).finish(640,360);await retry;
  await intervals[0]();
  assert.equal(elements.networkVideo.hidden,false,'a new decoded frame restores the video');

  mapHeader='100,80,0.05,-1,2,1.5707963267948966';
  now+=1000;await intervals[0]();await flush();images.at(-1).finish(100,80);
  elements.waypointClear.onclick();
  const point=(x,y)=>{elements.radarCloud.onpointerdown({clientX:x,clientY:y,pointerId:1});
    elements.radarCloud.onpointerup({clientX:x,clientY:y,pointerId:1});};
  point(110,210);
  assert.equal(elements.waypointList.children[0].children[1].textContent,'-2.000, 3.000 m',
    'real map resolution and rotated origin produce the correct coordinates');
  // At this zoom, one metre spans 60 px and map +x points down the canvas.
  const endpointBox=(color='#d97706')=>strokes.filter(line=>line.color===color && line.points.length===5 &&
    !line.dash.length).at(-1).points.map(([,x,y])=>[+x.toFixed(2),+y.toFixed(2)]);
  assert.deepEqual(endpointBox(),[[118.7,219],[101.3,219],[101.3,201],[118.7,201],[118.7,219]],
    'preview uses the padded 30 x 29 cm body at the submit heading on the rotated map');
  assert.match(elements.mapGoalHint.textContent,/整个方框/);
  const heading=mission.robot.yaw;
  mission.robot.yaw=heading+Math.PI/2;
  await intervals[0]();
  assert.deepEqual(endpointBox(),[[119,201.3],[119,218.7],[101,218.7],[101,201.3],[119,201.3]],
    'turning the car updates the draft footprint while its centre stays fixed');
  mission.robot.yaw=heading;
  elements.mapZoomIn.onclick();point(87.5,225);
  assert.equal(elements.waypointList.children.length,0,'the same marker can be deselected after zooming');
  point(87.5,225);
  assert.equal(elements.waypointList.children[0].children[1].textContent,'-2.000, 3.000 m',
    'zoom preserves the map coordinate under the pointer');
  const zoomedBox=endpointBox();
  assert.ok(Math.abs(Math.abs(zoomedBox[0][0]-zoomedBox[1][0])-21.75)<0.02,
    'the outline scales with the real map through zoom');
  elements.radarCloud.onpointerdown({clientX:200,clientY:200,pointerId:1});
  elements.radarCloud.onpointermove({clientX:210,clientY:215,pointerId:1});
  elements.radarCloud.onpointerup({clientX:210,clientY:215,pointerId:1});
  assert.equal(elements.waypointList.children.length,1,'panning does not add or remove a waypoint');
  point(97.5,240);
  assert.equal(elements.waypointList.children.length,0,'the same marker can be deselected after panning');
  point(97.5,240);
  assert.equal(elements.waypointList.children[0].children[1].textContent,'-2.000, 3.000 m',
    'panning preserves the map coordinate under the pointer');
  elements.waypointClear.onclick();elements.mapFit.onclick();
  const previousTask=mission.waypoint_task;
  mission.waypoint_task={state:'running',current_index:1,total:1,completed:0,
    points:[{x:-2,y:3,yaw:Math.PI/2}]};
  await intervals[0]();
  assert.deepEqual(endpointBox('#dc2626'),[[119,201.3],[119,218.7],[101,218.7],[101,201.3],[119,201.3]],
    'the accepted endpoint uses its submitted yaw even when the robot is facing elsewhere');
  mission.waypoint_task=previousTask;
  for(let row=0;row<4;row++)for(let col=0;col<8;col++)point(40+40*col,40+40*row);
  assert.equal(elements.waypointList.children.length,32);
  const beforeFullToggle=posts.length;
  point(40,40);
  assert.equal(elements.waypointList.children.length,31,'a marker can be deselected when the queue is full');
  assert.equal(posts.length,beforeFullToggle,'editing a full queue sends no commands');
  telemetry.navigation.data.status=5;telemetry.navigation.data.error=0;
  telemetry.fusion.data.nav_status=(8<<8)|5;
  await intervals[0]();
  assert.match(elements.localTaskPose.textContent,/局部区域被障碍占用/,'the local guard reason is visible');
  telemetry.navigation.data.error=78;
  await intervals[0]();
  assert.match(elements.localTaskPose.textContent,/前方硬件保护停车/,'hardware stop is distinguished from local planning');
  status.local_map_age_ms=2000;await intervals[0]();
  assert.equal(elements.localMapPreview.hidden,true,'a stale local map is hidden');
  acgMap.age_ms=80;
  acgMap.data.version='scan-2';
  carStatusAvailable=false;
  elements.localMapRobot.value='acg720';elements.localMapRobot.onchange();
  await intervals[0]();
  assert.equal(localImages.at(-1).url,'/acg720/local_map.png?v=scan-2');
  localImages.at(-1).finish(64,64);
  assert.equal(elements.localMapPreview.hidden,false,'the ACG720 map stays live when car video status is offline');
  carStatusAvailable=true;
  await intervals[0]();

  const fireTimers = async ms => {
    for(const [id,timer] of [...timeouts]) if(timer.ms===ms) {timeouts.delete(id);timer.fn();}
    await flush();
  };
  stopReady=false;telemetry.navigation.data.error=76;
  const arming=elements.networkArm.onclick();await flush();
  assert.doesNotMatch(elements.networkDriveStatus.textContent,/已允许/,'HTTP acceptance without a worker stop ACK cannot authorize control');
  telemetry.command={age_ms:0,data:{state:'error',detail:'heartbeat not acknowledged'}};
  await fireTimers(150);await arming;
  assert.match(elements.networkEvent.textContent,/heartbeat not acknowledged/);
  const delayedCalibrationArm=elements.networkArm.onclick();await flush();
  assert.match(elements.networkEvent.textContent,/正在建立 FPGA 零速会话/);
  now+=24000;stopReady=true;
  telemetry.command={age_ms:0,data:{state:'stop_requested'}};
  await fireTimers(150);await delayedCalibrationArm;
  assert.match(elements.networkDriveStatus.textContent,/已允许/,'confirmed idle stop with lease-expiry code76 authorizes control');
  const idlePosts=posts.length;
  now+=5000;telemetry.command.age_ms=5000;
  await intervals[0]();
  assert.match(elements.networkDriveStatus.textContent,/已允许/,'idle lease expiry with confirmed stop remains armed');
  assert.equal(posts.length,idlePosts,'idle permission requires no continuous zero commands');
  telemetry.navigation.data.error=78;telemetry.navigation.data.flags=182;
  await intervals[0]();
  assert.match(elements.networkDriveStatus.textContent,/已允许/,'a forward guard stop preserves manual permission to reverse');
  telemetry.navigation.data.flags=174;
  await intervals[0]();
  assert.match(elements.networkDriveStatus.textContent,/已允许/,'directional missing returns preserve manual permission');
  telemetry.navigation.data.error=76;telemetry.navigation.data.flags=166;

  const rejectArm=async (label,change,restore)=>{
    await elements.networkStop.onclick();
    change();
    const pending=elements.networkArm.onclick();await flush();
    assert.doesNotMatch(elements.networkDriveStatus.textContent,/已允许/,label);
    now+=30001;await fireTimers(150);await pending;
    assert.doesNotMatch(elements.networkDriveStatus.textContent,/已允许/,label);
    restore();
  };
  await rejectArm('autonomous mode cannot authorize manual control',
    ()=>{telemetry.navigation.data.mode=2;},()=>{telemetry.navigation.data.mode=1;});
  await rejectArm('critical FPGA faults cannot authorize manual control',
    ()=>{telemetry.navigation.data.error=65;},()=>{telemetry.navigation.data.error=76;});
  await rejectArm('stale navigation cannot authorize manual control',
    ()=>{telemetry.navigation.age_ms=500;},()=>{telemetry.navigation.age_ms=0;});
  await rejectArm('invalid pose cannot authorize manual control',
    ()=>{telemetry.navigation.data.flags=164;},()=>{telemetry.navigation.data.flags=166;});
  await rejectArm('nonzero velocity cannot authorize manual control',
    ()=>{telemetry.navigation.data.vx_mm_s=1;},()=>{telemetry.navigation.data.vx_mm_s=0;});
  await rejectArm('zero boot identity cannot authorize manual control',
    ()=>{telemetry.navigation.data.boot_id=0;},()=>{telemetry.navigation.data.boot_id=1;});
  await rejectArm('zero odometry domain cannot authorize manual control',
    ()=>{telemetry.navigation.data.odom_domain=0;},()=>{telemetry.navigation.data.odom_domain=1;});
  await rejectArm('a stale stop ACK cannot authorize manual control',
    ()=>{stopAge=500;},()=>{stopAge=0;});
  await elements.networkArm.onclick();
  telemetry.navigation.age_ms=500;
  const stopsBeforeStaleNavigation=posts.filter(url=>url==='/stop').length;
  await intervals[0]();await flush();
  assert.doesNotMatch(elements.networkDriveStatus.textContent,/已允许/,'stale navigation revokes an existing permission');
  assert.ok(posts.filter(url=>url==='/stop').length>stopsBeforeStaleNavigation,'stale navigation still requests a stop');
  telemetry.navigation.age_ms=0;
  await elements.networkArm.onclick();
  telemetry.command={age_ms:7*60*1000,data:{state:'error',detail:'old heartbeat error'}};
  const postsBeforeOldError=posts.length;
  await intervals[0]();await flush();
  assert.match(elements.networkDriveStatus.textContent,/已允许/,'a seven-minute-old error does not revoke an existing permission');
  assert.equal(posts.length,postsBeforeOldError,'a stale command error does not send a stop request');
  telemetry.command={age_ms:0,data:{state:'error',detail:'heartbeat not acknowledged'}};
  const stopsBeforeFreshError=posts.filter(url=>url==='/stop').length;
  await intervals[0]();await flush();
  assert.doesNotMatch(elements.networkDriveStatus.textContent,/已允许/,'worker communication failure revokes permission');
  assert.ok(posts.filter(url=>url==='/stop').length>stopsBeforeFreshError,'a fresh command error still requests a stop');
  await elements.networkArm.onclick();

  hangManual=true;
  const manualCount=posts.filter(url=>url.startsWith('/manual?')).length;
  keys.find(key=>key.dataset.networkKey==='w').onpointerdown({preventDefault:noop});await flush();
  await fireTimers(2000);
  assert.equal(posts.filter(url=>url.startsWith('/manual?')).length,manualCount+1);
  assert.doesNotMatch(elements.networkDriveStatus.textContent,/已允许/,'a timed-out manual request disarms control');
  assert.equal(posts.at(-1),'/stop','manual timeout requests stop');
  hangManual=false;

  hangState=true;
  const stalledPoll=intervals[0]();await flush();await fireTimers(2000);await stalledPoll;
  assert.match(elements.wbConnection.textContent,/未连接/,'a stalled response body expires');
  hangState=false;await intervals[0]();
  assert.match(elements.wbConnection.textContent,/已连接/,'polling recovers after a timed-out request');

  await elements.networkArm.onclick();
  assert.match(elements.networkDriveStatus.textContent,/已允许/);
  sandbox.window.HexapodManualControls.create({
    id:'acg720',name:'ACG720 六足',url:'/acg720/api/control',keys:[],
    arm:elements.acg720Arm,stand:elements.acg720Stand,stop:elements.acg720Stop,
    result:elements.acg720ControlResult,drive:elements.acg720DriveStatus,panel:elements.acg720Remote,
    ready:()=>true,connected:()=>true,canStand:armed=>armed,message:()=> '已发送'
  });
  const beforeAcg=posts.length;
  elements.acg720Arm.onclick();await flush();
  assert.doesNotMatch(elements.networkDriveStatus.textContent,/已允许/,'selecting ACG720 disarms the actual car controller');
  assert.ok(posts.slice(beforeAcg).includes('/stop'),'switching from car requests car stop');
  const dispatchKey=(type,key)=>documentListeners[type].forEach(fn=>fn({key,code:`Key${key.toUpperCase()}`,
    target:{tagName:'BODY'},preventDefault:noop}));
  const carMoves=posts.filter(url=>url.startsWith('/manual?')).length;
  dispatchKey('keydown','w');await flush();dispatchKey('keyup','w');await flush();
  assert.equal(posts.filter(url=>url.startsWith('/manual?')).length,carMoves,'the car never responds to the selected ACG720 keyboard');
  assert.equal(posts.at(-1),'/acg720/api/control');
  await elements.networkArm.onclick();
  assert.equal(sandbox.window.HexapodManualControls.keyboardOwner(),'car');
  assert.doesNotMatch(elements.acg720DriveStatus.textContent,/当前手控目标/,'selecting car disarms ACG720');
  await elements.networkStop.onclick();
  stopReady=false;
  const pendingCarArm=elements.networkArm.onclick();await flush();
  elements.acg720Arm.onclick();await flush();
  await fireTimers(150);await pendingCarArm;
  assert.doesNotMatch(elements.networkDriveStatus.textContent,/已允许/,'late car arming cannot override the selected ACG720');
  assert.equal(sandbox.window.HexapodManualControls.keyboardOwner(),'acg720');

  mission.map_session='second';versionedMap=false;
  mapHeader='10,10,1,0,0,0';mapOverlay='unavailable';
  now+=200;await intervals[0]();await flush();
  const sessionBase=images.at(-1);sessionBase.finish();
  assert.equal(mapDraws.at(-1),sessionBase,'a new map session can display its initial SLAM base');
  assert.match(elements.mapStatus.textContent,/SLAM 底图.*等待导航融合图/);
  const rawReads=reads.filter(url=>url==='/map.png').length;
  now+=200;await intervals[0]();await flush();
  assert.equal(reads.filter(url=>url==='/map.png').length,rawReads,'raw-only operation avoids downloading every poll');
});
