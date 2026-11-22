/* The page uses the new Jetson mission, video and ROS telemetry interfaces. */
(() => {
  'use strict';
  const $ = id => document.getElementById(id);
  const text = (id, value) => { const el = $(id); if (el) el.textContent = value; };
  const number = (v, digits = 2) => Number.isFinite(v) ? v.toFixed(digits) : '—';
  const fresh = (s, limit = 1500) => s && s.age_ms >= 0 && s.age_ms < limit;
  const data = (name, limit) => fresh(telemetry[name], limit) ? telemetry[name].data : null;
  const video = $('networkVideo'), overlay = $('visionOverlay');
  const canvas = $('radarCloud'), context = canvas.getContext('2d');
  const localMaps = $('localMaps'), localMapRobot = $('localMapRobot');
  const localMapPreview = $('localMapPreview'), localMapDownload = $('localMapDownload');
  const localMapSources = {
    car:{name:'小车',path:'/local_map.png',file:'bx72-local-map.png'},
    acg720:{name:'六足',path:'/acg720/local_map.png',file:'acg720-local-map.png'}
  };
  const localMapMaxAge = 1500;
  let localMapSamples = {}, localMapRequest = null, localMapFrame = null;
  const history = [];
  let telemetry = {}, status = null, mission = null, online = false, armed = false, armPending = false;
  let mapImage = null, mapGeometry = null, mapLoading = false, mapOverlayActive = false;
  let mapSession = null, mapGeneration = 0, mapVersion = null, holdingRecoveryMap = false;
  let mapUpdateIssue = '';
  let peerPose = null;
  const waypoints = [];
  const tracks = {car:{points:[],continuing:false}};
  let waypointSubmitting = false, submittedWaypoints = null;
  let zoom = 1, panX = 0, panY = 0, mapLayout = null, drag = null;
  let videoWanted = true, videoGeneration = 0, videoBusy = false, videoUrl = null;
  let videoFrameId = '', videoDecodedAt = 0, videoWidth = 640, videoHeight = 360;
  let visionDraft = null, visionOptionsKey = '';
  let manualTimer = null, manualGeneration = 0, manualKey = null, manualInFlight = Promise.resolve(), stopPending = Promise.resolve();
  let pollRunning = false, mapUpdatedAt = 0, trendWindow = 30000;

  window.addEventListener('bx72:swarm-pose', event => {
    // The navigation canvas uses the car map; the coordinator alignment supplies the projection.
    peerPose = event.detail?.frame === 'map' ? event.detail : null;
    text('mapPeerPose', peerPose
      ? `六足位置（小车地图）：${number(peerPose.x)}, ${number(peerPose.y)} m · 朝向 ${number(peerPose.yaw*180/Math.PI,1)}°`
      : '六足位置：等待有效协同定位');
    drawMap();
  });

  async function request(path, options = {}, read = response => response.json()) {
    const controller = new AbortController();
    const timeout = setTimeout(() => controller.abort(), 2000);
    try {
      const response = await fetch(path, {...options, cache:'no-store', signal:controller.signal});
      if (!response.ok) throw new Error(await response.text() || `HTTP ${response.status}`);
      return await read(response);
    } finally { clearTimeout(timeout); }
  }

  async function post(path) {
    return (await request(path, {method:'POST', keepalive:true}, response => response.text())).trim();
  }

  function showEvent(message) {
    text('networkEvent', message);
    text('localTaskResult', message);
  }

  function setArmed(value) {
    armed = value;
    if (!value) window.HexapodManualControls?.releaseCar();
    const chip = $('networkDriveStatus');
    chip.classList.toggle('ready', value);
    chip.textContent = value ? '手动控制已允许 · 按住运动' : '网络仅监控 · 未允许控制';
    text('networkOwner', value ? '控制者：当前页面' : '控制者：未选择');
    text('networkArm', value ? '手动控制已允许' : '允许网络控制');
  }

  function releaseManual(reason, disarm = false) {
    const hadControl = manualKey !== null || manualTimer !== null || armed;
    manualKey = null;
    const generation = ++manualGeneration;
    if (manualTimer) clearTimeout(manualTimer);
    manualTimer = null;
    document.querySelectorAll('.network-key').forEach(b => b.classList.remove('active'));
    if (disarm) setArmed(false);
    if (!hadControl && !disarm) return stopPending;
    text('networkStopRequest', '停车请求：已发送');
    const first = post('/stop').then(() => text('networkStopRequest', '停车请求：已提交'))
      .catch(e => text('networkStopRequest', `停车请求失败：${e.message}`));
    stopPending = Promise.allSettled([first, manualInFlight]).then(() => {
      if (generation === manualGeneration) return post('/stop').catch(() => {});
    });
    if (reason) showEvent(reason);
    return stopPending;
  }
  window.HexapodManualControls?.registerCar(() => {
    if (armed || manualKey || armPending) releaseManual('已切换手控目标，已请求小车停车', true);
  });

  async function stopRobot(reason = '已请求停车，等待设备反馈') {
    await releaseManual(reason, true);
  }

  const motions = {w:[1,0,0], s:[-1,0,0], a:[0,1,0], d:[0,-1,0], q:[0,0,1], e:[0,0,-1]};
  async function beginManual(key) {
    if (window.HexapodManualControls && window.HexapodManualControls.keyboardOwner() !== 'car') return;
    if (!armed || !online || !motions[key] || manualKey === key) return;
    if (manualKey) return;
    const generation = ++manualGeneration;
    manualKey = key;
    document.querySelector(`[data-network-key="${key}"]`)?.classList.add('active');
    await stopPending;
    if (!armed || generation !== manualGeneration || manualKey !== key) return;
    const send = () => {
      if (!armed || generation !== manualGeneration || manualKey !== key) return;
      const started = performance.now();
      const v = Number($('networkSpeed').value) / 1000;
      const w = Number($('networkTurn').value) / 1000;
      const [x,y,z] = motions[key];
      manualInFlight = post(`/manual?x=${x*v}&y=${y*v}&yaw=${z*w}`)
        .catch(e => { if (generation === manualGeneration) releaseManual(`手动指令失败：${e.message}`, true); })
        .finally(() => {
          if (generation === manualGeneration && manualKey === key)
            manualTimer = setTimeout(send, Math.max(0, 60 - (performance.now() - started)));
        });
    };
    send();
  }

  $('networkArm').onclick = async () => {
    if (!online || armed || armPending || waypointSubmitting) return;
    window.HexapodManualControls?.selectCar();
    armPending = true; updateControls();
    try {
      const started = Date.now();
      showEvent('正在建立 FPGA 零速会话…');
      await post('/stop');
      let ready = false;
      while (Date.now() - started < 30000) {
        const current = await request('/api/state');
        if (window.HexapodManualControls && window.HexapodManualControls.keyboardOwner() !== 'car')
          throw new Error('手控目标已切换');
        const command = current.command, nav = current.navigation;
        const received = fresh(command, 500) && command.age_ms <= Date.now() - started + 10;
        if (received && command.data?.state === 'error') throw new Error(command.data.detail || '控制链路故障');
        // An idle session reports 76 once its manual command lease expires.
        if (received && command.data?.state === 'stop_requested' && fresh(nav, 500) &&
            nav.data?.boot_id && nav.data?.odom_domain && [0,76].includes(nav.data?.error) && nav.data?.mode === 1 &&
            (nav.data.flags & 2) && [nav.data.vx_mm_s,nav.data.vy_mm_s,nav.data.wz_mrad_s].every(v=>v===0)) {
          ready = true; break;
        }
        await new Promise(resolve => setTimeout(resolve, 150));
      }
      if (!ready) throw new Error('等待 FPGA 零速会话超时');
      setArmed(true);
      showEvent('手动控制已允许；按住方向键运动，松开停车');
    } catch (e) { window.HexapodManualControls?.releaseCar(); showEvent(`无法建立手动控制：${e.message}`); }
    finally {armPending = false; updateControls();}
  };
  ['networkStop', 'wbStop', 'localTaskStop', 'localTaskCancel'].forEach(id => {
    $(id).onclick = () => stopRobot();
  });
  document.querySelectorAll('.network-key').forEach(button => {
    const key = button.dataset.networkKey;
    button.onpointerdown = event => { event.preventDefault(); beginManual(key); };
    button.onpointerup = button.onpointercancel = button.onpointerleave = () => {
      if (manualKey === key) releaseManual('手动方向已松开，已请求停车');
    };
  });
  document.addEventListener('keydown', event => {
    if (event.repeat || /^(INPUT|TEXTAREA|SELECT)$/.test(event.target.tagName)) return;
    const key = event.key.toLowerCase();
    if (motions[key]) { event.preventDefault(); beginManual(key); }
    if (event.code === 'Space' && armed) { event.preventDefault(); stopRobot(); }
  });
  document.addEventListener('keyup', event => {
    if (event.key.toLowerCase() === manualKey) releaseManual('手动方向已松开，已请求停车');
  });
  document.addEventListener('visibilitychange', () => {
    if (document.hidden && (armed || manualKey)) releaseManual('页面不可见，已请求停车', true);
  });
  window.addEventListener('blur', () => {
    if (armed || manualKey) releaseManual('页面失去焦点，已请求停车', true);
  });
  ['networkSpeed','networkTurn'].forEach(id => {
    const update = () => text(`${id}Value`, `${$(id).value} ${id === 'networkSpeed' ? 'mm/s' : 'mrad/s'}`);
    $(id).addEventListener('input', update); update();
  });

  function mapPixel(point, m) {
    const dx = point[0] - m.origin_x, dy = point[1] - m.origin_y;
    const c = Math.cos(m.origin_yaw), s = Math.sin(m.origin_yaw);
    return [(dx*c + dy*s) / m.resolution, m.height - (-dx*s + dy*c) / m.resolution];
  }
  function sampleTrack(track, position) {
    if (!position || !Number.isFinite(position.x) || !Number.isFinite(position.y)) {
      track.continuing=false; return;
    }
    const counter=mission?.localization?.reset_counter;
    if(Number.isInteger(counter)) {
      if(Number.isInteger(track.resetCounter) && counter!==track.resetCounter) track.continuing=false;
      track.resetCounter=counter;
    }
    const last=track.points[track.points.length-1];
    if (!track.continuing || !last || Math.hypot(position.x-last.x,position.y-last.y)>=0.03) {
      track.points.push({x:position.x,y:position.y,breakBefore:!track.continuing});
    }
    track.continuing=true;
  }
  function updateTrackStatus() {
    const car=tracks.car.points.length;
    const pending=currentRobot()?'':` · ${localizationStatus()} · 历史轨迹已保留`;
    const unknown=tracks.car.points.some((p,i)=>i>0 && p.breakBefore)
      ? ' · 轨迹记录有缺口' : '';
    text('mapTrackStatus',`轨迹：小车 ${car} 点${pending}${unknown}`);
    text('trackCount',`轨迹采样：小车 ${car}`);
  }
  function currentRobot() {
    const p=mission?.robot;
    return mission?.localization?.state==='tracking' && p && [p.x,p.y,p.yaw].every(Number.isFinite) ? p : null;
  }
  function localizationStatus() {
    if(!mission) return '定位状态不可用';
    const localization=mission.localization;
    if(localization?.state==='lost') return '定位已丢失，正在重定位';
    if(localization?.state==='recovering') {
      const {confirmations,required_confirmations:required}=localization;
      return Number.isInteger(confirmations) && confirmations>=0 && Number.isInteger(required) && required>0
        ? `正在重定位，确认 ${confirmations}/${required}` : '正在重定位，等待确认';
    }
    if(localization?.state==='waiting') return '等待初始定位';
    return '当前位置暂不可用，等待有效定位';
  }
  function updatePoseReadouts() {
    const p=currentRobot();
    text('metricValueX',number(p?.x));text('metricValueY',number(p?.y));
    text('metricValueYaw',number(p?.yaw*180/Math.PI,1));
    text('mapPose',p?`${number(p.x)}, ${number(p.y)} m`:localizationStatus());
    text('poseHeading',p?`${number(p.yaw*180/Math.PI,1)}°`:'—');
  }
  function drawMap() {
    const width = Math.max(1, Math.round(canvas.clientWidth)), height = Math.max(1, Math.round(canvas.clientHeight));
    if (canvas.width !== width || canvas.height !== height) { canvas.width = width; canvas.height = height; }
    context.fillStyle = '#edf3f2'; context.fillRect(0, 0, width, height);
    const m = mapGeometry;
    if (!m) { mapLayout = null; return; }
    const pad = Math.ceil(0.5 / m.resolution);
    const scale = Math.min(width / (m.width + 2*pad), height / (m.height + 2*pad)) * zoom;
    const x = (width - m.width * scale) / 2 + panX, y = (height - m.height * scale) / 2 + panY;
    mapLayout = {m, x, y, scale};
    if (mapImage) {
      context.imageSmoothingEnabled = false; context.drawImage(mapImage, x, y, m.width*scale, m.height*scale);
    }
    context.lineCap='round';context.lineJoin='round';
    const pos = p => { const a = mapPixel(p,m); return [x+a[0]*scale, y+a[1]*scale]; };
    for (const [name,color] of [['car','#0891b2']]) {
      const points=tracks[name].points;
      context.beginPath();context.lineWidth=3;context.strokeStyle=color;context.setLineDash([]);
      points.forEach((p,i) => {
        const q=pos([p.x,p.y]);if (!i || p.breakBefore) context.moveTo(...q);else context.lineTo(...q);
      });
      context.stroke();
      if(points.some((p,i)=>i>0 && p.breakBefore)) {
        context.beginPath();context.setLineDash([6,4]);
        points.forEach((p,i)=>{
          if(i>0 && p.breakBefore) {context.moveTo(...pos([points[i-1].x,points[i-1].y]));context.lineTo(...pos([p.x,p.y]));}
        });
        context.stroke();context.setLineDash([]);
      }
    }
    if (mission?.route?.length) {
      context.beginPath(); context.lineWidth = 2; context.strokeStyle = '#7c3aed';
      mission.route.forEach((p, i) => { const q=pos(p); if (i) context.lineTo(...q); else context.moveTo(...q); });
      context.stroke();
    }
    function mark(p, color, radius, robot) {
      const q=pos(p); context.fillStyle=color;
      if(robot) {
        const angle=robot.yaw-m.origin_yaw, dx=Math.cos(angle), dy=-Math.sin(angle);
        context.beginPath();context.moveTo(q[0]+15*dx,q[1]+15*dy);
        context.lineTo(q[0]+7*dx-4*dy,q[1]+7*dy+4*dx);
        context.lineTo(q[0]+7*dx+4*dy,q[1]+7*dy-4*dx);context.fill();
      }
      context.beginPath(); context.strokeStyle='#fff';context.lineWidth=2;
      context.arc(q[0],q[1],radius,0,2*Math.PI); context.fill();context.stroke();
      if(robot) {
        context.font='bold 12px sans-serif';context.textBaseline='middle';context.textAlign='left';
        const labelWidth=context.measureText(robot.name).width;
        const labelX=Math.max(3,Math.min(width-labelWidth-3,q[0]+9));
        const labelY=Math.max(9,Math.min(height-9,q[1]+(robot.below?16:-16)));
        context.lineWidth=3;context.strokeText(robot.name,labelX,labelY);
        context.fillStyle=color;context.fillText(robot.name,labelX,labelY);
      }
    }
    const current=currentRobot();
    if (current) mark([current.x,current.y], '#0891b2', 5, {name:'小车',yaw:current.yaw});
    if (peerPose) mark([peerPose.x,peerPose.y], '#208b63', 5, {...peerPose,below:true});
    const task=mission?.waypoint_task;
    const showTaskPoints=Array.isArray(task?.points) && task.points.length &&
      (task.state==='running' || (task.state==='failed' && !waypoints.length));
    if (mission?.goal && task?.state!=='completed' && !showTaskPoints) mark(mission.goal, '#ef4444', 5);
    if (waypoints.length > 1) {
      context.beginPath(); context.lineWidth = 1.5; context.strokeStyle = '#d97706'; context.setLineDash([5,4]);
      waypoints.forEach((p,i) => { const q=pos(p); if (i) context.lineTo(...q); else context.moveTo(...q); });
      context.stroke(); context.setLineDash([]);
    }
    function numberedPoint(p,i,color,terminalYaw) {
      const q=pos(p);
      if (Number.isFinite(terminalYaw)) {
        // Padded footprint from car_navigation/grid_route.hpp, in map metres.
        const c=Math.cos(terminalYaw), s=Math.sin(terminalYaw);
        const project=(forward,left) => pos([p[0]+forward*c-left*s,p[1]+forward*s+left*c]);
        const corners=[project(.15,.145),project(.15,-.145),project(-.15,-.145),project(-.15,.145)];
        context.beginPath(); context.strokeStyle=color; context.lineWidth=1.5;
        context.moveTo(...corners[0]);
        for (const corner of corners.slice(1)) context.lineTo(...corner);
        context.lineTo(...corners[0]); context.stroke();
        context.beginPath(); context.moveTo(...q); context.lineTo(...project(.15,0)); context.stroke();
        const labelY=Math.min(...corners.map(corner=>corner[1]))-3;
        context.font='bold 12px sans-serif'; context.textAlign='center'; context.textBaseline='bottom';
        context.strokeStyle='#fff'; context.lineWidth=3; context.strokeText(String(i+1),q[0],labelY);
        context.fillStyle=color; context.fillText(String(i+1),q[0],labelY);
      } else {
        mark(p,color,11);
        context.fillStyle='#fff'; context.font='bold 11px sans-serif';
        context.textAlign='center'; context.textBaseline='middle'; context.fillText(String(i+1),q[0],q[1]);
      }
    }
    if (showTaskPoints) {
      task.points.forEach((point,i) => {
        if (i<task.completed || !Number.isFinite(point?.x) || !Number.isFinite(point?.y)) return;
        const color=i+1===task.current_index?'#dc2626':'#d97706';
        numberedPoint([point.x,point.y],i,color,i===task.points.length-1 ? point.yaw : undefined);
      });
    }
    waypoints.forEach((p,i) => {
      numberedPoint(p,i,'#d97706',i===waypoints.length-1 ? mission?.robot?.yaw : undefined);
    });
  }
  function renderWaypoints() {
    const list=$('waypointList');
    list.hidden=!waypoints.length;
    list.replaceChildren(...waypoints.map((point,index) => {
      const row=document.createElement('li'), label=document.createElement('span'), coords=document.createElement('span');
      const actions=document.createElement('span');
      label.className='waypoint-number'; label.textContent=String(index+1);
      coords.className='waypoint-coordinates'; coords.textContent=`${number(point[0],3)}, ${number(point[1],3)} m`;
      actions.className='waypoint-actions';
      [['↑','上移',index===0,() => { [waypoints[index-1],waypoints[index]]=[waypoints[index],waypoints[index-1]]; }],
       ['↓','下移',index===waypoints.length-1,() => { [waypoints[index],waypoints[index+1]]=[waypoints[index+1],waypoints[index]]; }],
       ['删除','删除',false,() => { waypoints.splice(index,1); }]].forEach(([caption,action,disabled,change]) => {
        const button=document.createElement('button'); button.type='button'; button.className='btn-ghost';
        button.textContent=caption; button.disabled=disabled || waypointSubmitting; button.setAttribute('aria-label',`${action}第 ${index+1} 个航点`);
        button.onclick=() => { change(); renderWaypoints(); drawMap(); updateControls(); };
        actions.append(button);
      });
      row.append(label,coords,actions); return row;
    }));
    text('waypointCount',`待提交航点：${waypoints.length}`);
    text('mapGoalHint',waypoints.length?`已选择 ${waypoints.length} 个航点。末点方框为车身及边距（30 × 29 cm），请为整个方框留出空地。朝向取提交时的车头方向。`:'');
    $('waypointClear').disabled=!waypoints.length || waypointSubmitting;
  }
  $('waypointClear').onclick=() => { waypoints.length=0; renderWaypoints(); drawMap(); updateControls(); };
  function updateMapStatus() {
    let message, state='stale';
    if (!online) message='地图服务不可用';
    else if (holdingRecoveryMap) message='显示上次导航地图 · 等待重新定位后的融合图';
    else if (mapUpdateIssue) {
      const retained=mapVersion?' · 显示上次导航地图':mapImage?' · 显示上次 SLAM 底图':'';
      message=mapUpdateIssue+retained;
    } else if (!mapImage) {
      state='unknown'; message='等待地图';
    } else if (mapVersion) {
      state='fresh'; message=mapOverlayActive?'地图实时 · 已叠加局部观测':'导航地图实时 · 局部未叠加';
    } else {
      const age=status?.map_age_ms;
      if (Number.isFinite(age) && age>=0 && age<5000) {
        state='fresh'; message='SLAM 底图 · 等待导航融合图';
      } else message='SLAM 底图 · 缺少新鲜地图状态';
    }
    text('mapStatus',message); $('mapStatus').dataset.state=state;
  }
  function acceptMapSession(next) {
    if (typeof next!=='string' || !next) return;
    const changed=mapSession!==null && mapSession!==next;
    mapSession=next;
    if (!changed) return;
    mapGeneration++;
    mapImage=mapGeometry=mapLayout=mapVersion=null;mapOverlayActive=holdingRecoveryMap=false;peerPose=null;
    mapUpdateIssue='';
    tracks.car.points=[];tracks.car.continuing=false;tracks.car.resetCounter=null;
    waypoints.length=0;submittedWaypoints=null;
    mission={...mission,route:[],goal:null};
    zoom=1;panX=panY=0;drag=null;mapUpdatedAt=0;
    renderWaypoints();
    text('mapPeerPose','六足位置：等待有效协同定位');
  }
  async function refreshMap() {
    const recoveryReady = () => currentRobot() && mission.map &&
      (mission.planning_local_map || mission.planning_laser_clear);
    if (mapLoading || (holdingRecoveryMap && !recoveryReady())) return;
    mapLoading = true;
    const generation=mapGeneration;
    const poseGeneration=mission?.localization?.reset_counter;
    let objectUrl = null;
    const finish=(issue='') => {
      if (objectUrl) URL.revokeObjectURL(objectUrl);
      mapLoading=false;
      if (generation===mapGeneration) { mapUpdateIssue=issue; updateMapStatus(); }
    };
    try {
      const {response, blob} = await request('/map.png', {},
        async response => ({response, blob:await response.blob()}));
      if (generation!==mapGeneration) {finish();return;}
      const version = response.headers.get('X-BX72-Map-Version');
      // An unversioned PNG is the raw SLAM fallback and may still contain cleared obstacles.
      if (!version && mapVersion) {finish('融合图暂不可用');return;}
      const values = (response.headers.get('X-BX72-Map') || '').split(',').map(Number);
      const [width, height, resolution, origin_x, origin_y, origin_yaw] = values;
      if (values.length !== 6 || !values.every(Number.isFinite) ||
          !Number.isInteger(width) || !Number.isInteger(height) ||
          width <= 0 || height <= 0 || resolution <= 0) throw new Error('invalid map geometry');
      const geometry = {width, height, resolution, origin_x, origin_y, origin_yaw};
      const overlayActive = response.headers.get('X-BX72-Map-Overlay') === 'active';
      objectUrl = URL.createObjectURL(blob);
      const nextImage = new Image();
      nextImage.onload = () => {
        if (generation!==mapGeneration || (holdingRecoveryMap && (!version || !recoveryReady() ||
            poseGeneration!==mission?.localization?.reset_counter))) {finish();return;}
        if (nextImage.naturalWidth!==width || nextImage.naturalHeight!==height) {finish('地图图像尺寸无效');return;}
        try { mapImage=window.BX72MapPalette.colorizeImage(nextImage); }
        catch (_) {finish('地图图像处理失败');return;}
        mapGeometry=geometry; mapVersion=version; holdingRecoveryMap=false; mapOverlayActive=overlayActive;
        finish(); drawMap();
      };
      nextImage.onerror = () => finish('地图图像解码失败');
      nextImage.src = objectUrl;
    } catch (_) {
      finish('地图更新失败');
    }
  }
  canvas.onpointerdown = event => {
    drag = {x:event.clientX,y:event.clientY,panX,panY,moved:false};
    canvas.setPointerCapture(event.pointerId);
  };
  canvas.onpointermove = event => {
    if (!drag) return;
    const dx=event.clientX-drag.x, dy=event.clientY-drag.y;
    if (Math.hypot(dx,dy)>4) drag.moved=true;
    if (drag.moved) { panX=drag.panX+dx; panY=drag.panY+dy; drawMap(); }
  };
  canvas.onpointerup = event => {
    if (!drag || !mapLayout) { drag=null; return; }
    if (!drag.moved) {
      if (waypointSubmitting) {drag=null;return;}
      const r=canvas.getBoundingClientRect(), {m,x,y,scale}=mapLayout;
      const clickX=event.clientX-r.left, clickY=event.clientY-r.top;
      let hit=-1, nearest=14;
      for (let i=waypoints.length-1;i>=0;i--) {
        const p=mapPixel(waypoints[i],m);
        const distance=Math.hypot(clickX-(x+p[0]*scale),clickY-(y+p[1]*scale));
        if (distance<=nearest && (hit<0 || distance<nearest)) {hit=i;nearest=distance;}
      }
      if (hit>=0) {
        waypoints.splice(hit,1);
        renderWaypoints();
        text('mapGoalHint',`已取消第 ${hit+1} 个待提交航点${waypoints.length?`，剩余 ${waypoints.length} 个`:''}。`);
        drawMap();updateControls();drag=null;return;
      }
      if (waypoints.length>=32) {text('mapGoalHint','最多添加 32 个航点，请先删除或清空。');drag=null;return;}
      const px=(clickX-x)/scale, py=(clickY-y)/scale;
      const lx=px*m.resolution, ly=(m.height-py)*m.resolution;
      const c=Math.cos(m.origin_yaw), s=Math.sin(m.origin_yaw);
      waypoints.push([m.origin_x+lx*c-ly*s,m.origin_y+lx*s+ly*c]);
      renderWaypoints(); drawMap(); updateControls();
    }
    drag=null;
  };
  canvas.onpointercancel = () => { drag=null; };
  $('mapZoomIn').onclick = () => { zoom=Math.min(5,zoom*1.25); drawMap(); };
  $('mapZoomOut').onclick = () => { zoom=Math.max(.4,zoom/1.25); drawMap(); };
  $('mapFit').onclick = () => { zoom=1;panX=panY=0;drawMap(); };
  $('mapLayerToggle').hidden = true;
  function localMapSample(age, version=null) {
    return Number.isFinite(age) && age>=0 ? {at:Date.now()-age, version} : null;
  }
  function clearLocalMap() {
    localMapRequest=localMapFrame=null;
    localMapPreview.hidden=localMapDownload.hidden=true;
  }
  function refreshLocalMap() {
    const id=localMapRobot.value, source=localMapSources[id], sample=localMapSamples[id];
    const age=sample ? Date.now()-sample.at : null;
    if (age===null || age>localMapMaxAge) {
      clearLocalMap();
      text('localMapStatus',age===null ? `等待${source.name}局部地图` : `${source.name}局部地图已过期`);
      return;
    }
    const frameAge=localMapFrame ? Date.now()-localMapFrame.at : null;
    if (frameAge===null || frameAge>localMapMaxAge) {
      localMapFrame=null;
      localMapPreview.hidden=localMapDownload.hidden=true;
    }
    text('localMapStatus',localMapPreview.hidden?'正在加载局部地图':`实时 · ${frameAge} ms`);
    if (!localMaps.open || localMapRequest || (sample.version && sample.version===localMapFrame?.version)) return;
    const image = new Image();
    localMapRequest=image;
    image.onerror=()=>{
      if (localMapRequest!==image) return;
      clearLocalMap();
      text('localMapStatus',`${source.name}局部地图暂不可用`);
    };
    image.onload=()=>{
      if (localMapRequest!==image) return;
      localMapRequest=null;
      const loadedAge=Date.now()-sample.at;
      if (loadedAge>localMapMaxAge) return;
      const colored=window.BX72MapPalette.colorizeImage(image);
      localMapPreview.width=colored.width;localMapPreview.height=colored.height;
      localMapPreview.getContext('2d').drawImage(colored,0,0);
      localMapFrame=sample;
      localMapPreview.hidden=localMapDownload.hidden=false;
      text('localMapStatus',`实时 · ${loadedAge} ms`);
    };
    image.src=source.path+(sample.version ? `?v=${encodeURIComponent(sample.version)}` : `?t=${Date.now()}`);
  }
  localMapRobot.onchange=()=>{
    const source=localMapSources[localMapRobot.value];
    localMapDownload.href=source.path;
    localMapDownload.download=source.file;
    localMapPreview.setAttribute('aria-label',`${source.name}实时局部地图`);
    clearLocalMap();
    refreshLocalMap();
  };
  localMaps.addEventListener('toggle',refreshLocalMap);
  $('localTaskStart').onclick = async () => {
    if ($('localTaskStart').disabled) return;
    waypointSubmitting=true;renderWaypoints();updateControls();
    const points=waypoints.slice(), generation=mapGeneration;
    try {
      const stopping=armed ? releaseManual('切换至航点任务', true) : stopPending;
      await stopping;
      if (stopping!==stopPending) throw Error('已请求停车，请重新提交航点');
      if (generation!==mapGeneration) throw Error('地图已更新，请重新选择航点');
      const yaw=mission?.robot?.yaw, task=mission?.waypoint_task;
      if (!online || !Number.isFinite(yaw)) throw Error('导航位姿不可用');
      if (points.length>1 && typeof task?.state!=='string') throw Error('多航点导航接口不可用');
      if (task?.state==='running') throw Error('多航点任务正在执行');
      const path=points.length===1?`/goal?x=${points[0][0]}&y=${points[0][1]}&yaw=${yaw}`:
        `/goal?waypoints=${encodeURIComponent(points.map(p=>p.join(',')).join(';'))}&yaw=${yaw}`;
      const message=await post(path);
      if (generation===mapGeneration)
        submittedWaypoints={points,previousTaskId:task?.task_id,retryAt:performance.now()+2000};
      showEvent(message);
    } catch (e) { showEvent(`航点提交失败：${e.message}`); }
    finally {waypointSubmitting=false;renderWaypoints();updateControls();}
  };
  $('exploreStart').onclick = async () => {
    if (armed) await releaseManual('切换至自主探索', true);
    try { showEvent(await post('/explore')); } catch (e) { showEvent(`探索提交失败：${e.message}`); }
  };

  function videoConnect() { videoWanted=true; videoGeneration++; videoFrameId=''; }
  function videoDisconnect() {
    videoWanted=false; videoGeneration++; videoDecodedAt=0; video.removeAttribute('src'); video.hidden=true;
    if (videoUrl) URL.revokeObjectURL(videoUrl); videoUrl=null;
    $('videoPlaceholder').hidden=false; text('videoStatus','视频：已断开');
  }
  $('videoConnect').onclick=videoConnect; $('videoDisconnect').onclick=videoDisconnect;
  $('videoRoutePreview').hidden=true; $('videoRouteToggle').hidden=true;
  async function loadVideoFrame() {
    if (!videoWanted || videoBusy) return;
    videoBusy=true;
    const generation=videoGeneration, controller=new AbortController();
    const timeout=setTimeout(()=>controller.abort(), 2000);
    try {
      const response=await fetch('/frame.jpg', {cache:'no-store', signal:controller.signal});
      if (!response.ok) throw new Error('视频帧不可用');
      const frameId=response.headers.get('X-BX72-Frame');
      if (!frameId || frameId===videoFrameId) return;
      const blob=await response.blob();
      if (!videoWanted || generation!==videoGeneration) return;
      const url=URL.createObjectURL(blob), image=new Image();
      await new Promise((resolve,reject)=>{image.onload=resolve;image.onerror=reject;image.src=url;})
        .catch(error=>{URL.revokeObjectURL(url);throw error;});
      if (!videoWanted || generation!==videoGeneration) {URL.revokeObjectURL(url);return;}
      const previous=videoUrl; videoUrl=url; videoFrameId=frameId; videoDecodedAt=Date.now();
      videoWidth=image.naturalWidth; videoHeight=image.naturalHeight;
      video.src=url; if (previous) URL.revokeObjectURL(previous);
    } catch (_) {
      // A later request retries after upstream, network, or JPEG decoding recovers.
    } finally {clearTimeout(timeout);videoBusy=false;}
  }
  function videoReady() { return videoWanted && videoDecodedAt && Date.now()-videoDecodedAt<1500; }
  function currentDetections() {
    return status && status.vision_age_ms>=0 && status.vision_age_ms<800 && videoReady()
      ? status.detections || [] : [];
  }
  function chooseDetection(det) {
    visionDraft=det ? {id:det.id, class_id:String(det.class)} : null;
    updateVisionControls();drawDetections();
  }
  overlay.onclick = event => {
    const rect=overlay.getBoundingClientRect(), scale=Math.min(rect.width/videoWidth,rect.height/videoHeight);
    const x=(event.clientX-rect.left-(rect.width-videoWidth*scale)/2)/scale;
    const y=(event.clientY-rect.top-(rect.height-videoHeight*scale)/2)/scale;
    const matches=currentDetections().filter(d=>x>=d.x-d.w/2 && x<=d.x+d.w/2 && y>=d.y-d.h/2 && y<=d.y+d.h/2);
    if (matches.length) chooseDetection(matches.sort((a,b)=>a.w*a.h-b.w*b.h)[0]);
  };
  $('visionDraftSelect').onchange = () => {
    const value=$('visionDraftSelect').value;
    chooseDetection(value===''?null:currentDetections()[Number(value)] || null);
  };
  $('visionBehavior').onchange = updateVisionControls;
  $('visionStart').onclick = async () => {
    if (armed) await releaseManual('切换至视觉任务', true);
    const behavior=$('visionBehavior').value, search=behavior==='search';
    const fields={behavior, class_id:search?$('visionClass').value:visionDraft?.class_id || '',
      target_id:search?'':visionDraft?.id || '', color:$('visionColor').value};
    const query=Object.entries(fields).map(([k,v])=>`${k}=${encodeURIComponent(v)}`).join('&');
    try { text('visionResult', await post(`/vision/start?${query}`)); }
    catch(e) {text('visionResult', `视觉任务请求失败：${e.message}`);}
  };
  $('visionStop').onclick = async () => {
    try {text('visionResult', await post('/vision/stop'));}
    catch(e) {text('visionResult', `停止视觉任务失败：${e.message}`);}
  };
  function updateVisionControls() {
    const state=data('vision_mission', 1500), detections=currentDetections();
    const active=state && ['searching','tracking','turning','following'].includes(state.state);
    const behavior=$('visionBehavior').value, search=behavior==='search';
    const present=visionDraft && detections.some(d=>d.id===visionDraft.id && String(d.class)===visionDraft.class_id);
    const key=JSON.stringify(detections.map(d=>[d.id,d.class]));
    const select=$('visionDraftSelect');
    if(key!==visionOptionsKey){
      const placeholder=document.createElement('option');placeholder.value='';placeholder.textContent='选择检测目标';
      const options=detections.map((d,i)=>{const o=document.createElement('option');o.value=String(i);o.textContent=`${d.class} · ${d.id}`;return o;});
      select.replaceChildren(placeholder,...options);visionOptionsKey=key;
    }
    const index=visionDraft?detections.findIndex(d=>d.id===visionDraft.id && String(d.class)===visionDraft.class_id):-1;
    select.value=index>=0?String(index):''; select.disabled=!!active || !detections.length;
    text('visionDraft',visionDraft?`已选择 ${visionDraft.id}${present?'':'（当前不可用）'}`:'未选择目标');
    const labels={idle:'空闲',searching:'搜索中',tracking:'跟踪中',turning:'转向中',following:'接近中',arrived:'保护停车',blocked:'受阻',stopped:'已停止'};
    text('visionStatus',state?`${labels[state.state] || state.state}：${state.reason || '—'}`:'视觉任务节点未连接');
    text('visionTarget',`检测 ${detections.length}${state?.target_id?` · 锁定 ${state.target_id}`:''}`);
    const observing=videoReady() && status?.vision_age_ms>=0 && status.vision_age_ms<800;
    $('visionStart').disabled=!online || !state || !observing || !!active || (!search && !present) || !['idle','arrived','blocked'].includes(mission?.mode);
    $('visionStop').disabled=!online || !state;
  }
  function drawDetections() {
    const rect=overlay.getBoundingClientRect();
    const w=Math.max(1,Math.round(rect.width)), h=Math.max(1,Math.round(rect.height));
    if (overlay.width!==w || overlay.height!==h) { overlay.width=w;overlay.height=h; }
    const ctx=overlay.getContext('2d');ctx.clearRect(0,0,w,h);
    if (!videoReady()) return;
    const scale=Math.min(w/videoWidth,h/videoHeight), x=(w-videoWidth*scale)/2, y=(h-videoHeight*scale)/2;
    ctx.lineWidth=2;ctx.font='13px sans-serif';
    for (const d of currentDetections()) {
      ctx.strokeStyle=d.id===visionDraft?.id?'#fbbf24':'#3be8a3';ctx.fillStyle=ctx.strokeStyle;
      ctx.strokeRect(x+(d.x-d.w/2)*scale,y+(d.y-d.h/2)*scale,d.w*scale,d.h*scale);
      ctx.fillText(`${d.class} ${d.id} ${number(d.score,2)}`,x+(d.x-d.w/2)*scale,y+(d.y-d.h/2)*scale-3);
    }
  }

  function drawTrend() {
    const c=$('telemetryTrend'), w=Math.max(1,c.clientWidth), h=Math.max(1,c.clientHeight);
    if (c.width!==w || c.height!==h) {c.width=w;c.height=h;}
    const ctx=c.getContext('2d');ctx.clearRect(0,0,w,h);
    const now=Date.now(), samples=history.filter(item=>item.at>=now-trendWindow);
    if (!samples.length) return;
    const max=Math.max(100,...samples.flatMap(s=>s.wheels.map(Math.abs)));
    ['#0e7490','#d97706','#7c3aed','#dc2626'].forEach((color,index)=>{
      ctx.beginPath();ctx.strokeStyle=color;ctx.lineWidth=1.5;
      samples.forEach((s,i)=>{const x=w-(now-s.at)*w/trendWindow,y=h/2-s.wheels[index]*(h*.45)/max;
        if(i)ctx.lineTo(x,y);else ctx.moveTo(x,y);});ctx.stroke();
    });
  }
  document.querySelectorAll('#trendRanges [data-range]').forEach(button=>button.onclick=()=>{
    trendWindow=Number(button.dataset.range)*1000;
    document.querySelectorAll('#trendRanges [data-range]').forEach(b=>{
      const active=b===button;b.classList.toggle('is-active',active);b.setAttribute('aria-pressed',String(active));
    });drawTrend();
  });
  $('clear').onclick=()=>{history.length=0;drawTrend();};

  function updateControls() {
    const nav=data('navigation',1000);
    $('networkArm').disabled=!online || !nav || ![0,76].includes(nav.error) || armed || armPending || waypointSubmitting;
    const task=mission?.waypoint_task, multiReady=typeof task?.state==='string', running=task?.state==='running';
    const pendingPoints=submittedWaypoints?.points;
    const sameSubmission=pendingPoints?.length===waypoints.length && waypoints.every((p,i)=>
      p[0]===pendingPoints[i][0] && p[1]===pendingPoints[i][1]);
    const awaiting=sameSubmission && performance.now()<submittedWaypoints.retryAt;
    $('localTaskStart').disabled=!online || !waypoints.length || waypointSubmitting || armPending || running || awaiting ||
      (waypoints.length>1 && !multiReady) || !Number.isFinite(mission?.robot?.yaw);
    text('waypointAvailability',waypointSubmitting?'正在提交航点任务…':running?'航点任务执行中':
      awaiting?'等待导航状态确认…':sameSubmission?'未收到导航确认，可重新提交':multiReady?'':'仅支持单航点');
    $('exploreStart').disabled=!online;
    $('localTaskStop').disabled=!online; $('localTaskCancel').disabled=!online;
  }
  function displayTelemetry() {
    const imu=data('imu'), scan=data('scan'), odom=data('odom'), motor=data('motor'), fusion=data('fusion'), nav=data('navigation');
    text('streamImu',`IMU：${imu?'实时':'不可用'}`);text('streamLidar',`雷达：${scan?'实时':'不可用'}`);
    text('streamFusion',`融合：${odom?'实时':'不可用'}`);text('streamMotor',`电机：${motor?'实时':'不可用'}`);
    text('streamHealth',`健康：${status?.fpga_health_age_ms>=0 && status.fpga_health_age_ms<2500?'实时':'不可用'}`);
    text('imuRaw',imu?`加速度 m/s²：${imu.accel.map(v=>number(v,3)).join(' / ')}\n角速度 rad/s：${imu.gyro.map(v=>number(v,3)).join(' / ')}`:'等待 IMU');
    text('networkWheelMapping',motor?`轮速 mm/s：${motor.measured_mm_s.join(' / ')}\n目标 mm/s：${motor.target_mm_s.join(' / ')}\n编码器：${motor.ticks.join(' / ')}`:'等待四轮遥测');
    text('flags',fusion?`融合标志：${fusion.flags} · 最大控制间隔：${number(fusion.max_control_gap_us/1000,1)} ms`:'等待融合状态');
    text('lastSeen',new Date().toLocaleTimeString());
    updatePoseReadouts();text('metricValueVx',number(odom?.vx));
    text('radarStats',scan?`前方最近：${number(scan.front_min_m)} m · 有效点 ${scan.valid_points}`:'等待雷达');
    text('networkReady',nav?`FPGA 状态：${nav.status} · 错误 ${nav.error}`:'FPGA 状态不可用');
    text('networkActualMode',nav?`FPGA 模式：${nav.mode}`:'FPGA 模式不可用');
    text('networkStopAck',data('command',2000)?`命令状态：${data('command',2000).state} · ${data('command',2000).detail}`:'命令状态：暂无反馈');
    const wheels=motor?.measured_mm_s;
    text('networkWheelsLow',wheels?`四轮反馈：${wheels.join(' / ')} mm/s`:'四轮反馈不可用');
    if (wheels?.length===4) {
      wheels.forEach((v,i)=>text(`wheelSpeed${i}`,String(v)));
      history.push({at:Date.now(),wheels});while(history.length && history[0].at<Date.now()-600000)history.shift();
      text('telemetryTrendStatus','实时轮速');drawTrend();
    }
    text('wbActualMode',`任务：${mission?.mode || '未知'} · FPGA 错误：${nav?.error ?? '未知'}`);
    text('localTaskState',mission?`${mission.mode}：${mission.reason || '—'}`:'等待任务状态');
    const task=mission?.waypoint_task;
    const taskLabel=task?.state==='running'?`正在前往第 ${task.current_index}/${task.total} 个航点 · 已完成 ${task.completed}/${task.total}`:
      task?.state==='completed'?`多航点任务已完成 · ${task.completed}/${task.total}`:
      task?.state==='failed'?`第 ${task.failed_index ?? task.current_index}/${task.total} 个航点失败：${task.failure || mission?.reason || '原因未知'}`:
      task?.state==='cancelled'?`多航点任务已取消 · 已完成 ${task.completed}/${task.total}`:
      mission?'暂无多航点任务':'多航点状态不可用';
    text('waypointProgress',taskLabel);
    text('localTaskFreshness',mission?'导航状态已连接':'导航状态不可用');
    const routeLabels={frontier:'前往观测边界',goal:'前往当前目标',waypoint_through:'连续航点',local_fallback:'局部绕障'};
    const routeKind=routeLabels[mission?.route_kind] || '导航路线';
    text('mapRouteStatus',mission?.route?.length?`紫线：${routeKind}（${mission.route.length} 点），随导航状态更新。`:'暂无导航端路线');
    const guardCode=Number.isInteger(fusion?.nav_status) ? (fusion.nav_status >>> 8) & 15 : null;
    const guardReasons={1:'等待运动或重新规划',2:'运动检查参数无效',3:'前向雷达数据过期',
      4:'前方保护',5:'横移速度不足',6:'局部区域未观测',7:'局部观测已过期',8:'局部区域被障碍占用'};
    const guardReason=nav?.status===5 ? (nav.error===78?'前方硬件保护停车':guardReasons[guardCode]) : null;
    text('localTaskPose',[nav?`FPGA 导航状态 ${nav.status} · 错误 ${nav.error}`:'FPGA 导航状态不可用',
      scan?`前方最近回波 ${number(scan.front_min_m)} m`:'雷达数据不可用',
      guardReason?`当前受阻：${guardReason}`:null].filter(Boolean).join(' · '));
    text('overviewTaskState',mission?`${mission.mode}：${mission.reason || '—'}`:'等待任务状态');
    text('plannerState',mission?.mode || '等待任务');
    text('plannerCommand',motor?`vx ${motor.vx_mm_s} · vy ${motor.vy_mm_s} · wz ${motor.wz_mrad_s}`:'等待电机遥测');
    updateVisionControls();
    text('diagnostics',JSON.stringify({navigation:nav,fusion,video:status},null,2));
    updateControls();
  }

  async function poll() {
    if (pollRunning) return; pollRunning=true;
    try {
      const values=await Promise.all(['/api/state','/status.json']
        .map(url=>request(url).catch(()=>null)));
      const acgMap=values[0]?.acg720_local_map;
      localMapSamples={car:localMapSample(values[1]?.local_map_age_ms),
        acg720:localMapSample(acgMap?.age_ms,acgMap?.data?.version)};
      refreshLocalMap();
      if (!values[0]) throw new Error('Jetson 遥测不可用');
      telemetry=values[0];status=values[1] || {};
      mission=fresh(telemetry.mission,2000)?telemetry.mission.data:null;
      acceptMapSession(mission?.map_session);
      const localization=mission?.localization;
      if (mapVersion && (localization?.state==='lost' || localization?.state==='recovering' ||
          (Number.isInteger(tracks.car.resetCounter) && Number.isInteger(localization?.reset_counter) &&
           tracks.car.resetCounter!==localization.reset_counter))) holdingRecoveryMap=true;
      if (armed && ((fresh(telemetry.command, 2000) && telemetry.command.data?.state === 'error') ||
          !fresh(telemetry.navigation, 500)))
        releaseManual('FPGA 控制链路失效，已请求停车', true);
      const task=mission?.waypoint_task;
      if (submittedWaypoints && Array.isArray(task?.points)) {
        const {points,previousTaskId}=submittedWaypoints;
        const matches=task.points.length===points.length && task.points.every((p,i)=>
          Math.abs(p.x-points[i][0])<1e-6 && Math.abs(p.y-points[i][1])<1e-6);
        const changed=typeof task.task_id==='string' && task.task_id!==previousTaskId;
        if (changed || (!task.task_id && task.state==='running' && matches)) {
          submittedWaypoints=null;
          if (matches) {
            waypoints.splice(0,waypoints.length,...waypoints.filter(point=>!points.includes(point)));
            renderWaypoints();
          }
        }
      }
      sampleTrack(tracks.car,currentRobot());updateTrackStatus();
      online=true;text('wbConnection','连接状态：已连接');text('networkConnection','新架构服务已连接');
      const videoFresh=status.last_jpeg_age_ms>=0 && status.last_jpeg_age_ms<1500 && videoReady();
      video.hidden=!videoFresh; $('videoPlaceholder').hidden=!video.hidden;
      text('videoStatus',video.hidden?'视频：等待画面':'视频：实时');
      text('videoMetrics',video.hidden?'等待新鲜 JPEG':`帧龄 ${status.last_jpeg_age_ms} ms · 收到 ${status.complete_frames} 帧`);
      $('videoConnect').disabled=videoWanted && videoFresh;$('videoDisconnect').disabled=!videoWanted;
      const now=Date.now();
      if (mapVersion && telemetry.map_version===mapVersion && !holdingRecoveryMap) {
        mapUpdateIssue='';
      } else {
        // A null version also means the server hasn't encoded a new navigation grid yet.
        const retryMs=!telemetry.map_version && (mapUpdateIssue || (mapImage && !mapVersion))?1000:150;
        if (now-mapUpdatedAt>=retryMs) { mapUpdatedAt=now; refreshMap(); }
      }
      updateMapStatus();
      displayTelemetry();drawMap();drawDetections();
    } catch (error) {
      if (online && (armed || manualKey)) releaseManual('Jetson 连接中断，已请求停车', true);
      online=false;mission=null;video.hidden=true;$('videoPlaceholder').hidden=false;
      sampleTrack(tracks.car,null);updateTrackStatus();updatePoseReadouts();
      text('mapStatus','地图服务不可用');$('mapStatus').dataset.state='stale';
      text('localTaskFreshness','导航状态不可用');text('localTaskState','等待任务状态');
      text('waypointProgress','多航点状态不可用');
      text('mapRouteStatus','导航状态不可用，暂无实时路线');text('localTaskPose','');drawMap();
      text('wbConnection','连接状态：未连接');text('networkConnection',`服务不可用：${error.message}`);
      text('videoStatus','视频：服务不可用');updateControls();
    } finally {pollRunning=false;}
  }

  text('wbTransport','接入方式：Jetson 新架构');text('wbSimulation','实机数据');
  text('networkTarget','FPGA 由 Jetson 新控制节点连接');text('networkSimulation','ROS 2 实机遥测');
  $('jetsonHost').disabled=true;$('jetsonPort').disabled=true;
  $('networkApplyTarget').disabled=true;$('networkDisconnect').disabled=true;
  $('networkConnect').onclick=poll;
  function openSettings(){const d=$('wbConnectionDialog');if(!d.open)d.showModal();}
  $('wbConfigure').onclick=openSettings;$('wbConnectionClose').onclick=()=>$('wbConnectionDialog').close();
  const targets={overview:'overview',tasks:'tasks',remote:'remotePage',sensors:'sensors',diagnostics:'diagnosticsPage'};
  document.querySelectorAll('[data-nav]').forEach(a=>a.addEventListener('click',event=>{
    const id=a.dataset.nav;if(id==='connectionSettings'){event.preventDefault();openSettings();return;}
    const target=$(targets[id]);if(!target)return;event.preventDefault();
    if(id!=='remote' && armed)releaseManual('切换页面，已请求停车',true);
    target.scrollIntoView({behavior:'smooth',block:'start'});
    document.querySelectorAll('.nav-item').forEach(n=>{
      const active=n.dataset.nav===id;
      n.classList.toggle('is-active',active);
      if(active)n.setAttribute('aria-current','page');else n.removeAttribute('aria-current');
    });
    text('pageBreadcrumb',a.textContent.trim().replace(/\d+$/,''));
  }));
  window.addEventListener('resize',()=>{drawMap();drawTrend();drawDetections();});
  setArmed(false);poll();setInterval(poll,200);loadVideoFrame();setInterval(loadVideoFrame,150);
})();
