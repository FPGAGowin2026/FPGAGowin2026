/* Source maps remain local until the coordinator reports an actual alignment. */
(() => {
  'use strict';
  const $ = id => document.getElementById(id);
  const text = (id, value) => { $(id).textContent = value; };
  const fmt = value => Number.isFinite(value) ? value.toFixed(2) : '—';
  function heading(yaw, robot) {
    const angle=yaw+(Number.isFinite(robot?.heading_offset_rad)?robot.heading_offset_rad:0);
    return Math.atan2(Math.sin(angle),Math.cos(angle));
  }
  const pose = (p, robot) => p && [p.x,p.y,p.yaw].every(Number.isFinite)
    ? `${fmt(p.x)}, ${fmt(p.y)} m · ${fmt(heading(p.yaw,robot))} rad` : '不可用';
  const arrayPose = (p, robot) => Array.isArray(p) && p.length >= 3 ? pose({x:p[0],y:p[1],yaw:p[2]},robot) : '不可用';
  let state = null, connected = false, busy = false;
  let connectionError = '', lastStateAt = 0;
  let lastMapAt = 0;
  let waitingStop = false, reobservePending = false;
  let carPoseCurrent = false;
  const carTrack = {frame:null,points:[],continuing:false};
  const mapViews = {};
  const members = {
    car:{name:'小车',prefix:'swarmMemberCar',canvas:'carSwarmLocalMap',status:'carSwarmLocalMapStatus',card:'carSwarmLocalMapCard',color:'#2074bb'},
    acg720:{name:'ACG720',prefix:'swarmMemberAcg720',canvas:'acg720LocalMap',status:'acg720LocalMapStatus',card:'acg720LocalMapCard',color:'#208b63'}
  };
  const participantState = (s=state) => Array.isArray(s?.participants) && s.robots && typeof s.robots==='object';
  const participantIds = (s=state) => participantState(s) ? s.participants.filter(id=>members[id] && s.robots[id]) : [];
  async function get(path) {
    const controller=new AbortController(), timer=setTimeout(()=>controller.abort(),2000);
    try {
      const response = await fetch(`/swarm${path}`, {cache:'no-store',signal:controller.signal});
      if (!response.ok) throw new Error(response.status === 404 ? '网页服务尚未加载协同接口' : `协同接口 HTTP ${response.status}`);
      return await response.json();
    } finally {clearTimeout(timer);}
  }
  async function post(path, body={}) {
    const response = await fetch(`/swarm${path}`, {method:'POST', cache:'no-store',
      headers:{'Content-Type':'application/json'}, body:JSON.stringify(body), keepalive:true});
    let result;
    try { result=await response.json(); } catch (_) { result={}; }
    if (!response.ok || result.accepted === false) throw new Error(result.error || `请求失败 HTTP ${response.status}`);
    return result;
  }
  function age(name) {
    const value=state?.robots?.[name]?.map_age_s;
    return Number.isFinite(value) && value >= 0 ? value + Math.max(0,Date.now()-lastStateAt)/1000 : null;
  }
  function mapAge(name) { const a=age(name);return a===null?'接收时间未知':`${fmt(a)} 秒前${a>30?'（保留上次地图）':''}`; }
  function acceptMapSessions(s) {
    for (const id of participantIds(s)) {
      const member=members[id], next=s.robots[id].map_session;
      if (typeof next!=='string' || !next) continue;
      const changed=member.mapSession!==undefined && member.mapSession!==next;
      member.mapSession=next;
      if (!changed) continue;
      delete mapViews[id];$(member.canvas).hidden=true;
      lastMapAt=0;
      carTrack.points=[];carTrack.continuing=false;carTrack.resetCounter=null;
    }
  }
  function carLocalPose(s) {
    const p=s?.robots?.car?.pose_local;
    return participantIds(s).includes('car') && s.robots.car.map_frame &&
      Array.isArray(p) && p.length>=3 && p.slice(0,3).every(Number.isFinite) ? p : null;
  }
  function currentCarPose(s) {
    const p=carLocalPose(s), current=s?.car?.robot;
    // The coordinator may retain a prior local pose after Mission loses tracking.
    return carPoseCurrent && s?.car?.localization?.state==='tracking' && p && current &&
      [current.x,current.y,current.yaw].every(Number.isFinite) ? p : null;
  }
  function publishPeerPose(s) {
    const ids=participantIds(s), car=s?.robots?.car, peer=s?.robots?.acg720;
    const p=peer?.pose_common, transform=s?.alignments?.car?.transform;
    let detail=null;
    if(connected && carPoseCurrent && s.anchor_locked===true &&
        ids.includes('car') && ids.includes('acg720') && car.selected===true && peer.selected===true &&
        car.aligned===true && peer.aligned===true && peer.online===true && car.map_frame==='map' &&
        p && [p.x,p.y,p.yaw].every(Number.isFinite) && Array.isArray(transform) &&
        transform.length===3 && transform.every(Number.isFinite)) {
      // The alignment maps car-local into common; the main map needs its inverse.
      const [tx,ty,angle]=transform, c=Math.cos(angle), sn=Math.sin(angle), dx=p.x-tx, dy=p.y-ty;
      detail={frame:'map',name:'六足',x:c*dx+sn*dy,y:-sn*dx+c*dy,
        yaw:heading(p.yaw-angle,peer)};
    }
    window.dispatchEvent(new CustomEvent('bx72:swarm-pose',{detail}));
  }
  function carLocalizationStatus(s) {
    if(!carPoseCurrent || !s?.car) return '小车定位状态不可用';
    const localization=s.car.localization;
    if(localization?.state==='lost') return '小车定位已丢失，正在重定位';
    if(localization?.state==='recovering') {
      const {confirmations,required_confirmations:required}=localization;
      return Number.isInteger(confirmations) && confirmations>=0 && Number.isInteger(required) && required>0
        ? `小车正在重定位，确认 ${confirmations}/${required}` : '小车正在重定位，等待确认';
    }
    if(localization?.state==='waiting') return '小车等待初始定位';
    return '小车当前位置暂不可用，等待有效定位';
  }
  function sampleCarTrack(s) {
    const frame=s?.robots?.car?.map_frame;
    if(frame && frame!==carTrack.frame) {
      carTrack.frame=frame;carTrack.points=[];carTrack.continuing=false;carTrack.resetCounter=null;
    }
    const p=currentCarPose(s);
    if(!p) {carTrack.continuing=false;return;}
    const counter=s.car.localization.reset_counter;
    if(Number.isInteger(counter)) {
      if(Number.isInteger(carTrack.resetCounter) && counter!==carTrack.resetCounter) carTrack.continuing=false;
      carTrack.resetCounter=counter;
    }
    const last=carTrack.points[carTrack.points.length-1];
    if(!carTrack.continuing || !last || Math.hypot(p[0]-last.x,p[1]-last.y)>=.03)
      carTrack.points.push({x:p[0],y:p[1],breakBefore:!carTrack.continuing});
    carTrack.continuing=true;
  }
  function localCarTrack(frame) {
    return frame===carTrack.frame ? carTrack.points : [];
  }
  function unknownPathStatus(points) {
    return points.some((p,i)=>i>0 && p.breakBefore)
      ? ' · 路径中断' : '';
  }
  function pointPixel(grid, x, y) {
    const [ox,oy,a]=grid.origin, c=Math.cos(a), s=Math.sin(a), dx=x-ox, dy=y-oy;
    return [(c*dx+s*dy)/grid.resolution,grid.height-(-s*dx+c*dy)/grid.resolution];
  }
  function makeMapView(grid) {
    if (!grid || !Number.isInteger(grid.width) || !Number.isInteger(grid.height) || grid.width<=0 || grid.height<=0 ||
        grid.width*grid.height>4000000 || !Number.isFinite(grid.resolution) || grid.resolution<=0 ||
        !Array.isArray(grid.origin) || grid.origin.length!==3 || !grid.origin.every(Number.isFinite)) throw Error('地图格式无效');
    const {cells,...geometry}=grid;
    const bytes=atob(cells);
    if(bytes.length!==grid.width*grid.height) throw Error('地图数据长度不匹配');
    const raster=document.createElement('canvas');raster.width=grid.width;raster.height=grid.height;
    const rasterContext=raster.getContext('2d'), pixels=rasterContext.createImageData(grid.width,grid.height);
    for(let y=0;y<grid.height;y++) for(let x=0;x<grid.width;x++){
      const v=bytes.charCodeAt(y*grid.width+x);
      if(v>101) throw Error('地图栅格值无效');
      const i=((grid.height-1-y)*grid.width+x)*4;
      window.BX72MapPalette.writeOccupancy(pixels.data,i,v-1);
    }
    rasterContext.putImageData(pixels,0,0);
    return {grid:geometry,raster};
  }
  function drawGrid(canvas, grid, raster, markers, trail=[]) {
    const valid=point=>point && Number.isFinite(point.x) && Number.isFinite(point.y);
    const dots=markers.filter(valid), points=trail.filter(valid);
    let minX=0,minY=0,maxX=grid.width,maxY=grid.height;
    for(const point of [...dots,...points]) {
      const [x,y]=pointPixel(grid,point.x,point.y);
      minX=Math.min(minX,x);minY=Math.min(minY,y);maxX=Math.max(maxX,x);maxY=Math.max(maxY,y);
    }
    // Expand the display view to include the body; the grid and its frame stay unchanged.
    const padding=9, scale=Math.min(grid.width/(maxX-minX),grid.height/(maxY-minY));
    const width=grid.width+2*padding, height=grid.height+2*padding;
    if(canvas.width!==width) canvas.width=width;
    if(canvas.height!==height) canvas.height=height;
    const offsetX=(canvas.width-(maxX-minX)*scale)/2-minX*scale;
    const offsetY=(canvas.height-(maxY-minY)*scale)/2-minY*scale;
    const screen=point=>{const [x,y]=pointPixel(grid,point.x,point.y);return [offsetX+x*scale,offsetY+y*scale];};
    const ctx=canvas.getContext('2d');
    ctx.fillStyle='#edf3f2';ctx.fillRect(0,0,canvas.width,canvas.height);
    ctx.imageSmoothingEnabled=false;ctx.drawImage(raster,offsetX,offsetY,grid.width*scale,grid.height*scale);
    if(points.length) {
      ctx.beginPath();ctx.lineWidth=2;ctx.strokeStyle=members.car.color;ctx.lineCap='round';ctx.lineJoin='round';ctx.setLineDash([]);
      points.forEach((point,i)=>{const p=screen(point);if(!i || point.breakBefore)ctx.moveTo(...p);else ctx.lineTo(...p);});
      ctx.stroke();
      if(points.some((point,i)=>i>0 && point.breakBefore)) {
        ctx.beginPath();ctx.setLineDash([6,4]);
        points.forEach((point,i)=>{
          if(i>0 && point.breakBefore) {ctx.moveTo(...screen(points[i-1]));ctx.lineTo(...screen(point));}
        });
        ctx.stroke();ctx.setLineDash([]);
      }
    }
    for(const marker of dots){
      const [x,y]=screen(marker);
      ctx.fillStyle=marker.color;ctx.strokeStyle='#fff';ctx.lineWidth=2;
      if(Number.isFinite(marker.yaw)) {
        const angle=marker.yaw-grid.origin[2], dx=Math.cos(angle), dy=-Math.sin(angle);
        const reach=Math.min(12,dx>0?(canvas.width-2-x)/dx:dx<0?(2-x)/dx:Infinity,
          dy>0?(canvas.height-2-y)/dy:dy<0?(2-y)/dy:Infinity);
        const base=Math.max(0,reach-5);
        ctx.beginPath();ctx.moveTo(x+reach*dx,y+reach*dy);
        ctx.lineTo(x+base*dx-3*dy,y+base*dy+3*dx);
        ctx.lineTo(x+base*dx+3*dy,y+base*dy-3*dx);ctx.closePath();ctx.fill();ctx.stroke();
      }
      ctx.beginPath();ctx.arc(x,y,5,0,2*Math.PI);ctx.fill();ctx.stroke();
    }
    for(const marker of dots) {
      const [x,y]=screen(marker), fontSize=Math.min(12,(canvas.height-8)/2);
      ctx.font=`bold ${fontSize}px sans-serif`;ctx.textAlign='center';ctx.textBaseline='middle';
      const maxWidth=canvas.width-4, halfWidth=Math.min(ctx.measureText(marker.name).width,maxWidth)/2;
      const labelX=Math.max(halfWidth+2,Math.min(canvas.width-halfWidth-2,x));
      const offset=marker.name==='小车'?-(fontSize+4):fontSize+4;
      const labelY=Math.max(fontSize/2+2,Math.min(canvas.height-fontSize/2-2,y+offset));
      ctx.lineWidth=3;ctx.strokeStyle='#fff';ctx.strokeText(marker.name,labelX,labelY,maxWidth);
      ctx.fillStyle=marker.color;ctx.fillText(marker.name,labelX,labelY,maxWidth);
    }
    canvas.hidden=false;
  }
  function paintMap(name, {grid,raster}, s) {
    const member=members[name], robot=s.robots[name], canvas=$(member.canvas);
    if(grid.mode!==`${name}_local` || grid.frame!==robot.map_frame) throw Error('地图坐标系不匹配');
    const p=name==='car'?currentCarPose(s):robot.pose_local;
    const history=name==='car'?localCarTrack(grid.frame):[];
    drawGrid(canvas,grid,raster,p?[{x:p[0],y:p[1],yaw:heading(p[2],robot),name:name==='car'?'小车':'六足',color:member.color}]:[],history);
    const location=name==='car' && !p?` · ${carLocalizationStatus(s)} · 历史轨迹已保留`:'';
    text(member.status,`${grid.frame} · ${mapAge(name)}${robot.online?'':' · 保留最近地图'}${robot.pose_reference?' · 参考 '+robot.pose_reference:''}${location}${unknownPathStatus(history)}`);
  }
  function mapError(name,error) {
    delete mapViews[name];
    $(members[name].canvas).hidden=true;text(members[name].status,`${members[name].name}地图不可用：${error.message}`);
  }
  function paintCachedMaps() {
    if(!$('swarmSourceMaps').open) return;
    for(const [name,view] of Object.entries(mapViews)) {
      if(!participantIds().includes(name) || !state.robots[name].map_ready) {
        $(members[name].canvas).hidden=true;delete mapViews[name];continue;
      }
      try {paintMap(name,view,state);} catch(error) {mapError(name,error);}
    }
  }
  async function refreshMaps() {
    const cohort=state, ids=participantIds(cohort);
    const requests=$('swarmSourceMaps').open ? ids.map(async id=>{
      const member=members[id], robot=cohort.robots[id], canvas=$(member.canvas);
      if(!robot.map_ready) {
        delete mapViews[id];canvas.hidden=true;text(member.status,`${member.name}：等待本地建图服务发布地图`);return;
      }
      if(robot.map_version && robot.map_version===mapViews[id]?.grid.map_version) return;
      try {
        const grid=await get(`/api/map?view=${id}`);
        if(!connected || state!==cohort) return;
        if(robot.map_session && grid.map_session && robot.map_session!==grid.map_session) return;
        const view=makeMapView(grid);
        paintMap(id,view,cohort);mapViews[id]=view;
      } catch(error) {mapError(id,error);}
    }) : [];
    await Promise.all(requests);
  }
  function renderParticipants(s) {
    const modern=participantState(s), ids=participantIds(s);
    $('swarmParticipants').hidden=!modern;
    $('swarmSourceMaps').hidden=!modern || !ids.length;
    $('carSwarmLocalMapCard').hidden=!modern || !ids.includes('car');
    $('acg720LocalMapCard').hidden=!modern || !ids.includes('acg720');
    if(!modern) {
      text('swarmParticipantsSummary','等待建图成员配置');
      text('swarmMappingStatus',connected?'等待建图成员状态':'协同服务不可用');
      return;
    }
    const offline=ids.filter(id=>s.robots[id].online!==true);
    text('swarmParticipantsSummary',`当前成员：${ids.map(id=>members[id].name).join(' / ') || '未选择'} · ${s.anchor_locked?'共同坐标基准':'临时局部参考'}：${members[s.anchor]?.name || s.anchor || '等待地图'}${offline.length?' · 离线：'+offline.map(id=>members[id].name).join(' / '):''}`);
    const mapping=s.mapping_status==='aligned'?'地图已对齐':
      s.mapping_status==='local_only'?'自身 SLAM 地图可用 · 等待对齐':'等待 SLAM 地图';
    const notes=Array.isArray(s.mapping_readiness)?s.mapping_readiness:[];
    text('swarmMappingStatus',`${mapping}${notes.length?'\n'+notes.join('\n'):''}`);
    for(const [id,member] of Object.entries(members)) {
      $(member.prefix).hidden=!ids.includes(id);
      if(!ids.includes(id)) continue;
      const r=s.robots[id], local=arrayPose(r.pose_local,r), common=pose(r.pose_common,r);
      text(member.prefix+'State',`${r.online===true?'接入在线':'当前离线'}${id===s.anchor?(s.anchor_locked?' · 共同坐标基准':' · 临时局部参考，未跨机对齐'):''}`);
      text(member.prefix+'Map',`SLAM 地图：${r.map_ready?'可用 · '+(r.map_frame || '坐标系未知')+' · '+mapAge(id):'等待地图'}${r.map_ready && r.online!==true?' · 保留最近地图':''}`);
      const readiness=value=>value===true?'已加载':value===false?'待配置':'状态未知';
      text(member.prefix+'Calibration',`相机内参：${r.camera_model_ready===false?'待标定':r.camera_intrinsics_preliminary?'初步参数 · 待验证':readiness(r.camera_model_ready)} · 相机安装变换：${readiness(r.camera_transform_ready)}`);
      const alignment=r.aligned===true?(id===s.anchor?'坐标基准':'AprilTag 已对齐'):
        r.alignment_status==='waiting_calibration'?'等待相机内参与安装变换':r.alignment_status || '等待初次对齐';
      text(member.prefix+'Alignment',`对齐：${alignment}`);
      const position=id==='car' && !currentCarPose(s)
        ? `${carLocalizationStatus(s)} · 历史轨迹已保留` : `局部 ${local} · 共同 ${r.aligned?common:'未对齐'}`;
      const reference=Number.isFinite(r.heading_offset_rad) && r.heading_offset_rad!==0?' · 朝向按机头':'';
      text(member.prefix+'Pose',`定位参考：${r.pose_reference || '后端局部参考'}${reference} · ${position}`);
      text(member.prefix+'Control',`手控：${r.manual_available?'已接入':'未接入'} · 自动导航：${r.auto_navigation_available?'就绪':id==='acg720'?'未接入':'未就绪'}`);
    }
  }
  function render() {
    const s=connected ? (state || {}) : {};
    publishPeerPose(s);
    renderParticipants(s);
    text('swarmConnection',connected?'协同服务在线':'协同服务不可用');
    text('swarmMission',connected?(s.exploring?'自动探索运行中：'+(s.reason || '等待任务状态'):'手动建图'):(connectionError || '等待协同任务状态'));
    text('swarmReadiness',!connected?'协同服务不可用，无法确认准备状态':
      Array.isArray(s.readiness) && s.readiness.length?`未就绪：\n${s.readiness.map(reason=>`• ${reason}`).join('\n')}`:
      Array.isArray(s.readiness)?'后端报告已满足探索前置条件':'等待协同准备状态');
    $('swarmReobserve').disabled=!connected || s.exploring!==false || reobservePending;
    $('swarmExploreStart').disabled=!connected || !participantState(s) || s.exploring!==false ||
      !Array.isArray(s.readiness) || s.readiness.length>0;
    $('swarmExploreStop').disabled=!connected || !s.exploring;
    const needsStopConfirmation=connected && Array.isArray(s.readiness) &&
      s.readiness.includes('等待上一条小车目标的取消结果');
    if(!needsStopConfirmation && waitingStop)$('swarmStopConfirmed').checked=false;
    waitingStop=needsStopConfirmation;
    $('swarmStopConfirm').hidden=!needsStopConfirmation;
    if(s.exploring || needsStopConfirmation) $('swarmAutoDetails').open=true;
    $('swarmConfirmStopped').disabled=!needsStopConfirmation || !!s.exploring ||
      !['idle','arrived','blocked'].includes(s.car?.mode) || !$('swarmStopConfirmed').checked;
  }
  $('swarmExploreStart').onclick=async()=>{try{await post('/api/explore/start');text('swarmResult','探索请求已提交，等待任务状态。');}catch(e){text('swarmResult',e.message);}};
  $('swarmExploreStop').onclick=async()=>{try{await post('/api/explore/stop');text('swarmResult','停止请求已提交，等待任务状态。');}catch(e){text('swarmResult',e.message);}};
  $('swarmReobserve').onclick=async()=>{
    if($('swarmReobserve').disabled)return;
    reobservePending=true;text('swarmReobserveResult','正在提交重新识别请求…');render();
    try{await post('/api/alignment/reobserve',{});
      text('swarmReobserveResult','重新识别请求已接收');
    }catch(e){text('swarmReobserveResult',`重新识别请求失败：${e.message}`);}
    finally{reobservePending=false;render();}
  };
  $('swarmStopConfirmed').onchange=render;
  $('swarmConfirmStopped').onclick=async()=>{
    if($('swarmConfirmStopped').disabled)return;
    try{await post('/api/explore/confirm-stopped',{confirmed:true});$('swarmStopConfirmed').checked=false;
      text('swarmResult','已提交现场停车确认，等待状态更新。');render();}
    catch(e){text('swarmResult',e.message);}
  };
  async function poll() {
    if(busy){
      if(Date.now()-lastStateAt>2500){
        connected=false;carPoseCurrent=false;sampleCarTrack(state);
        render();
        if(!connected){Object.values(members).forEach(member=>{$(member.canvas).hidden=true;});}
      }
      return;
    }
    busy=true;
    try {
      try {state=await get('/api/state');connected=true;lastStateAt=Date.now();carPoseCurrent=true;acceptMapSessions(state);}
      catch(error) {
        carPoseCurrent=false;connectionError=error.message || '协同服务不可用';
        if(Date.now()-lastStateAt>2500){connected=false;state=null;}
      }
      sampleCarTrack(state);render();
      if(connected && participantState()) paintCachedMaps();
      if(connected && Date.now()-lastMapAt>=2000){lastMapAt=Date.now();await refreshMaps();}
      if(!connected){Object.values(members).forEach(member=>{$(member.canvas).hidden=true;});}
    } finally {busy=false;}
  }
  $('swarmSourceMaps').addEventListener('toggle',()=>{lastMapAt=0;poll();});
  render();poll();setInterval(poll,1000);
})();
