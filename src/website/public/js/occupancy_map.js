/* 显示设备或 Jetson 提供的局部栅格；浏览器不重建占用地图。 */
(function (root, factory) {
  const codec = typeof module === 'object' && module.exports ? require('./grid_codec.js') : root.BX72GridCodec;
  const api = factory(codec);
  if (typeof module === 'object' && module.exports) module.exports = api;
  if (root) root.BX72OccupancyMap = api;
})(typeof globalThis === 'undefined' ? this : globalThis, function (codec) {
  'use strict';
  const PALETTE = [[214,222,229], [250,252,254], [37,52,70]];

  function decodeGrid(map) {
    if (!['fpga_grid_view_v1','jetson_grid_view_v1'].includes(map?.protocol) ||
        ![map.width,map.height].every(n => Number.isInteger(n) && n > 0 && n <= 1200) ||
        !Number.isFinite(map.resolution_m) || map.resolution_m <= 0) throw new Error('地图格式无效');
    return codec.decodeCells(map.cells_rle, map.width * map.height);
  }

  // map <- odom 与 FPGA grid <- odom 合成；所有输入均为米和弧度。
  function gridFromMap(point, gridFromOdom, mapFromOdom) {
    const a = gridFromOdom.yaw_offset_mrad / 1000, b = mapFromOdom.yaw;
    const c = Math.cos(b), s = Math.sin(b);
    const dx = point.x-mapFromOdom.x, dy = point.y-mapFromOdom.y;
    const ox = c*dx+s*dy-gridFromOdom.odom_anchor_x_mm/1000;
    const oy = -s*dx+c*dy-gridFromOdom.odom_anchor_y_mm/1000;
    return {x:gridFromOdom.map_anchor_x_mm/1000+Math.cos(a)*ox-Math.sin(a)*oy,
      y:gridFromOdom.map_anchor_y_mm/1000+Math.sin(a)*ox+Math.cos(a)*oy};
  }

  function createView({document:doc, window:win, onSelectGoal = () => {},
    onGlobalGoalInvalidated = () => {}, onMapChange = () => {}, onViewChange = () => {},
    WebSocketImpl = typeof WebSocket === 'undefined' ? null : WebSocket} = {}) {
    const canvas = doc.getElementById('radarCloud'), context = canvas?.getContext('2d');
    const videoCanvas = doc.getElementById('videoRouteCanvas');
    const videoContext = videoCanvas?.getContext('2d');
    const videoPreview = doc.getElementById('videoRoutePreview');
    const videoToggle = doc.getElementById('videoRouteToggle');
    const text = (id, value) => { const node = doc.getElementById(id); if (node) node.textContent = value; };
    const bind = (id, callback) => doc.getElementById(id)?.addEventListener('click', callback);
    const raster = doc.createElement('canvas'), rasterContext = raster.getContext('2d');
    const globalRaster = doc.createElement('canvas'), globalRasterContext = globalRaster.getContext('2d');
    let map = null, mapIdentity = null, socket = null, receivedAt = 0, receivedMonoAt = null, openedAt = 0, online = false, textureKey = '', autoFit = true;
    let mapEligible = false, validatedRoute = null, routeTimer = 0;
    let wantedUrl = null, reconnectTimer = 0, watchdogTimer = 0, retryMs = 250, lastFailure = null;
    let center = {x:0,y:0}, span = 16, dragging = null;
    let videoRouteVisible = true;
    let unavailableReason = '等待局部栅格';
    let globalMap = null, globalOnline = false, globalReceivedMonoAt = null;
    let globalTextureKey = '', globalReason = '等待 Jetson 全局地图', displayFrame = 'grid', globalOnly = false;
    let globalIdentity = null, navigation = null;

    const monotonicNow = () => typeof win.performance?.now === 'function' ? win.performance.now() : Date.now();
    function ageSinceReceive() { return receivedMonoAt !== null ? Math.max(0, monotonicNow() - receivedMonoAt) : Infinity; }
    function poseIsFresh(elapsed) {
      return !!(map?.pose_fresh === true && Number.isFinite(map.pose_age_ms) &&
        map.pose_age_ms >= 0 && map.pose_age_ms + elapsed <= 500);
    }
    function gridIsFresh(elapsed) {
      return !!(map?.grid_fresh === true && map.grid_ready === true && map.grid_gap === false &&
        Number.isFinite(map.grid_age_ms) && map.grid_age_ms >= 0 && map.grid_age_ms + elapsed <= 1500);
    }
    function isFresh() {
      const elapsed = ageSinceReceive();
      return !!(online && receivedAt && mapEligible && Number.isFinite(map?.snapshot_age_ms) &&
        map.snapshot_age_ms >= 0 && map.snapshot_age_ms + elapsed < 750 &&
        poseIsFresh(elapsed) && gridIsFresh(elapsed));
    }
    function globalTransformFresh() {
      const t = globalMap?.map_from_odom;
      const elapsed = globalReceivedMonoAt === null ? Infinity : Math.max(0,monotonicNow()-globalReceivedMonoAt);
      return !!(globalOnline && globalMap?.available && globalMap.status === 'building' && t &&
        [t.x,t.y,t.yaw,globalMap.map_from_odom_age_ms].every(Number.isFinite) &&
        globalMap.map_from_odom_age_ms >= 0 && globalMap.map_from_odom_age_ms+elapsed <= 750);
    }
    function canOverlay() {
      return !!(isFresh() && globalTransformFresh() && globalMap.domain === map.domain);
    }
    function currentFrame() {
      return globalOnly || (!isFresh() && globalMap?.available) ? 'global' : 'grid';
    }
    function getInfo() {
      return {fresh:isFresh(), domain:map?.domain || null, generation:map?.generation ?? null,
        yaw:Number.isFinite(map?.pose_odom?.yaw) ? map.pose_odom.yaw : null};
    }
    function getGlobalInfo() {
      return {fresh:globalTransformFresh(),
        domain:globalMap?.domain ?? null, generation:globalMap?.generation ?? null};
    }
    function gridCoordinatesAvailable() {
      return !!(map?.origin && [map.origin.x,map.origin.y].every(Number.isFinite) &&
        typeof map.domain === 'string' && map.domain && map.grid_from_odom);
    }
    function globalCoordinatesAvailable() {
      return !!(globalMap?.origin && typeof globalMap.domain === 'string' && globalMap.domain &&
        typeof globalMap.generation === 'string' && globalMap.generation);
    }
    function toOdom(point) {
      if (!gridCoordinatesAvailable() || !point || ![point.x,point.y].every(Number.isFinite)) return null;
      const t=map.grid_from_odom, a=t.yaw_offset_mrad/1000, c=Math.cos(a), s=Math.sin(a);
      const dx=point.x-t.map_anchor_x_mm/1000, dy=point.y-t.map_anchor_y_mm/1000;
      return {x:t.odom_anchor_x_mm/1000+c*dx+s*dy, y:t.odom_anchor_y_mm/1000-s*dx+c*dy};
    }
    function setNavigation(next) {
      navigation = next;
      draw();
    }

    function setGlobalMap(update) {
      const next = update?.map;
      globalOnline = update?.online === true;
      if (!globalOnline) {
        globalReason = update?.reason || '全局地图连接中断';
        armFreshnessTimer();
        draw(); onMapChange();
        return;
      }
      if (!next || next.available === false) {
        globalReason = next?.reason || '等待 Jetson 全局地图';
        globalMap = null; globalTextureKey = ''; globalReceivedMonoAt = monotonicNow();
        draw(); onMapChange();
        return;
      }
      const cells = update.cells;
      if (!(cells instanceof Uint8Array) || cells.length !== next.width*next.height ||
          !next.origin || !Number.isFinite(next.origin.x) || !Number.isFinite(next.origin.y) ||
          !Number.isFinite(next.resolution_m) || next.resolution_m <= 0) return;
      if (globalIdentity && (globalIdentity.domain !== next.domain || globalIdentity.generation !== next.generation))
        onGlobalGoalInvalidated();
      globalIdentity = {domain:next.domain,generation:next.generation};
      const key = JSON.stringify([next.domain,next.generation,next.revision,next.width,next.height,
        next.resolution_m,next.origin.x,next.origin.y]);
      if (key !== globalTextureKey) {
        globalRaster.width = next.width; globalRaster.height = next.height;
        const pixels = globalRasterContext.createImageData(next.width,next.height);
        for (let y=0; y<next.height; y++) for (let x=0; x<next.width; x++) {
          const color=PALETTE[cells[y*next.width+x]], p=((next.height-1-y)*next.width+x)*4;
          pixels.data[p]=color[0]; pixels.data[p+1]=color[1]; pixels.data[p+2]=color[2]; pixels.data[p+3]=255;
        }
        globalRasterContext.putImageData(pixels,0,0);
        if (globalMap && (globalMap.domain !== next.domain || globalMap.generation !== next.generation)) autoFit = true;
        globalTextureKey = key;
      }
      globalMap = next; globalReason = next.reason || '全局地图'; globalReceivedMonoAt = monotonicNow();
      if (autoFit) fit();
      armFreshnessTimer();
      draw(); onMapChange();
    }

    function routeLength(route) {
      return route.points.slice(1).reduce((length, point, index) =>
        length + Math.hypot(point.x - route.points[index].x, point.y - route.points[index].y), 0);
    }

    function drawRoute(ctx, route, project) {
      const points = route.points.map(point => project(point.x, point.y));
      // Coincident points are a position, not a forward arrow or an extended path.
      if (routeLength(route) > 1e-9) {
        ctx.strokeStyle = '#db7b13'; ctx.lineWidth = 4; ctx.setLineDash([7,4]);
        ctx.beginPath();
        points.forEach(([x,y], index) => { if (index) ctx.lineTo(x,y); else ctx.moveTo(x,y); });
        ctx.stroke(); ctx.setLineDash([]);
      }
      ctx.fillStyle = '#ffad43';
      for (const [x,y] of [points[0], points[points.length - 1]]) ctx.fillRect(x-3,y-3,6,6);
    }

    function drawVideoRoute(route) {
      if (!videoCanvas || !videoContext || videoPreview?.hidden) return;
      const rect = videoCanvas.getBoundingClientRect(), dpr = win.devicePixelRatio || 1;
      const w = rect.width, h = rect.height;
      if (w <= 16 || h <= 16) return;
      const pw = Math.round(w*dpr), ph = Math.round(h*dpr);
      if (videoCanvas.width !== pw || videoCanvas.height !== ph) {
        videoCanvas.width = pw; videoCanvas.height = ph;
      }
      videoContext.setTransform(dpr,0,0,dpr,0,0);
      videoContext.clearRect(0,0,w,h);
      if (!route) {
        text('videoRouteStatus', '暂无新鲜设备规划');
        if (videoCanvas.dataset) videoCanvas.dataset.state = 'empty';
        return;
      }

      const pose = map.pose, c = Math.cos(pose.yaw), s = Math.sin(pose.yaw);
      const relative = (x,y) => {
        const dx = x - pose.x, dy = y - pose.y;
        return {forward:dx*c + dy*s, left:-dx*s + dy*c};
      };
      const localPoints = route.points.map(point => relative(point.x,point.y));
      const side = Math.max(.4, ...localPoints.map(point => Math.abs(point.left)*1.15));
      const front = Math.max(.5, ...localPoints.map(point => point.forward*1.15));
      const back = Math.max(.3, ...localPoints.map(point => -point.forward*1.15));
      const scale = Math.min(Math.max(1,w-16)/(2*side), Math.max(1,h-16)/(front+back));
      const originX = w/2, originY = 8 + front*scale;
      const project = (x,y) => {
        const point = relative(x,y);
        return [originX - point.left*scale, originY - point.forward*scale];
      };

      // 车体与预览使用同一个米制比例；画面是俯视示意而非摄像头投影。
      videoContext.fillStyle = '#237f91'; videoContext.strokeStyle = '#cdeef4';
      videoContext.lineWidth = 1; videoContext.beginPath();
      [[.13,.125],[.13,-.125],[-.13,-.125],[-.13,.125]].forEach(([forward,left],index) => {
        const x=originX-left*scale, y=originY-forward*scale;
        if (index) videoContext.lineTo(x,y); else videoContext.moveTo(x,y);
      });
      videoContext.closePath(); videoContext.fill(); videoContext.stroke();
      videoContext.strokeStyle = '#e3f9ff'; videoContext.beginPath();
      videoContext.moveTo(originX,originY); videoContext.lineTo(originX,originY-.1*scale);
      videoContext.stroke();
      drawRoute(videoContext, route, project);

      const rawBar = Math.min(24, (w-16)/2)/scale, power = 10 ** Math.floor(Math.log10(rawBar));
      const bar = [1,2,5,10].map(value => value*power).find(value => value >= rawBar);
      const barWidth = bar*scale;
      videoContext.strokeStyle = '#dcebf0'; videoContext.lineWidth = 1.5;
      videoContext.beginPath(); videoContext.moveTo(8,h-5); videoContext.lineTo(8+barWidth,h-5); videoContext.stroke();
      videoContext.fillStyle = '#dcebf0'; videoContext.font = '10px system-ui';
      videoContext.fillText(`${Number(bar.toPrecision(2))} m`, 8, h-9);
      const length = routeLength(route);
      const first = localPoints[0], last = localPoints[localPoints.length - 1];
      const forward = last.forward - first.forward, left = last.left - first.left;
      const coincident = Math.hypot(forward,left) <= 1e-9;
      const distance = coincident ? '端点重合' : length < 1 ? `${Math.round(length*1000)} mm` : `${length.toFixed(2)} m`;
      let direction = '';
      if (!coincident) {
        const degrees = Math.round(Math.atan2(Math.abs(left),Math.abs(forward))*180/Math.PI);
        const longitudinal = forward >= 0 ? '前' : '后', lateral = left >= 0 ? '左' : '右';
        direction = degrees === 0 ? longitudinal : degrees === 90 ? lateral :
          `${longitudinal}偏${lateral}${degrees}°`;
      }
      const source = route.source === 'fpga' ? 'FPGA' : 'Jetson';
      text('videoRouteStatus', `${source} · ${distance}${direction ? ` · ${direction}` : ''}${route.guard_stop ? ' · 保护停车' : ''}`);
      if (videoCanvas.dataset) videoCanvas.dataset.state = 'fresh';
    }

    function fit() {
      const frame = currentFrame(), points = [];
      if (frame === 'grid' && map?.origin && map.bounds) {
        const b = map.bounds, r = map.resolution_m;
        points.push({x:map.origin.x+b.min_x*r,y:map.origin.y+b.min_y*r},
          {x:map.origin.x+(b.max_x+1)*r,y:map.origin.y+(b.max_y+1)*r});
      }
      if (globalMap?.origin && (frame === 'global' || canOverlay())) {
        const x=globalMap.origin.x, y=globalMap.origin.y;
        const right=x+globalMap.width*globalMap.resolution_m, top=y+globalMap.height*globalMap.resolution_m;
        for (const point of [{x,y},{x:right,y},{x,y:top},{x:right,y:top}])
          points.push(frame === 'global' ? point : gridFromMap(point,map.grid_from_odom,globalMap.map_from_odom));
      }
      if (!points.length) return;
      const xs=points.map(p=>p.x), ys=points.map(p=>p.y);
      const minX=Math.min(...xs), maxX=Math.max(...xs), minY=Math.min(...ys), maxY=Math.max(...ys);
      center={x:(minX+maxX)/2,y:(minY+maxY)/2};
      span=Math.max(4,Math.max(maxX-minX,maxY-minY)*1.18);
    }

    function armFreshnessTimer() {
      if (routeTimer) win.clearTimeout(routeTimer);
      routeTimer=0;
      if (!online || !map || !receivedAt) return;
      const elapsed=ageSinceReceive(), deadlines=[];
      for (const [value,limit] of [[map.snapshot_age_ms,750],[map.pose_age_ms,501],[map.grid_age_ms,1501]])
        if (Number.isFinite(value) && value + elapsed < limit) deadlines.push(limit-value-elapsed);
      if (validatedRoute && validatedRoute.age_ms+elapsed < 500) deadlines.push(500-validatedRoute.age_ms-elapsed);
      const globalElapsed=globalReceivedMonoAt === null ? Infinity : Math.max(0,monotonicNow()-globalReceivedMonoAt);
      if (globalOnline && Number.isFinite(globalMap?.map_from_odom_age_ms) &&
          globalMap.map_from_odom_age_ms+globalElapsed <= 750)
        deadlines.push(751-globalMap.map_from_odom_age_ms-globalElapsed);
      if (deadlines.length) routeTimer=win.setTimeout(() => {
        routeTimer=0; draw(); armFreshnessTimer();
      },Math.max(1,Math.ceil(Math.min(...deadlines))));
    }

    function receive(raw) {
      let next, nextCells;
      try {
        next = JSON.parse(raw);
        if (next.type !== 'mapping' || next.v !== 1 ||
            !((next.protocol === 'fpga_grid_view_v1' && next.source === 'fpga') ||
              (next.protocol === 'jetson_grid_view_v1' && next.source === 'jetson'))) return;
        if (next.available === false) {
          unavailableReason=typeof next.reason === 'string' ? next.reason : '局部栅格不可用';
          map=null; validatedRoute=null; mapEligible=false; textureKey=''; receivedAt=Date.now(); receivedMonoAt=monotonicNow();
          if (routeTimer) win.clearTimeout(routeTimer); routeTimer=0;
          draw(); onMapChange(); return true;
        }
        if (next.available !== true || next.width !== 64 || next.height !== 64 || next.resolution_m !== .05 ||
            !next.grid_from_odom || !['map_anchor_x_mm','map_anchor_y_mm','odom_anchor_x_mm',
              'odom_anchor_y_mm','yaw_offset_mrad'].every(key => Number.isInteger(next.grid_from_odom[key])))
          throw new Error('局部栅格合同无效');
        nextCells = decodeGrid(next);
      } catch (_) { text('mapStatus', '地图消息无效，保留上一张图'); return false; }
      const key = `${next.domain}:${next.generation}:${next.revision}`;
      if (key !== textureKey) {
        raster.width = next.width; raster.height = next.height;
        const pixels = rasterContext.createImageData(next.width, next.height);
        for (let y = 0; y < next.height; y++) {
          for (let x = 0; x < next.width; x++) {
            const color = PALETTE[nextCells[y * next.width + x]];
            const p = ((next.height - 1 - y) * next.width + x) * 4;
            pixels.data[p] = color[0]; pixels.data[p+1] = color[1];
            pixels.data[p+2] = color[2]; pixels.data[p+3] = 255;
          }
        }
        rasterContext.putImageData(pixels, 0, 0);
        textureKey = key;
      }
      if (mapIdentity && (next.generation !== mapIdentity.generation || next.domain !== mapIdentity.domain)) {
        autoFit = true;
      }
      mapIdentity = {generation:next.generation,domain:next.domain};
      mapEligible = !!(next.status === 'building' && next.pose_fresh && next.origin && next.domain &&
        [next.pose?.x,next.pose?.y,next.pose?.yaw].every(Number.isFinite));
      const candidate = next.planned_route;
      validatedRoute = candidate && ['fpga','jetson'].includes(candidate.source) &&
        candidate.frame_id === (next.source === 'jetson' ? 'jetson_grid' : 'fpga_grid') && candidate.domain === next.domain &&
        Number.isFinite(candidate.age_ms) && candidate.age_ms >= 0 &&
        Array.isArray(candidate.points) && candidate.points.length >= 2 && candidate.points.length <= 2000 &&
        candidate.points.every(p => Number.isFinite(p?.x) && Number.isFinite(p?.y)) ? candidate : null;
      next.planned_history = Array.isArray(next.planned_history) && next.planned_history.length <= 300
        ? next.planned_history.filter(segment => Number.isInteger(segment?.task_id) &&
            Array.isArray(segment.points) && segment.points.length === 2 &&
            segment.points.every(point => Number.isFinite(point?.x) && Number.isFinite(point?.y))) : [];
      if (!Number.isInteger(next.active_goal?.task_id) ||
          !Number.isFinite(next.active_goal?.x) || !Number.isFinite(next.active_goal?.y)) next.active_goal=null;
      map = next; receivedAt = Date.now(); receivedMonoAt = monotonicNow();
      lastFailure = null;
      retryMs = 250;
      armWatchdog();
      armFreshnessTimer();
      if (autoFit) fit();
      if (canvas?.dataset) {
        canvas.dataset.generation = String(map.generation);
        canvas.dataset.revision = String(map.revision);
        canvas.dataset.occupied = String(map.counts.occupied);
        canvas.dataset.free = String(map.counts.free);
        canvas.dataset.snapshot = String(map.revision);
      }
      draw();
      onMapChange();
      return true;
    }

    function draw() {
      if (!canvas || !context) return;
      const frame=currentFrame(), overlay=!globalOnly && canOverlay();
      if (frame !== displayFrame) { displayFrame=frame; autoFit=true; fit(); }
      const rect = canvas.getBoundingClientRect(), dpr = win.devicePixelRatio || 1;
      const w = Math.max(1, rect.width), h = Math.max(1, rect.height);
      const pw = Math.round(w * dpr), ph = Math.round(h * dpr);
      if (canvas.width !== pw || canvas.height !== ph) { canvas.width = pw; canvas.height = ph; }
      context.setTransform(dpr,0,0,dpr,0,0);
      context.fillStyle = '#d6dee5'; context.fillRect(0,0,w,h);
      const scale = Math.min(w,h) / span;
      const screen = (x,y) => [w/2 + (x - center.x)*scale, h/2 - (y - center.y)*scale];
      const elapsed = ageSinceReceive();
      const fresh = isFresh();
      const route = fresh && validatedRoute && validatedRoute.age_ms + Math.max(0, elapsed) < 500 ? validatedRoute : null;
      const status = !online ? (lastFailure === 'stalled' ? '地图数据停更，正在重连 · 历史地图' :
        wantedUrl ? '地图连接已断开，正在重连 · 历史地图' : '地图连接已断开 · 历史地图') :
        !receivedAt ? (map ? '地图已连接，等待新数据 · 历史地图' : '等待地图') :
        !map ? unavailableReason :
        elapsed >= 1500 ? '地图数据停更 · 历史地图' :
        !poseIsFresh(elapsed) ? '位姿无效或已过期 · 历史地图' :
        map.snapshot_age_ms + elapsed >= 750 ? '局部栅格快照已过期 · 历史地图' :
        !gridIsFresh(elapsed) ? `${map.reason} · 历史地图` : map.reason;
      text('mapStatus', status);
      const statusNode = doc.getElementById('mapStatus');
      if (statusNode?.dataset) statusNode.dataset.state = fresh ? 'fresh' : 'stale';
      let globalStatus=globalReason;
      if (!globalOnline && globalMap) globalStatus += frame === 'global' ? ' · 显示历史地图' : ' · 未叠加';
      else if (globalMap?.available && frame === 'grid' && fresh && !overlay) {
        globalStatus=globalStatus.replace('显示历史地图','历史地图');
        globalStatus += globalMap.domain !== map.domain ? ' · 坐标域不一致，未叠加' : ' · 等待新鲜坐标变换，未叠加';
      } else if (overlay) globalStatus += ' · 已叠加局部栅格';
      text('globalMapStatus',globalStatus);
      const globalStatusNode=doc.getElementById('globalMapStatus');
      if (globalStatusNode?.dataset) globalStatusNode.dataset.state=
        globalOnline && globalMap?.status === 'building' && (frame === 'global' || overlay) ? 'fresh' : 'stale';
      const layerButton=doc.getElementById('mapLayerToggle');
      if (layerButton) {
        layerButton.disabled=!globalOnly && !globalMap?.available;
        layerButton.textContent=globalOnly ? '叠加局部' : '仅看全局';
        layerButton.setAttribute?.('aria-pressed',String(globalOnly));
      }
      const localLegend=doc.getElementById('radarLegend'), globalLegend=doc.getElementById('globalMapLegend');
      if (localLegend) localLegend.hidden=frame === 'global';
      if (globalLegend) globalLegend.hidden=frame !== 'global';
      const globalCounts=globalMap?.counts;
      if (globalMap?.available && [globalCounts?.free,globalCounts?.occupied].every(Number.isFinite) &&
          globalCounts.free >= 0 && globalCounts.occupied >= 0) {
        const area=(globalCounts.free+globalCounts.occupied)*globalMap.resolution_m**2;
        const age=Number.isFinite(globalMap.map_age_ms) && globalMap.map_age_ms >= 0 && globalReceivedMonoAt !== null
          ? ` · 更新 ${((globalMap.map_age_ms+Math.max(0,monotonicNow()-globalReceivedMonoAt))/1000).toFixed(1)} 秒前` : '';
        text('globalMapStats',`全局已知 ${area.toFixed(1)} m² · 障碍 ${globalCounts.occupied} 格${age}`);
      } else text('globalMapStats','等待全局地图');
      if (canvas.dataset) {
        canvas.dataset.status = fresh ? 'building' : 'paused';
        canvas.dataset.frame=frame; canvas.dataset.globalOverlay=overlay ? 'aligned' : 'unavailable';
      }
      if (globalMap?.origin && (frame === 'global' || overlay)) {
        const origin=frame === 'global' ? globalMap.origin : gridFromMap(globalMap.origin,map.grid_from_odom,globalMap.map_from_odom);
        const angle=frame === 'global' ? 0 : map.grid_from_odom.yaw_offset_mrad/1000-globalMap.map_from_odom.yaw;
        const [x,y]=screen(origin.x,origin.y);
        context.imageSmoothingEnabled=false;
        context.save(); context.translate(x,y); context.rotate(-angle);
        context.drawImage(globalRaster,0,-globalMap.height*globalMap.resolution_m*scale,
          globalMap.width*globalMap.resolution_m*scale,globalMap.height*globalMap.resolution_m*scale);
        context.restore();
      }
      if (frame === 'grid' && map?.origin) {
        const [left, top] = screen(map.origin.x, map.origin.y + map.height * map.resolution_m);
        context.imageSmoothingEnabled = false;
        context.drawImage(raster, left, top, map.width * map.resolution_m * scale,
          map.height * map.resolution_m * scale);
        if (fresh) {
          const width=map.width*map.resolution_m*scale, height=map.height*map.resolution_m*scale;
          context.strokeStyle='#087e99'; context.lineWidth=1.5; context.beginPath();
          context.moveTo(left,top); context.lineTo(left+width,top); context.lineTo(left+width,top+height);
          context.lineTo(left,top+height); context.closePath(); context.stroke();
        }
        context.lineWidth = 1; context.strokeStyle = 'rgba(104,125,144,.14)';
        const step = span > 24 ? 5 : span > 12 ? 2 : 1;
        context.beginPath();
        for (let x = Math.floor((center.x - w/scale/2)/step)*step; x < center.x + w/scale/2; x += step) {
          const [px] = screen(x,0); context.moveTo(px,0); context.lineTo(px,h);
        }
        for (let y = Math.floor((center.y - h/scale/2)/step)*step; y < center.y + h/scale/2; y += step) {
          const [,py] = screen(0,y); context.moveTo(0,py); context.lineTo(w,py);
        }
        context.stroke();
        const [ox,oy] = screen(0,0);
        context.strokeStyle = '#8a9bad'; context.beginPath();
        context.moveTo(ox-5,oy); context.lineTo(ox+5,oy);
        context.moveTo(ox,oy-5); context.lineTo(ox,oy+5); context.stroke();
        if (map.planned_history.length) {
          context.strokeStyle = 'rgba(219,123,19,.45)'; context.lineWidth = 2; context.beginPath();
          for (const segment of map.planned_history) {
            const [x0,y0]=screen(segment.points[0].x,segment.points[0].y);
            const [x1,y1]=screen(segment.points[1].x,segment.points[1].y);
            context.moveTo(x0,y0); context.lineTo(x1,y1);
          }
          context.stroke();
        }
        context.strokeStyle = '#0aa5c9'; context.lineWidth = 2; context.beginPath();
        (map.trajectory || []).forEach((p,i) => {
          const [x,y] = screen(p.x,p.y); if (!i) context.moveTo(x,y); else context.lineTo(x,y);
        });
        context.stroke();
        if (fresh && map.active_goal) {
          const [gx,gy]=screen(map.active_goal.x,map.active_goal.y);
          context.strokeStyle = '#aa3041'; context.fillStyle = '#fff'; context.lineWidth = 2;
          context.beginPath(); context.arc(gx,gy,7,0,Math.PI*2); context.fill(); context.stroke();
          context.fillStyle = '#8e2533'; context.font = 'bold 11px sans-serif';
          context.fillText(`T${map.active_goal.task_id}`,gx+9,gy-7);
        }
        if (map.pose) {
          const [x,y] = screen(map.pose.x,map.pose.y);
          const yaw = map.pose.yaw;
          // 车体按米制比例绘制；231 mm为grid-v1基础保护圆，扫掠占用检查另加10 mm。
          if (['nav_grid_v1','jetson_nav_v1'].includes(map.navigation_protocol)) {
            context.strokeStyle = '#087e99'; context.lineWidth = 1.5;
            context.beginPath();context.arc(x,y,0.231*scale,0,Math.PI*2);context.stroke();
          }
          context.fillStyle = fresh && map.pose_fresh ? '#087e99' : '#7b8793';
          context.strokeStyle = '#fff'; context.lineWidth = 1.5; context.beginPath();
          [[.13,.125],[.13,-.125],[-.13,-.125],[-.13,.125]].forEach(([forward,left],index) => {
            const px=x+(forward*Math.cos(yaw)-left*Math.sin(yaw))*scale;
            const py=y-(forward*Math.sin(yaw)+left*Math.cos(yaw))*scale;
            if (!index) context.moveTo(px,py); else context.lineTo(px,py);
          });
          context.closePath(); context.fill(); context.stroke();
          context.strokeStyle = '#fff'; context.beginPath();context.moveTo(x,y);
          context.lineTo(x+Math.cos(yaw)*Math.max(5,.12*scale),y-Math.sin(yaw)*Math.max(5,.12*scale));
          context.stroke();
        }
        if (route) drawRoute(context, route, screen);
        const area = (map.counts.free + map.counts.occupied) * map.resolution_m ** 2;
        text('radarStats', `局部已知 ${area.toFixed(1)} m² · 障碍 ${map.counts.occupied} 格 · 扫描 ${map.scan_generation ?? '—'}`);
        text('mapPose', map.pose ? `grid x ${map.pose.x.toFixed(3)} m · y ${map.pose.y.toFixed(3)} m${fresh ? '' : ' · 历史'}` : '等待有效位姿');
        text('trackCount', `${map.trajectory.length} 个融合估计轨迹点`);
        text('mapDiagnostics', `${map.source === 'jetson' ? 'Jetson' : 'FPGA'} 局部栅格 · 64×64 · 50 mm/格 · ` +
          `观测${map.grid_fresh ? '新鲜' : '过期'} · 完整圈${map.grid_ready ? '就绪' : '未就绪'} · ` +
          `缺口${map.grid_gap ? '有' : '无'}`);
      } else if (frame === 'global' && globalMap?.origin) {
        const pose=globalOnline && globalMap.status === 'building' ? globalMap.pose : null;
        if (pose && [pose.x,pose.y,pose.yaw].every(Number.isFinite)) {
          const [x,y]=screen(pose.x,pose.y), yaw=pose.yaw;
          context.fillStyle='#087e99'; context.strokeStyle='#fff'; context.lineWidth=1.5;
          context.beginPath();
          [[.2,0],[-.14,.1],[-.14,-.1]].forEach(([forward,left],index)=>{
            const px=x+(forward*Math.cos(yaw)-left*Math.sin(yaw))*scale;
            const py=y-(forward*Math.sin(yaw)+left*Math.cos(yaw))*scale;
            if (index) context.lineTo(px,py); else context.moveTo(px,py);
          });
          context.closePath(); context.fill(); context.stroke();
        }
        text('radarStats', globalOnly && fresh ? '局部图已隐藏' : '局部栅格不可用');
        text('mapPose', pose ? `map x ${pose.x.toFixed(3)} m · y ${pose.y.toFixed(3)} m` : '全局位姿不可用');
        text('trackCount',globalOnly && fresh ? '局部轨迹已隐藏' : '局部轨迹不可用');
        text('mapDiagnostics',globalOnly && fresh ? '仅显示全局地图；点击添加全局航点' :
          '局部栅格不可用，当前显示全局地图');
      } else {
        context.fillStyle = '#536779'; context.font = '13px system-ui'; context.textAlign = 'center';
        context.fillText(globalOnly ? '等待全局地图' : '等待局部栅格', w/2, h/2); context.textAlign = 'start';
        text('radarStats',globalOnly && fresh ? '局部图已隐藏' : '局部栅格不可用');
        text('mapPose','等待有效位姿');
        text('trackCount','等待地图轨迹');
        text('mapDiagnostics',globalOnly ? '全局地图不可用；航点选择已停用' :
          '等待同一板卡和坐标域内的完整局部栅格快照');
      }
      const navigationVisible = navigation?.frame === 'map'
        ? globalMap && navigation.domain === globalMap.domain && navigation.generation === globalMap.generation &&
          (frame === 'global' || overlay)
        : navigation?.frame === 'odom' && (frame === 'grid'
          ? gridCoordinatesAvailable() && navigation.domain === map.domain
          : globalTransformFresh() && navigation.domain === globalMap.domain);
      if (navigationVisible) {
        const project = point => {
          let p = point;
          if (navigation.frame === 'map' && frame === 'grid')
            p = gridFromMap(point,map.grid_from_odom,globalMap.map_from_odom);
          else if (navigation.frame === 'odom' && frame === 'grid') {
            const t=map.grid_from_odom, a=t.yaw_offset_mrad/1000, c=Math.cos(a), s=Math.sin(a);
            const dx=point.x-t.odom_anchor_x_mm/1000, dy=point.y-t.odom_anchor_y_mm/1000;
            p={x:t.map_anchor_x_mm/1000+c*dx-s*dy,y:t.map_anchor_y_mm/1000+s*dx+c*dy};
          } else if (navigation.frame === 'odom') {
            const t=globalMap.map_from_odom, c=Math.cos(t.yaw), s=Math.sin(t.yaw);
            p={x:t.x+c*point.x-s*point.y,y:t.y+s*point.x+c*point.y};
          }
          return screen(p.x,p.y);
        };
        if (navigation.route?.length) {
          context.strokeStyle='#7651b4'; context.lineWidth=3; context.setLineDash([8,4]); context.beginPath();
          navigation.route.forEach((point,index) => {
            const [x,y]=project(point); if (index) context.lineTo(x,y); else context.moveTo(x,y);
          });
          context.stroke(); context.setLineDash([]);
        }
        (navigation.goals || []).forEach((point,index) => {
          const [x,y]=project(point);
          context.fillStyle='#7651b4'; context.strokeStyle='#fff'; context.lineWidth=2;
          context.beginPath(); context.arc(x,y,11,0,Math.PI*2); context.fill(); context.stroke();
          context.fillStyle='#fff'; context.font='bold 12px sans-serif';
          context.textAlign='center'; context.textBaseline='middle';
          context.fillText(String(index+1),x,y);
          context.textAlign='start'; context.textBaseline='alphabetic';
        });
      }
      const bar = scale * (span > 24 ? 5 : span > 12 ? 2 : 1);
      context.strokeStyle = '#526678'; context.lineWidth = 2; context.beginPath();
      context.moveTo(16,h-22); context.lineTo(16+bar,h-22); context.stroke();
      context.fillStyle = '#344a5e'; context.font = '11px system-ui';
      context.fillText(`${span > 24 ? 5 : span > 12 ? 2 : 1} m`, 16,h-29);
      context.fillText(frame === 'global' ? 'SLAM map · +X →  +Y ↑' : `${map?.source === 'jetson' ? 'Jetson' : 'FPGA'} grid · +X →  +Y ↑`, Math.max(16,w-150),h-12);
      const historyIds=map?.planned_history?.length ? [...new Set(map.planned_history.map(segment=>segment.task_id))] : [];
      const historyNote=historyIds.length ? ` · 历史规划短段 ${map.planned_history.length} 条（task ${historyIds.join(', ')}）` : '';
      text('mapRouteStatus', frame !== 'grid' ? '' : route
        ? `${route.source === 'fpga' ? (route.grid_direction_preview ? 'FPGA 局部方向预览（两点连线，非完整曲线）' : 'FPGA 局部规划段') : (route.grid_direction_preview ? 'Jetson 局部方向预览（两点连线，非完整曲线）' : 'Jetson 局部规划段')}${Number.isInteger(route.task_id) ? ` · task ${route.task_id}` : ''}${route.guard_stop === true ? ' · 保护停车' : ''}`
        + historyNote
        : map?.planned_route ? `设备规划段不可用、过期或坐标域不匹配${historyNote}` : historyNote.replace(/^ · /,''));
      drawVideoRoute(route);
    }

    function clearWatchdog() {
      if (watchdogTimer) win.clearTimeout(watchdogTimer);
      watchdogTimer = 0;
    }
    function clearReconnect() {
      if (reconnectTimer) win.clearTimeout(reconnectTimer);
      reconnectTimer = 0;
    }
    function scheduleReconnect() {
      if (!wantedUrl || reconnectTimer || socket) return;
      const delay = retryMs;
      retryMs = Math.min(retryMs * 2, 4000);
      reconnectTimer = win.setTimeout(() => { reconnectTimer = 0; openSocket(); }, delay);
    }
    function retireSocket(current, reason) {
      if (socket !== current) return;
      socket = null;
      online = false;
      receivedAt = 0;
      receivedMonoAt = null;
      openedAt = 0;
      textureKey = '';
      lastFailure = reason;
      clearWatchdog();
      if (current.readyState < 2) { try { current.close(); } catch (_) {} }
      draw();
      onMapChange();
      scheduleReconnect();
    }
    function armWatchdog() {
      clearWatchdog();
      if (!socket || !online) return;
      const current = socket;
      watchdogTimer = win.setTimeout(() => {
        watchdogTimer = 0;
        if (socket !== current || !online) return;
        if (Date.now() - (receivedAt || openedAt) >= 1500) retireSocket(current, 'stalled');
        else armWatchdog();
      }, 1500);
    }
    function openSocket() {
      if (!wantedUrl || !WebSocketImpl || socket) return;
      let current;
      try { current = new WebSocketImpl(wantedUrl); }
      catch (_) { lastFailure = 'connection'; draw(); onMapChange(); scheduleReconnect(); return; }
      socket = current;
      watchdogTimer = win.setTimeout(() => {
        watchdogTimer = 0;
        if (socket === current && !online) retireSocket(current, 'connection');
      }, 4000);
      current.onopen = () => {
        if (socket !== current) return;
        online = true;
        receivedAt = 0;
        receivedMonoAt = null;
        openedAt = Date.now();
        armWatchdog();
        draw(); onMapChange();
      };
      current.onmessage = event => {
        if (socket !== current || !online || typeof event.data !== 'string') return;
        // 长时间停更后的排队旧帧不能重新证明新鲜；重连后等待新快照。
        if (Date.now() - (receivedAt || openedAt) >= 1500) {
          retireSocket(current, 'stalled');
          return;
        }
        receive(event.data);
      };
      current.onclose = () => retireSocket(current, 'connection');
      current.onerror = () => retireSocket(current, 'connection');
    }
    function connect(url) {
      if (!WebSocketImpl) return;
      if (wantedUrl !== url) {
        const previous = socket;
        wantedUrl = url;
        socket = null;
        online = false;
        receivedAt = 0;
        receivedMonoAt = null;
        openedAt = 0;
        lastFailure = null;
        retryMs = 250;
        clearReconnect(); clearWatchdog();
        if (previous && previous.readyState < 2) { try { previous.close(); } catch (_) {} }
        draw(); onMapChange();
      }
      if (socket) return;
      clearReconnect();
      openSocket();
    }
    function disconnect() {
      const previous = socket;
      wantedUrl = null;
      socket = null;
      online = false;
      receivedAt = 0;
      receivedMonoAt = null;
      openedAt = 0;
      lastFailure = null;
      clearReconnect(); clearWatchdog();
      if (previous && previous.readyState < 2) { try { previous.close(); } catch (_) {} }
      draw(); onMapChange();
    }

    bind('mapZoomIn', () => { autoFit = false; span = Math.max(2, span / 1.4); draw(); });
    bind('mapZoomOut', () => { autoFit = false; span = Math.min(80, span * 1.4); draw(); });
    bind('mapFit', () => { autoFit = true; fit(); draw(); });
    bind('mapLayerToggle', () => {
      if (!globalOnly && !globalMap?.available) return;
      globalOnly = !globalOnly; autoFit = true; fit(); draw(); onViewChange();
    });
    bind('videoRouteToggle', () => {
      videoRouteVisible = !videoRouteVisible;
      if (videoPreview) videoPreview.hidden = !videoRouteVisible;
      if (videoToggle) {
        videoToggle.setAttribute('aria-pressed', String(videoRouteVisible));
        videoToggle.textContent = videoRouteVisible ? '隐藏局部规划' : '显示局部规划';
      }
      if (videoRouteVisible) draw();
    });
    canvas?.addEventListener('pointerdown', event => {
      if (event.button !== undefined && event.button !== 0) return;
      dragging = {x:event.clientX,y:event.clientY,center:{...center}, moved:false, pointerId:event.pointerId};
      canvas.setPointerCapture?.(event.pointerId);
    });
    canvas?.addEventListener('pointermove', event => {
      if (!dragging || event.pointerId !== dragging.pointerId) return;
      if (Math.hypot(event.clientX-dragging.x,event.clientY-dragging.y) < 8 && !dragging.moved) return;
      dragging.moved = true;
      const r = canvas.getBoundingClientRect(), scale = Math.min(r.width,r.height) / span;
      center = {x:dragging.center.x - (event.clientX-dragging.x)/scale,
        y:dragging.center.y + (event.clientY-dragging.y)/scale};
      autoFit = false; draw();
    });
    canvas?.addEventListener('pointerup', event => {
      if (!dragging || event.pointerId !== dragging.pointerId) return;
      const moved = dragging.moved || Math.hypot(event.clientX-dragging.x,event.clientY-dragging.y) >= 8;
      dragging = null;
      if (moved) return;
      const r = canvas.getBoundingClientRect(), scale = Math.min(r.width,r.height) / span;
      const x = center.x + (event.clientX - r.left - r.width/2) / scale;
      const y = center.y - (event.clientY - r.top - r.height/2) / scale;
      let odomPoint, mapPoint, domain;
      if (currentFrame() === 'grid') {
        if (!gridCoordinatesAvailable()) { text('mapGoalHint','等待带坐标域和坐标变换的局部地图。'); return; }
        domain = map.domain;
        odomPoint = toOdom({x,y});
        if (canOverlay() && globalCoordinatesAvailable()) {
          const t=globalMap.map_from_odom, c=Math.cos(t.yaw), s=Math.sin(t.yaw);
          mapPoint={x:t.x+c*odomPoint.x-s*odomPoint.y,y:t.y+s*odomPoint.x+c*odomPoint.y};
        }
      } else {
        if (!globalCoordinatesAvailable()) { text('mapGoalHint','等待带坐标域和代次的全局地图。'); return; }
        domain = globalMap.domain;
        mapPoint = {x,y};
        // map 坐标可直接使用；只有跨到 odom 草稿时才需要新鲜变换。
        if (globalTransformFresh()) {
          const t=globalMap.map_from_odom, c=Math.cos(t.yaw), s=Math.sin(t.yaw), dx=x-t.x, dy=y-t.y;
          odomPoint={x:c*dx+s*dy,y:-s*dx+c*dy};
        }
      }
      onSelectGoal({domain,odom:odomPoint,map:mapPoint && {...mapPoint,generation:globalMap.generation}});
    });
    for (const event of ['pointercancel','lostpointercapture'])
      canvas?.addEventListener(event, () => { dragging = null; });
    draw();
    return {connect, disconnect, draw, receive, setGlobalMap, setNavigation, getInfo, getGlobalInfo,
      isGlobalOnly:() => globalOnly};
  }
  return {decodeGrid, gridFromMap, createView};
});
