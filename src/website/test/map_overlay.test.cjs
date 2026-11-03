'use strict';

const assert = require('node:assert/strict');
const test = require('node:test');
const {createView,gridFromMap} = require('../public/js/occupancy_map.js');

test('map to FPGA grid transform combines both rotations, translations and anchors', () => {
  const grid={map_anchor_x_mm:2000,map_anchor_y_mm:3000,odom_anchor_x_mm:1000,
    odom_anchor_y_mm:-1000,yaw_offset_mrad:Math.round(Math.PI/2*1000)};
  const map={x:10,y:20,yaw:Math.PI/2};
  // odom (2,-1) maps to map (11,22), then to grid (2,4).
  const point=gridFromMap({x:11,y:22},grid,map);
  assert.ok(Math.abs(point.x-2)<.001);
  assert.ok(Math.abs(point.y-4)<.001);
  const identity={map_anchor_x_mm:0,map_anchor_y_mm:0,odom_anchor_x_mm:0,
    odom_anchor_y_mm:0,yaw_offset_mrad:0};
  assert.deepEqual(gridFromMap({x:3,y:5},identity,{x:1,y:2,yaw:0}),{x:2,y:3});
});

test('global raster sits below local grid and both map frames can select goals', () => {
  let now=1000, timerId=0;
  const timers=new Map(), nodes=new Map(), handlers={}, draws=[], rasters=[], labels=[], strokes=[];
  const context={setTransform(){},fillRect(){},drawImage(image){draws.push(image);},
    save(){},restore(){},translate(){},rotate(){},setLineDash(){},beginPath(){},
    moveTo(){},lineTo(){},closePath(){},stroke(){strokes.push(this.strokeStyle);},fill(){},
    fillText(text,x,y){labels.push({text,x,y,color:this.fillStyle});},arc(){}};
  const canvas={dataset:{},getContext:()=>context,
    getBoundingClientRect:()=>({left:0,top:0,width:600,height:400}),
    addEventListener:(event,fn)=>{handlers[event]=fn;}};
  nodes.set('radarCloud',canvas);
  const layerButton={disabled:false,addEventListener:(event,fn)=>{handlers[`layer_${event}`]=fn;},setAttribute(){}};
  nodes.set('mapLayerToggle',layerButton);
  for (const id of ['globalMapStatus','globalMapStats','mapStatus','mapGoalHint','radarStats',
    'radarLegend','globalMapLegend']) nodes.set(id,{textContent:'',dataset:{}});
  const document={getElementById:id=>nodes.get(id),createElement:()=>{
    const ctx={createImageData:(w,h)=>({data:new Uint8ClampedArray(w*h*4)}),
      putImageData(image){this.image=image;}};
    const raster={getContext:()=>ctx,ctx}; rasters.push(raster); return raster;
  }};
  const window={devicePixelRatio:1,performance:{now:()=>now},
    setTimeout:(fn,delay)=>{timers.set(++timerId,{fn,delay});return timerId;},
    clearTimeout:id=>timers.delete(id)};
  class Socket {constructor(){Socket.current=this;this.readyState=0;}}
  const selected=[];
  const view=createView({document,window,WebSocketImpl:Socket,onSelectGoal:goal=>selected.push(goal)});
  view.connect('ws://local/map'); Socket.current.readyState=1; Socket.current.onopen();
  const local={v:1,type:'mapping',protocol:'fpga_grid_view_v1',source:'fpga',available:true,
    generation:1,revision:1,domain:'7:3',status:'building',reason:'FPGA 局部栅格',
    width:64,height:64,resolution_m:.05,origin:{x:-1.6,y:-1.6},
    bounds:{min_x:0,max_x:63,min_y:0,max_y:63},cells_rle:[0,4096],counts:{free:0,occupied:0},
    pose:{x:0,y:0,yaw:0},pose_odom:{x:0,y:0,yaw:0},trajectory:[],
    pose_fresh:true,pose_age_ms:0,snapshot_age_ms:0,grid_age_ms:0,grid_fresh:true,grid_ready:true,grid_gap:false,
    grid_from_odom:{map_anchor_x_mm:0,map_anchor_y_mm:0,odom_anchor_x_mm:0,odom_anchor_y_mm:0,yaw_offset_mrad:0}};
  view.receive(JSON.stringify(local));
  const global={available:true,status:'building',reason:'SLAM 建图中',domain:'7:3',generation:'g',revision:1,
    width:20,height:20,resolution_m:.5,origin:{x:-5,y:-5},counts:{free:400,occupied:0},map_age_ms:100,
    map_from_odom:{x:0,y:0,yaw:0},map_from_odom_age_ms:0};
  draws.length=0;
  view.setGlobalMap({map:global,cells:new Uint8Array(400).fill(1),online:true});
  assert.equal(canvas.dataset.globalOverlay,'aligned');
  assert.equal(canvas.dataset.frame,'grid');
  assert.match(nodes.get('globalMapStats').textContent,/全局已知 100\.0 m² · 障碍 0 格 · 更新 0\.1 秒前/);
  assert.deepEqual(draws.slice(0,2),[rasters[1],rasters[0]],'global is below local');
  assert.deepEqual([...rasters[0].ctx.image.data.slice(0,4)],[214,222,229,255],
    'unknown local cells remain opaque over global free space');
  const click=(x,y)=>{const e={button:0,pointerId:1,clientX:x,clientY:y}; handlers.pointerdown(e); handlers.pointerup(e);};
  click(300,200); assert.equal(selected.length,1);
  assert.ok(selected[0].map && selected[0].odom,'inside the local grid both coordinate frames are available');
  click(450,200); assert.equal(selected.length,2);
  assert.ok(selected[1].map.x > 1.6,'outside the local grid the same callback returns a map point');
  assert.equal(selected[1].domain,'7:3'); assert.equal(selected[1].map.generation,'g');
  const transformed={...global,map_from_odom:{x:.5,y:-.3,yaw:.35}};
  view.setGlobalMap({map:transformed,cells:new Uint8Array(400),online:true});
  const corners=[{x:-5,y:-5},{x:5,y:-5},{x:-5,y:5},{x:5,y:5}]
    .map(point=>gridFromMap(point,local.grid_from_odom,transformed.map_from_odom));
  const xs=[-1.6,1.6,...corners.map(p=>p.x)], ys=[-1.6,1.6,...corners.map(p=>p.y)];
  const minX=Math.min(...xs), maxX=Math.max(...xs), minY=Math.min(...ys), maxY=Math.max(...ys);
  const center={x:(minX+maxX)/2,y:(minY+maxY)/2};
  const scale=400/Math.max(4,Math.max(maxX-minX,maxY-minY)*1.18);
  labels.length=0; strokes.length=0;
  view.setNavigation({frame:'map',domain:'7:3',generation:'g',goals:[{x:1,y:1}],
    route:[{x:0,y:0},{x:1,y:1}]});
  const projected=gridFromMap({x:1,y:1},local.grid_from_odom,transformed.map_from_odom);
  const label=labels.find(entry=>entry.text==='1' && entry.color==='#fff');
  assert.ok(Math.abs(label.x-(300+(projected.x-center.x)*scale))<1e-9);
  assert.ok(Math.abs(label.y-(200-(projected.y-center.y)*scale))<1e-9);
  assert.ok(strokes.includes('#7651b4'),'the global route is drawn in its own color');
  click(400,200);
  assert.equal(selected.length,3);
  const gridPoint=gridFromMap(selected[2].map,local.grid_from_odom,transformed.map_from_odom);
  assert.ok(Math.abs(gridPoint.x-(center.x+100/scale))<1e-9);
  assert.ok(Math.abs(gridPoint.y-center.y)<1e-9,'global selection applies the inverse rotation and translation');
  view.setGlobalMap({map:global,cells:new Uint8Array(400),online:true});
  draws.length=0; handlers.layer_click();
  assert.equal(view.isGlobalOnly(),true); assert.equal(canvas.dataset.frame,'global');
  assert.deepEqual(draws,[rasters[1]],'global-only mode shows the independent SLAM map');
  assert.equal(nodes.get('radarStats').textContent,'局部图已隐藏');
  assert.equal(nodes.get('radarLegend').hidden,true); assert.equal(nodes.get('globalMapLegend').hidden,false);
  assert.equal(view.getInfo().fresh,true,'changing the view does not invalidate ongoing navigation');
  click(300,200); assert.equal(selected.length,4,'global-only mode uses the same selection callback');
  assert.ok(selected[3].map && selected[3].odom);
  handlers.layer_click();
  assert.equal(view.isGlobalOnly(),false); assert.equal(canvas.dataset.globalOverlay,'aligned');
  assert.equal(nodes.get('radarLegend').hidden,false); assert.equal(nodes.get('globalMapLegend').hidden,true);

  draws.length=0;
  view.setGlobalMap({map:{...global,domain:'other'},cells:new Uint8Array(400),online:true});
  assert.equal(canvas.dataset.globalOverlay,'unavailable');
  assert.deepEqual(draws,[rasters[0]]); assert.match(nodes.get('globalMapStatus').textContent,/坐标域不一致/);
  view.setGlobalMap({map:global,cells:new Uint8Array(400),online:true});
  now+=600; draws.length=0; view.draw();
  assert.match(nodes.get('globalMapStats').textContent,/更新 0\.7 秒前/);
  assert.equal(canvas.dataset.frame,'global'); assert.deepEqual(draws,[rasters[1]]);
  click(300,200); assert.equal(selected.length,5);
  now+=200; view.receive(JSON.stringify(local));
  assert.equal(canvas.dataset.frame,'grid'); assert.equal(canvas.dataset.globalOverlay,'unavailable');
  assert.match(nodes.get('globalMapStatus').textContent,/等待新鲜坐标变换/);
  view.setGlobalMap({map:global,cells:new Uint8Array(400),online:true});
  assert.equal(canvas.dataset.globalOverlay,'aligned');
  handlers.layer_click(); draws.length=0;
  view.setGlobalMap({map:{available:false,reason:'坐标域已变化'},online:true});
  assert.equal(canvas.dataset.frame,'global'); assert.deepEqual(draws,[],
    'loss of the global map never silently substitutes the local map in global-only mode');
  click(300,200); assert.equal(selected.length,5);
  handlers.layer_click();
  view.setGlobalMap({map:global,cells:new Uint8Array(400),online:true});
  view.setGlobalMap({map:global,cells:new Uint8Array(400),online:false});
  assert.equal(canvas.dataset.globalOverlay,'unavailable');
  assert.match(nodes.get('globalMapStatus').textContent,/未叠加/);
});
