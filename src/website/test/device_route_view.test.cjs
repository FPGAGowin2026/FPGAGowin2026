'use strict';

const assert = require('node:assert/strict');
const test = require('node:test');
const {createView} = require('../public/js/occupancy_map.js');

test('map displays only fresh FPGA or Jetson routes in the matching odom domain', () => {
  const nodes = new Map();
  const context = {
    strokes:[], path:[], commands:[], texts:[], circles:[],
    beginPath() { this.path=[]; this.commands=[]; },
    moveTo(x,y) { this.path.push([x,y]); this.commands.push('move'); },
    lineTo(x,y) { this.path.push([x,y]); this.commands.push('line'); },
    stroke() { this.strokes.push({color:this.strokeStyle, path:this.path.slice(),commands:this.commands.slice()}); },
    setTransform(){}, fillRect(){}, drawImage(){}, setLineDash(){}, closePath(){},
    fill(){}, fillText(value) { this.texts.push(String(value)); }, arc(x,y,r) { this.circles.push({x,y,r}); }
  };
  const canvas = {dataset:{}, width:0, height:0, getContext:()=>context,
    getBoundingClientRect:()=>({left:0,top:0,width:400,height:300}), addEventListener(){}};
  nodes.set('radarCloud',canvas);
  nodes.set('mapRouteStatus',{textContent:''});
  nodes.set('mapDiagnostics',{textContent:''});
  const rasterContext = {createImageData:(w,h)=>({data:new Uint8Array(w*h*4)}),putImageData(){}};
  const doc = {getElementById:id=>nodes.get(id),createElement:()=>({getContext:()=>rasterContext})};
  class Socket { constructor() { this.readyState=0; Socket.current=this; } }
  let expiry, now=1000;
  const originalNow=Date.now;
  Date.now=()=>now;
  const view=createView({document:doc,window:{devicePixelRatio:1,
    setTimeout:callback=>(expiry=callback,1),clearTimeout:()=>{expiry=null;}},WebSocketImpl:Socket});
  view.connect('ws://local/map'); Socket.current.readyState=1; Socket.current.onopen();
  const snapshot={v:1,type:'mapping',protocol:'fpga_grid_view_v1',source:'fpga',available:true,
    navigation_protocol:'nav_grid_v1',generation:1,revision:1,domain:'7:3',
    status:'building',reason:'FPGA 真实局部栅格',width:64,height:64,resolution_m:.05,
    origin:{x:-1.6,y:-1.6},bounds:{min_x:0,max_x:63,min_y:0,max_y:63},cells_rle:[1,4096],
    counts:{free:4096,occupied:0},pose:{x:0,y:0,yaw:0},pose_odom:{x:0,y:0,yaw:0},
    pose_fresh:true,pose_age_ms:0,snapshot_age_ms:0,grid_age_ms:0,grid_fresh:true,grid_ready:true,grid_gap:false,
    grid_from_odom:{map_anchor_x_mm:0,map_anchor_y_mm:0,odom_anchor_x_mm:0,odom_anchor_y_mm:0,yaw_offset_mrad:0},trajectory:[]};
  const route={source:'fpga',frame_id:'fpga_grid',domain:'7:3',age_ms:20,task_id:5,
    guard_stop:true,points:[{x:0,y:0},{x:0.9,y:0}]};
  const receive = planned_route => {
    context.strokes=[];
    view.receive(JSON.stringify({...snapshot,planned_route}));
    return context.strokes.filter(stroke=>stroke.color==='#db7b13');
  };
  let strokes=receive(undefined);
  assert.equal(strokes.length,0);
  assert.equal(nodes.get('mapRouteStatus').textContent,'');
  strokes=receive(route);
  assert.match(nodes.get('mapRouteStatus').textContent,/FPGA 局部规划段 · task 5 · 保护停车/);
  assert.ok(context.circles.length>0,'真实 grid 使用保护半径');
  assert.match(nodes.get('mapDiagnostics').textContent,/FPGA 局部栅格/);
  assert.equal(strokes.length,1);
  assert.equal(strokes[0].path[0][1],strokes[0].path[1][1], 'the supplied route runs horizontally');
  context.circles=[];
  strokes=receive({...route,grid_direction_preview:true,map_version:42});
  assert.match(nodes.get('mapRouteStatus').textContent,/局部方向预览（两点连线，非完整曲线）/);
  assert.match(nodes.get('mapDiagnostics').textContent,/FPGA 局部栅格/);
  assert.match(nodes.get('mapDiagnostics').textContent,/64×64 · 50 mm\/格/);
  assert.match(nodes.get('mapDiagnostics').textContent,/观测新鲜 · 完整圈就绪 · 缺口无/);
  const lineLength=strokes[0].path[1][0]-strokes[0].path[0][0];
  assert.ok(Math.abs(context.circles.at(-1).r/lineLength-.231/.9)<1e-9);
  now+=480; context.strokes=[]; expiry();
  assert.equal(context.strokes.filter(stroke=>stroke.color==='#db7b13').length,0);
  assert.match(nodes.get('mapRouteStatus').textContent,/过期/);
  strokes=receive({...route,source:'jetson',points:[{x:0,y:0},{x:0,y:1}]});
  assert.match(nodes.get('mapRouteStatus').textContent,/Jetson 局部规划段/);
  assert.equal(strokes[0].path[0][0],strokes[0].path[1][0], 'a new device route replaces the old one');
  for (const invalid of [{...route,domain:'8:3'},{...route,age_ms:500},
    {...route,source:'browser'},{...route,points:[{x:0,y:0},{x:'bad',y:0}]}]) {
    assert.equal(receive(invalid).length,0);
    assert.match(nodes.get('mapRouteStatus').textContent,/不可用|过期|不匹配/);
  }
  context.strokes=[]; context.texts=[];
  const history=[{task_id:5,points:[{x:0,y:0},{x:.2,y:0}]},
    {task_id:6,points:[{x:.4,y:.1},{x:.5,y:.2}]}];
  view.receive(JSON.stringify({...snapshot,planned_route:null,planned_history:history,
    active_goal:{task_id:6,x:.6,y:.3},trajectory:[{x:0,y:0},{x:.1,y:.1}]}));
  const historic=context.strokes.find(stroke=>stroke.color==='rgba(219,123,19,.45)');
  assert.deepEqual(historic.commands,['move','line','move','line'],
    'separate recorded segments are not joined into an invented route');
  assert.ok(context.strokes.some(stroke=>stroke.color==='#0aa5c9' && stroke.path.length===2));
  assert.ok(context.texts.includes('T6'),'the live device goal is labelled with its task id');
  assert.match(nodes.get('mapRouteStatus').textContent,/历史规划短段 2 条/);
  context.texts=[];
  const goals=[{x:0,y:0},{x:0.5,y:0},{x:1,y:0}];
  view.setNavigation({frame:'odom',domain:'7:3',generation:null,goals,route:[],goal_index:0});
  assert.deepEqual(context.texts.filter(value=>/^[123]$/.test(value)),['1','2','3']);
  context.texts=[];
  view.setNavigation({frame:'odom',domain:'7:3',generation:null,goals,route:[],goal_index:1});
  assert.deepEqual(context.texts.filter(value=>/^[123]$/.test(value)),['1','2','3'],
    'the complete queue keeps its numbering when the coordinator advances');
  Date.now=originalNow;
});

test('video inset draws the verified metric route and clears on expiry, domain change and disconnect', t => {
  function drawingContext() {
    return {
      strokes:[], rectangles:[], path:[],
      beginPath() { this.path=[]; },
      moveTo(x,y) { this.path.push([x,y]); },
      lineTo(x,y) { this.path.push([x,y]); },
      stroke() { this.strokes.push({color:this.strokeStyle, path:this.path.slice()}); },
      fillRect(x,y,w,h) { this.rectangles.push({color:this.fillStyle,x,y,w,h}); },
      clearRect() { this.strokes=[]; this.rectangles=[]; },
      setTransform(){}, drawImage(){}, setLineDash(){}, closePath(){},
      fill(){}, fillText(){}, arc(){}
    };
  }
  const mapContext=drawingContext(), insetContext=drawingContext(), nodes=new Map();
  const canvas=(context,width,height)=>({dataset:{},width:0,height:0,getContext:()=>context,
    getBoundingClientRect:()=>({left:0,top:0,width,height}),addEventListener(){}});
  nodes.set('radarCloud',canvas(mapContext,400,300));
  nodes.set('videoRouteCanvas',canvas(insetContext,160,128));
  nodes.set('videoRoutePreview',{hidden:false});
  const toggle={textContent:'',attributes:{},listeners:{},
    setAttribute(name,value){this.attributes[name]=value;},
    addEventListener(name,callback){this.listeners[name]=callback;},
    click(){this.listeners.click();}};
  nodes.set('videoRouteToggle',toggle);
  for (const id of ['videoRouteStatus','mapRouteStatus']) nodes.set(id,{textContent:''});
  const rasterContext={createImageData:(w,h)=>({data:new Uint8Array(w*h*4)}),putImageData(){}};
  const doc={getElementById:id=>nodes.get(id),createElement:()=>({getContext:()=>rasterContext})};
  let now=1000, nextTimer=0;
  const timers=new Map(), originalNow=Date.now;
  Date.now=()=>now;
  t.after(()=>{Date.now=originalNow;});
  const win={devicePixelRatio:1,
    setTimeout(callback,delay){const id=++nextTimer;timers.set(id,{callback,at:now+delay});return id;},
    clearTimeout(id){timers.delete(id);}};
  function advance(ms) {
    const target=now+ms;
    for (;;) {
      const due=[...timers].filter(([,timer])=>timer.at<=target).sort((a,b)=>a[1].at-b[1].at)[0];
      if (!due) break;
      timers.delete(due[0]); now=due[1].at; due[1].callback();
    }
    now=target;
  }
  class Socket { constructor(){this.readyState=0;Socket.current=this;} close(){this.readyState=3;} }
  const view=createView({document:doc,window:win,WebSocketImpl:Socket});
  view.connect('ws://local/map'); Socket.current.readyState=1; Socket.current.onopen();
  const snapshot={v:1,type:'mapping',protocol:'fpga_grid_view_v1',source:'fpga',available:true,
    navigation_protocol:'nav_grid_v1',generation:1,revision:1,domain:'7:3',
    status:'building',reason:'FPGA 真实局部栅格',width:64,height:64,resolution_m:.05,
    origin:{x:-1.6,y:-1.6},bounds:{min_x:0,max_x:63,min_y:0,max_y:63},cells_rle:[1,4096],
    counts:{free:4096,occupied:0},pose:{x:0,y:0,yaw:Math.PI/2},pose_odom:{x:0,y:0,yaw:Math.PI/2},
    pose_fresh:true,pose_age_ms:0,snapshot_age_ms:0,grid_age_ms:0,grid_fresh:true,grid_ready:true,grid_gap:false,
    grid_from_odom:{map_anchor_x_mm:0,map_anchor_y_mm:0,odom_anchor_x_mm:0,odom_anchor_y_mm:0,yaw_offset_mrad:0},trajectory:[]};
  const route={source:'fpga',frame_id:'fpga_grid',domain:'7:3',age_ms:10,task_id:5,
    grid_direction_preview:true,guard_stop:false,points:[{x:0,y:0},{x:0,y:.02}]};
  const send=(map,plan)=>view.receive(JSON.stringify({...map,planned_route:plan}));
  const orange=()=>insetContext.strokes.filter(stroke=>stroke.color==='#db7b13');
  send(snapshot,route);
  assert.equal(nodes.get('videoRouteCanvas').dataset.state,'fresh');
  assert.equal(nodes.get('videoRouteStatus').textContent,'FPGA · 20 mm · 前');
  assert.equal(orange().length,1);
  const [[x1,y1],[x2,y2]]=orange()[0].path;
  assert.ok(Math.abs(x2-x1)<1e-9 && y2<y1,'odom +Y is forward for a 90-degree heading');
  assert.ok(Math.abs((y1-y2)-.02*140)<1e-9,'the 20 mm segment uses the same metre scale as the vehicle');
  send(snapshot,{...route,points:[{x:0,y:0},{x:0,y:.04}]});
  assert.ok(Math.abs((orange()[0].path[0][1]-orange()[0].path[1][1])-2*(y1-y2))<1e-9,
    'doubling the real segment doubles its on-screen length');

  // For 90-degree yaw, body-forward is odom +Y and body-left is odom -X.
  // Offset both endpoints from the robot so direction must use the segment vector.
  send(snapshot,{...route,points:[{x:.2,y:.1},{x:.19,y:.12}]});
  assert.equal(nodes.get('videoRouteStatus').textContent,'FPGA · 22 mm · 前偏左27°');
  assert.ok(orange()[0].path[1][0] < orange()[0].path[0][0], 'leftward segment draws to the left');
  send(snapshot,{...route,points:[{x:.2,y:.1},{x:.21,y:.12}]});
  assert.equal(nodes.get('videoRouteStatus').textContent,'FPGA · 22 mm · 前偏右27°');
  assert.ok(orange()[0].path[1][0] > orange()[0].path[0][0], 'rightward segment draws to the right');
  send(snapshot,{...route,points:[{x:.2,y:.1},{x:.19,y:.08}]});
  assert.equal(nodes.get('videoRouteStatus').textContent,'FPGA · 22 mm · 后偏左27°');
  send(snapshot,{...route,points:[{x:.2,y:.1},{x:.19,y:.1}]});
  assert.equal(nodes.get('videoRouteStatus').textContent,'FPGA · 10 mm · 左');

  send(snapshot,{...route,points:[{x:0,y:0},{x:0,y:0}]});
  assert.equal(orange().length,0,'coincident endpoints do not create a fictitious forward line');
  assert.equal(nodes.get('videoRouteStatus').textContent,'FPGA · 端点重合');
  assert.ok(insetContext.rectangles.some(rect=>rect.color==='#ffad43'),'the actual endpoint remains visible');
  send(snapshot,route);
  advance(489);
  assert.equal(nodes.get('videoRouteCanvas').dataset.state,'fresh');
  advance(1);
  assert.equal(nodes.get('videoRouteCanvas').dataset.state,'empty','route is cleared at its 500 ms age limit');
  assert.equal(orange().length,0);
  assert.match(nodes.get('videoRouteStatus').textContent,/暂无新鲜设备规划/);

  send(snapshot,{...route,age_ms:10.1});
  advance(489);
  assert.equal(nodes.get('videoRouteCanvas').dataset.state,'fresh');
  advance(1);
  assert.equal(nodes.get('videoRouteCanvas').dataset.state,'empty','fractional source age cannot leave an expired preview visible');
  send({...snapshot,pose_age_ms:490},{...route,age_ms:0});
  advance(10);
  assert.equal(nodes.get('videoRouteCanvas').dataset.state,'fresh');
  advance(1);
  assert.equal(nodes.get('videoRouteCanvas').dataset.state,'empty','stale pose clears the inset even if route age is below 500 ms');
  send(snapshot,route);
  send(snapshot,null);
  assert.equal(nodes.get('videoRouteCanvas').dataset.state,'empty','an ended or invalidated task clears its old route');
  assert.equal(orange().length,0);
  send(snapshot,route);
  toggle.click();
  assert.equal(nodes.get('videoRoutePreview').hidden,true);
  assert.equal(toggle.attributes['aria-pressed'],'false');
  send({...snapshot,domain:'8:3',generation:2},route);
  toggle.click();
  assert.equal(nodes.get('videoRoutePreview').hidden,false);
  assert.equal(toggle.attributes['aria-pressed'],'true');
  assert.equal(nodes.get('videoRouteCanvas').dataset.state,'empty','a hidden old-domain route cannot reappear on show');
  send({...snapshot,domain:'8:3',generation:2},{...route,domain:'8:3'});
  assert.equal(nodes.get('videoRouteCanvas').dataset.state,'fresh');
  view.disconnect();
  assert.equal(nodes.get('videoRouteCanvas').dataset.state,'empty','disconnect clears the current preview');
  assert.equal(orange().length,0);
});

test('the video inset does not intercept clicks intended for detection boxes', () => {
  const fs=require('node:fs'), path=require('node:path');
  const html=fs.readFileSync(path.join(__dirname,'../public/index.html'),'utf8');
  const css=fs.readFileSync(path.join(__dirname,'../public/css/workspace.css'),'utf8');
  assert.match(html,/id="videoRoutePreview"[^>]*class="video-route-preview"/);
  assert.match(css,/\.video-route-preview\s*\{[^}]*pointer-events:\s*none/s);
  assert.match(html,/局部规划俯视图（非画面投影）/);
});
