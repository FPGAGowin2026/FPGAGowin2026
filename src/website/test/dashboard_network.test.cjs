'use strict';

const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const test = require('node:test');
const dashboard = require('../public/js/dashboard_network.js');

function fpgaMapFixture(overrides={}) {
  return {v:1,type:'mapping',protocol:'fpga_grid_view_v1',source:'fpga',available:true,
    navigation_protocol:'nav_grid_v1',generation:1,revision:1,domain:'7:3',status:'building',reason:'FPGA 真实局部栅格',
    width:64,height:64,resolution_m:.05,origin:{x:-1.6,y:-1.6},bounds:{min_x:0,max_x:63,min_y:0,max_y:63},
    cells_rle:[1,4096],counts:{free:4096,occupied:0},pose:{x:0,y:0,yaw:0},pose_odom:{x:0,y:0,yaw:0},
    pose_fresh:true,pose_age_ms:0,snapshot_age_ms:0,grid_age_ms:0,grid_fresh:true,grid_ready:true,grid_gap:false,
    grid_from_odom:{map_anchor_x_mm:0,map_anchor_y_mm:0,odom_anchor_x_mm:0,odom_anchor_y_mm:0,yaw_offset_mrad:0},
    trajectory:[],...overrides};
}

class MockWebSocket {
  static OPEN=1;
  static instances=[];
  constructor(url) { this.url=url; this.readyState=0; this.sent=[]; MockWebSocket.latest=this; MockWebSocket.instances.push(this); }
  send(text) { this.sent.push(JSON.parse(text)); }
  open() { this.readyState=1; this.onopen?.(); }
  message(value) { this.onmessage?.({data:JSON.stringify(value)}); }
  close() { this.readyState=3; this.onclose?.(); }
}

test('visual task commands are explicit and service results preserve unknown status', () => {
  const results=[];
  const adapter=new dashboard.DashboardNetworkAdapter({WebSocketImpl:MockWebSocket,
    onMissionResult:result=>results.push(result)});
  adapter.connect('ws://localhost/ws');
  const socket=MockWebSocket.latest;
  socket.open();
  assert.equal(adapter.missionCommand('enable_ros'),true);
  assert.deepEqual(socket.sent.at(-1),{v:1,type:'mission_command',action:'enable_ros'});
  assert.equal(adapter.missionCommand('invalid'),false);
  socket.message({v:1,type:'mission_result',action:'start',success:false,unknown:true,message:'结果未知'});
  assert.equal(results[0].unknown,true);
  assert.equal(results[0].success,false);
});

test('visual overlay mapping preserves contain letterboxing', () => {
  const box={cx:400,cy:240,width:100,height:100};
  assert.deepEqual(dashboard.visionRect(1000,1000,800,480,box),
    {x:437.5,y:437.5,width:125,height:125});
  assert.equal(dashboard.visionRect(0,1000,800,480,box),null);
  assert.equal(dashboard.visionRect(1000,1000,800,480,{...box,width:NaN}),null);
});

test('a slow-page disconnect reports the close reason and clears control', () => {
  const connections=[];
  const adapter=new dashboard.DashboardNetworkAdapter({WebSocketImpl:MockWebSocket,
    onConnection:(connected,message)=>connections.push({connected,message})});
  adapter.connect('ws://localhost/ws');
  const socket=MockWebSocket.latest;
  socket.open();
  adapter.arm();
  socket.readyState=3;
  socket.onclose({code:1013,reason:'client send queue exceeded'});
  assert.equal(adapter.localControlEnabled,false);
  assert.equal(adapter.state,null);
  assert.equal(adapter.connected,false);
  assert.match(connections.at(-1).message,/页面接收积压.*1013/);
});

test('consumption is acknowledged after page handlers, and handler failure stops acknowledgements', () => {
  const calls=[];
  const adapter=new dashboard.DashboardNetworkAdapter({WebSocketImpl:MockWebSocket,
    onState:state=>{ if (state) calls.push('state'); },
    onTelemetry:()=>{ calls.push('telemetry'); }});
  const socket=adapter.connect('ws://localhost/ws'); socket.open();
  socket.message({v:1,type:'hello',client_id:'page',delivery_ack:1});
  assert.deepEqual(socket.sent.at(-1),{v:1,type:'delivery_ready'});
  socket.message({v:1,type:'state',status:{bridge_id:'b'}});
  socket.message({v:1,type:'telemetry',stream:'motor',bridge_id:'b',epoch:1,valid:true,data:{sequence:1}});
  assert.deepEqual(calls,['state','telemetry']);
  assert.equal(socket.sent.some(item=>item.type==='delivery_ack'),false);
  socket.message({v:1,type:'delivery_probe',id:10});
  assert.deepEqual(socket.sent.at(-1),{v:1,type:'delivery_ack',id:10});
  adapter.onTelemetry=()=>{ throw Error('render failure'); };
  assert.throws(()=>socket.message({v:1,type:'telemetry',stream:'motor',bridge_id:'b',epoch:1,valid:true,data:{sequence:2}}));
  socket.message({v:1,type:'delivery_probe',id:11});
  assert.equal(socket.sent.some(item=>item.type==='delivery_ack'&&item.id===11),false);
  assert.equal(socket.sent.some(item=>['arm','local_task','mission_command'].includes(item.type)),false);
});

test('an unconfirmed arm and another owner clear local intent without stopping that owner', async () => {
  const page=startDashboard(), socket=await openNetworkService(page);
  const state={v:1,type:'state',owner:null,armed:false,jetson:{online:true,age_ms:0},
    fpga:{ready:true,ack_age_ms:0},status:{bridge_id:'ownership'},video:{}};
  socket.message(state);
  page.elements.get('networkArm').dispatch('click');
  assert.equal(socket.sent.at(-1).type,'arm');
  const start=socket.sent.length;
  socket.message({...state,owner:'another-page',armed:true});
  socket.message({v:1,type:'event',event:'arm_rejected',reason:'another-page-owns-control'});
  page.runIntervals(50);
  assert.equal(socket.sent.slice(start).some(item=>item.type==='stop' || item.type==='drive'),false);
  assert.match(page.elements.get('networkEvent').textContent,/其他页面持有控制权/);
});

function makeDashboardDom(html) {
  const elements = new Map();
  const dataButtons = [];
  const navLinks = [];
  const documentListeners = new Map();
  const windowListeners = new Map();
  const intervalDelays = [];
  const intervalCallbacks = new Map();
  const timeoutCallbacks = new Map();
  let nextTimer = 1;
  let focused = true;

  function parseAttributes(source) {
    const attributes = Object.create(null);
    for (const match of source.matchAll(/([^\s=/>]+)(?:\s*=\s*(?:"([^"]*)"|'([^']*)'|([^\s>]+)))?/g))
      attributes[match[1]] = match[2] ?? match[3] ?? match[4] ?? '';
    return attributes;
  }

  class FakeClassList {
    constructor() { this.values = new Set(); }
    toggle(name, force) {
      const enabled = force === undefined ? !this.values.has(name) : !!force;
      if (enabled) this.values.add(name); else this.values.delete(name);
      return enabled;
    }
    add(name) { this.values.add(name); }
    remove(name) { this.values.delete(name); }
    contains(name) { return this.values.has(name); }
  }
  class FakeElement {
    constructor(id='', tagName='div') {
      this._id = '';
      this.tagName = String(tagName).toUpperCase();
      this.attributes = Object.create(null);
      this.dataset = {};
      this.style = {};
      this.classList = new FakeClassList();
      this.children = [];
      this.listeners = new Map();
      this.hidden = false;
      this.open = false;
      this._disabled = false;
      this.value = '';
      this._textContent = '';
      this.textWrites = 0;
      this._innerHTML = '';
      this.width = 0;
      this.height = 0;
      if (id) this.id = id;
    }
    get id() { return this._id; }
    get textContent() { return this._textContent; }
    set textContent(value) { this._textContent=String(value); this.textWrites++; }
    set id(value) {
      this._id = String(value);
      if (this._id) elements.set(this._id, this);
    }
    get innerHTML() { return this._innerHTML; }
    set innerHTML(value) {
      this._innerHTML = String(value);
      for (const match of this._innerHTML.matchAll(/\bid="([^"]+)"/g)) {
        if (!elements.has(match[1])) new FakeElement(match[1]);
      }
    }
    get disabled() { return this._disabled; }
    set disabled(value) { this._disabled = !!value; }
    addEventListener(type, listener) {
      const list = this.listeners.get(type) || [];
      list.push(listener); this.listeners.set(type, list);
    }
    dispatch(type, event={}) {
      if (!Object.hasOwn(event,'target')) event.target = this;
      let result;
      for (const listener of this.listeners.get(type) || []) result = listener(event);
      return result;
    }
    appendChild(child) { this.children.push(child); return child; }
    replaceChildren(...children) { this.children = children; }
    click() { return this.dispatch('click'); }
    focus() {}
    showModal() { this.open = true; }
    close() { this.open = false; this.dispatch('close'); }
    remove() { if (this.id) elements.delete(this.id); }
    setAttribute(name, value) {
      this.attributes[name] = String(value);
      this[name] = name==='disabled' || name==='hidden' || name==='open' ? true : String(value);
    }
    getAttribute(name) { return Object.hasOwn(this.attributes,name) ? this.attributes[name] : null; }
    removeAttribute(name) {
      delete this.attributes[name];
      if (name==='disabled' || name==='hidden' || name==='open') this[name] = false;
      else delete this[name];
    }
    setPointerCapture() {}
    scrollIntoView(options) { this.lastScroll = options; }
    getBoundingClientRect() { return {left:0,top:0,width:400,height:300}; }
    getContext() {
      return new Proxy({}, {get:(_,key) => key === 'createImageData'
        ? ((width,height)=>({data:new Uint8ClampedArray(width*height*4)}))
        : typeof key === 'string' ? (()=>{}) : undefined, set:()=>true});
    }
  }

  for (const match of html.matchAll(/<([a-z][\w-]*)\b([^>]*)>/gi)) {
    const attributes = parseAttributes(match[2]);
    if (!attributes.id) continue;
    const element = elements.get(attributes.id) || new FakeElement(attributes.id,match[1]);
    element.tagName = match[1].toUpperCase();
    for (const [name,value] of Object.entries(attributes)) element.setAttribute(name,value);
  }
  for (const match of html.matchAll(/<select\b([^>]*)>([\s\S]*?)<\/select>/gi)) {
    const attributes = parseAttributes(match[1]);
    if (!attributes.id) continue;
    const select = elements.get(attributes.id);
    select.options = [...match[2].matchAll(/<option\b([^>]*)>([\s\S]*?)<\/option>/gi)].map(option => {
      const optionAttributes = parseAttributes(option[1]);
      return {value:optionAttributes.value ?? '',selected:Object.hasOwn(optionAttributes,'selected')};
    });
    const selected = select.options.find(option => option.selected) || select.options[0];
    select.value = selected?.value || '';
  }
  for (const match of html.matchAll(/<a\b([^>]*)>([\s\S]*?)<\/a>/gi)) {
    const attrs = parseAttributes(match[1]);
    const link = attrs.id ? (elements.get(attrs.id) || new FakeElement(attrs.id,'a')) : new FakeElement('','a');
    link.tagName = 'A';
    for (const [name,value] of Object.entries(attrs)) link.setAttribute(name,value);
    link.textContent = match[2].replace(/<[^>]*>/g,'').trim();
    navLinks.push(link);
  }
  for (const match of html.matchAll(/<button\b([^>]*)>/gi)) {
    const attrs = parseAttributes(match[1]);
    const button = attrs.id
      ? (elements.get(attrs.id) || new FakeElement(attrs.id,'button'))
      : new FakeElement('','button');
    button.tagName = 'BUTTON';
    for (const [name,value] of Object.entries(attrs)) button.setAttribute(name,value);
    const drive = attrs['data-drive-key'];
    const network = attrs['data-network-key'];
    if (!drive && !network) continue;
    if (drive) button.dataset.driveKey = drive;
    if (network) button.dataset.networkKey = network;
    dataButtons.push(button);
  }

  const document = {
    hidden:false,
    getElementById:id => elements.get(String(id)) || null,
    createElement:() => new FakeElement(),
    querySelectorAll:selector => {
      if (selector === '.console-nav .nav-item') return navLinks;
      if (selector === 'a[data-nav]') return navLinks.filter(link=>link.getAttribute('data-nav'));
      if (selector === '[data-drive-key]') return dataButtons.filter(button=>!!button.dataset.driveKey);
      if (selector === '[data-network-key]') return dataButtons.filter(button=>!!button.dataset.networkKey);
      return [];
    },
    addEventListener(type,listener) {
      const list=documentListeners.get(type)||[]; list.push(listener); documentListeners.set(type,list);
    },
    hasFocus:() => focused
  };
  document.dispatch = (type,event={}) => {
    for (const listener of documentListeners.get(type)||[]) listener(event);
  };
  const window = {
    devicePixelRatio:1,
    addEventListener(type,listener) {
      const list=windowListeners.get(type)||[]; list.push(listener); windowListeners.set(type,list);
    },
    setInterval:(listener,delay) => {
      intervalDelays.push(delay);
      const id=nextTimer++;
      intervalCallbacks.set(id,{listener,delay});
      return id;
    },
    clearInterval:id => { intervalCallbacks.delete(id); },
    setTimeout:(listener,delay) => {
      const id=nextTimer++;
      timeoutCallbacks.set(id,{listener,delay});
      return id;
    },
    clearTimeout:id => { timeoutCallbacks.delete(id); },
    focus(value) { focused=value; },
    dispatch(type,event={}) {
      if (type === 'blur') focused=false;
      for (const listener of windowListeners.get(type)||[]) listener(event);
    }
  };
  return {
    document,window,elements,intervalDelays,
    runIntervals(delay) {
      for (const interval of [...intervalCallbacks.values()])
        if (interval.delay===delay) interval.listener();
    },
    runTimeouts(delay) {
      for (const [id,timeout] of [...timeoutCallbacks]) {
        if (timeout.delay!==delay) continue;
        timeoutCallbacks.delete(id);
        timeout.listener();
      }
    },
    pendingTimeouts(delay) {
      return [...timeoutCallbacks.values()].filter(timeout=>timeout.delay===delay).length;
    }
  };
}

function startDashboard({navigator={},protocol='file:',initialHash='',globalMapSource=null}={}) {
  const csvExports=[];
  const html=fs.readFileSync(path.join(__dirname,'..','public','index.html'),'utf8');
  const script=fs.readFileSync(path.join(__dirname,'..','public','js','app.js'),'utf8');
  const dom=makeDashboardDom(html);
  let wallClock=Date.now();
  let performanceClock=1;
  class DashboardDate extends Date { static now() { return wallClock; } }
  const context={
    document:dom.document,
    window:dom.window,
    navigator,
    location:{protocol,host:'127.0.0.1:8765',hash:initialHash},
    performance:{now:()=>performanceClock},
    WebSocket:MockWebSocket,
    Blob:class { constructor(parts) { this.parts=parts; } },
    URL:{createObjectURL(blob) { csvExports.push(blob.parts.join('')); return 'blob:test'; },revokeObjectURL(){}},
    Date:DashboardDate,
    BX72DashboardNetwork:{
      ...dashboard,
      DashboardNetworkAdapter:class extends dashboard.DashboardNetworkAdapter {
        constructor(options={}) {
          super({...options,now:options.now || (()=>performanceClock),WebSocketImpl:MockWebSocket});
        }
      }
    }
  };
  if (globalMapSource) context.BX72GlobalMap=globalMapSource;
  vm.createContext(context);
  new vm.Script(fs.readFileSync(path.join(__dirname,'..','..','ugv_ws','src','car','car_navigation','scripts','global_planner','public','js','grid_codec.js'),'utf8'),
    {filename:'public/js/grid_codec.js'}).runInContext(context);
  new vm.Script(fs.readFileSync(path.join(__dirname,'..','public','js','occupancy_map.js'),'utf8'),
    {filename:'public/js/occupancy_map.js'}).runInContext(context);
  new vm.Script(script,{filename:'public/js/app.js'}).runInContext(context);
  return {
    ...dom,context,csvExports,
    setWallClock:value=>{wallClock=value;},
    advancePerf:milliseconds=>{performanceClock+=milliseconds;},
    setPerfNow:value=>{performanceClock=value;}
  };
}

async function flushMicrotasks() {
  for (let index=0;index<20;index++) await Promise.resolve();
}

async function openNetworkService(page,{clientId='test-page',simulation=true}={}) {
  await page.elements.get('networkConnect').dispatch('click');
  const socket=MockWebSocket.latest;
  assert.ok(socket,'network websocket was created');
  socket.open();
  socket.message({v:1,type:'hello',client_id:clientId,simulation,limits:{speed:100,turn:300}});
  return socket;
}

test('network adapter keeps telemetry streams independent, wraps sequence numbers and rejects duplicates/old bridges', () => {
  let now=100;
  const received=[];
  const adapter=new dashboard.DashboardNetworkAdapter({WebSocketImpl:MockWebSocket,now:()=>now,onTelemetry:(stream,item)=>received.push([stream,item.data.sequence])});
  const socket=adapter.connect('ws://127.0.0.1:8765/ws'); socket.open();
  socket.message({v:1,type:'hello',client_id:'page-1',simulation:true,limits:{speed:100,turn:300}});
  socket.message({v:1,type:'state',owner:null,armed:false,jetson:{online:true},fpga:{ready:true},status:{bridge_id:'bridge-a'},video:{},simulation:true});
  const telemetry=(stream,sequence,bridge='bridge-a',epoch=1)=>({v:1,type:'telemetry',bridge_id:bridge,stream,epoch,valid:true,data:{type:stream,sequence,time_us:1}});
  socket.message(telemetry('fusion',0xffffffff));
  socket.message(telemetry('motor',10));
  socket.message(telemetry('fusion',0));
  socket.message(telemetry('fusion',0));
  socket.message(telemetry('fusion',0xffffffff));
  assert.deepEqual(received,[['fusion',0xffffffff],['motor',10],['fusion',0]]);
  assert.equal(adapter.latest('fusion',1000,now).data.sequence,0);
  now+=1001;
  assert.equal(adapter.latest('fusion',1000,now),null);
  socket.message({v:1,type:'state',owner:null,armed:false,jetson:{online:true},fpga:{ready:true},status:{bridge_id:'bridge-b'},video:{},simulation:true});
  socket.message(telemetry('fusion',1,'bridge-a'));
  assert.deepEqual(received,[['fusion',0xffffffff],['motor',10],['fusion',0]]);
});

test('network adapter sends versioned UDP controls with fresh drive sequences', () => {
  const adapter=new dashboard.DashboardNetworkAdapter({WebSocketImpl:MockWebSocket});
  const socket=adapter.connect('ws://localhost/ws'); socket.open();
  socket.message({v:1,type:'hello',client_id:'page-2',simulation:false,limits:{speed:100,turn:300}});
  socket.message({v:1,type:'state',owner:null,armed:false,status:{bridge_id:'b'},jetson:{online:true},fpga:{ready:true},video:{}});
  assert.equal(adapter.configure('192.168.0.4',6200),true);
  assert.equal(adapter.arm(),true);
  assert.equal(adapter.drive(1,2,3),true);
  assert.equal(adapter.drive(4,5,6),true);
  assert.equal(adapter.navigationTask('start',{frame:'odom',domain:'7:3',generation:null,goals:[{x:1,y:2}]}),true);
  assert.equal(adapter.stop('test stop'),true);
  const controls=socket.sent;
  assert.deepEqual(controls.map(item=>item.type),['configure','arm','drive','drive','navigation_task','stop']);
  assert.deepEqual(controls.filter(item=>item.type==='drive').map(item=>item.seq),[1,2]);
  assert.deepEqual(controls.find(item=>item.type==='navigation_task'),
    {v:1,type:'navigation_task',action:'start',frame:'odom',domain:'7:3',generation:null,goals:[{x:1,y:2}]});
  assert.equal(controls.every(item=>item.v===1),true);
});

test('current dashboard loads the network adapter before its valid app script', () => {
  const html=fs.readFileSync(path.join(__dirname,'..','public','index.html'),'utf8');
  const scripts=[...html.matchAll(/<script\b[^>]*src="([^"]+)"/g)].map(match=>match[1]);
  assert.deepEqual(scripts,['js/dashboard_network.js','js/grid_codec.js','js/occupancy_map.js','js/global_map.js','js/app.js']);
  const startNote=html.match(/<p class="local-start-note">([^<]+)<\/p>/)?.[1] || '';
  assert.match(startNote,/首次任务前.*确认.*231 mm/);
  assert.match(startNote,/许可圆固定.*仅豁免未知栅格/);
  assert.match(startNote,/地图重建.*清除旧任务和许可圆.*重新选点启动/);
  const script=fs.readFileSync(path.join(__dirname,'..','public','js','app.js'),'utf8');
  assert.doesNotThrow(()=>new vm.Script(script,{filename:'public/js/app.js'}));
});

test('dashboard starts with required DOM and renders network state/fusion without a renderer reference error', async () => {
  MockWebSocket.latest=null;
  const page=startDashboard();
  const connect=page.elements.get('networkConnect').dispatch('click');
  await connect;
  const socket=MockWebSocket.latest;
  assert.ok(socket);
  socket.open();
  socket.message({v:1,type:'hello',client_id:'browser-1',simulation:true,limits:{speed:80,turn:200}});
  socket.message({v:1,type:'state',owner:null,armed:false,simulation:true,
    jetson:{online:true,host:'127.0.0.1',port:16200,age_ms:5},
    fpga:{ready:true,ack_age_ms:5},status:{bridge_id:'demo-bridge',actual_mode:'manual',auto_requested:false,auto_acknowledged:false},
    stop:{requested:true,pc_seq:7,session:'sim-1',fpga_acknowledged:true,wheels_low:false,threshold_mm_s:5,motor_age_ms:40},video:{}});
  assert.equal(page.elements.get('jetsonHost').value,'127.0.0.1');
  assert.equal(page.elements.get('jetsonPort').value,'16200');
  assert.equal(page.elements.get('jetsonHost').disabled,true);
  assert.equal(page.elements.get('networkApplyTarget').disabled,true);
  assert.equal(socket.sent.some(message=>message.type==='configure'),false,'opening monitoring must not override service target');
  socket.message({v:1,type:'telemetry',bridge_id:'demo-bridge',stream:'fusion',epoch:1,valid:true,
    data:{type:'fusion',sequence:1,flags:1,state:{x_m:1,y_m:2,yaw_rad:Math.PI/2,vx_m_s:0.1,vy_m_s:0,wz_rad_s:0,gyro_bias_rad_s:0},covariance_diag:[0,0,0,0,0,0,0],updates:1}});
  page.runIntervals(250);
  assert.equal(page.elements.get('v0').textContent,'1.0000');
  assert.equal(page.elements.get('v2').textContent,'90.00');
  assert.match(page.elements.get('poseHeading').textContent,/UDP 融合/);
  assert.equal(page.elements.get('networkActualMode').textContent,'FPGA融合状态位：手动位');
  assert.match(page.elements.get('networkStopRequest').textContent,/服务报告已请求/);
  assert.match(page.elements.get('networkStopAck').textContent,/已回执零速入队/);
  assert.match(page.elements.get('networkWheelsLow').textContent,/反馈仍高于阈值/);
  socket.message({v:1,type:'telemetry',bridge_id:'demo-bridge',stream:'fusion',epoch:1,valid:true,
    data:{type:'fusion',sequence:2,flags:0,state:{x_m:9,y_m:8,yaw_rad:1,vx_m_s:1,vy_m_s:1,wz_rad_s:1,gyro_bias_rad_s:1},covariance_diag:[0,0,0,0,0,0,0]}});
  page.runIntervals(250);
  assert.equal(page.elements.get('v0').textContent,'不可用','flags bit 0 clear must suppress fusion state values');
  assert.match(page.elements.get('lastSeen').textContent,/flags bit0=0/);
  page.elements.get('networkStop').dispatch('click');
  assert.equal(socket.sent.at(-1).type,'stop');
  assert.match(page.elements.get('networkStopRequest').textContent,/本页已发送/);
});

test('HTTP dashboard automatically opens monitoring and video without arming control', () => {
  MockWebSocket.instances=[];
  const page=startDashboard({protocol:'http:'});
  assert.deepEqual(MockWebSocket.instances.map(socket=>socket.url).sort(),
    ['ws://127.0.0.1:8765/map','ws://127.0.0.1:8765/video','ws://127.0.0.1:8765/ws']);
  assert.equal(page.elements.has('networkConnect'),true);
  assert.equal(page.elements.has('localTaskPanel'),true);
  assert.equal(page.elements.has('track'),false,'trajectory uses the occupancy map canvas');
  assert.equal(page.elements.has('mapPose'),true);
  assert.doesNotMatch(fs.readFileSync(path.join(__dirname,'..','public','index.html'),'utf8'),/Bluetooth|蓝牙|串口|navigator\.serial/i);
  for (const socket of MockWebSocket.instances) {
    socket.open();
    assert.equal(socket.sent.some(message=>['arm','drive','auto'].includes(message.type)),false);
  }
});

test('odom drafts survive grid changes and stay in one frame across map views', () => {
  MockWebSocket.instances=[];
  let deliverGlobal;
  const globalMapSource={createSource:({onUpdate})=>{deliverGlobal=onUpdate;return {start(){}};}};
  const page=startDashboard({protocol:'http:',globalMapSource});
  const ws=MockWebSocket.instances.find(socket=>socket.url.endsWith('/ws'));
  const map=MockWebSocket.instances.find(socket=>socket.url.endsWith('/map'));
  ws.open(); map.open();
  ws.message({v:1,type:'hello',client_id:'map-test',simulation:true,limits:{speed:100,turn:300}});
  const nav={boot_id:7,odom_domain:3,age_ms:0,stale:false,task_id_watermark_known:true,
    last_task_id:0,task_id:0,status:0,mode:1,active_task:false,goal_x_q16:0,goal_y_q16:0};
  let seq=0;
  const state=navigation=>ws.message({v:1,type:'state',owner:null,armed:false,jetson:{online:true},fpga:{ready:true},
    status:{bridge_id:'map-test',seq:seq++,emergency_stop:false},navigation,video:{}});
  state(nav);state(nav);
  const snapshot=fpgaMapFixture();
  map.message({...snapshot,status:'paused'});
  const canvas=page.elements.get('radarCloud');
  const click=x=>{canvas.dispatch('pointerdown',{button:0,pointerId:1,clientX:x,clientY:150});
    canvas.dispatch('pointerup',{pointerId:1,clientX:x,clientY:150});};
  click(240);
  assert.match(page.elements.get('mapGoalHint').textContent,/已选 1 个航点/);
  assert.equal(page.elements.get('localTaskStart').disabled,false);
  map.message(snapshot);
  assert.match(page.elements.get('mapGoalHint').textContent,/已选 1 个航点 · odom x 0\.533/);
  assert.equal(page.elements.get('localTaskStart').disabled,false);
  const global={available:true,status:'building',reason:'SLAM',domain:'7:3',generation:'g',revision:1,
    width:20,height:20,resolution_m:.16,origin:{x:-1.6,y:-1.6},counts:{free:400,occupied:0},map_age_ms:0,
    map_from_odom:{x:.5,y:-.3,yaw:Math.PI/2},map_from_odom_age_ms:0};
  deliverGlobal({online:true,cells:vm.runInContext('new Uint8Array(400)',page.context),map:global});
  page.elements.get('mapLayerToggle').dispatch('click');
  assert.equal(page.elements.get('localTaskStart').disabled,false,'view does not change task type');
  click(220);
  assert.match(page.elements.get('mapGoalHint').textContent,/已选 2 个航点 · odom/);
  page.elements.get('mapLayerToggle').dispatch('click');
  map.message({...snapshot,generation:2,revision:2,grid_from_odom:{map_anchor_x_mm:1000,
    map_anchor_y_mm:500,odom_anchor_x_mm:200,odom_anchor_y_mm:-300,yaw_offset_mrad:1571}});
  assert.match(page.elements.get('mapGoalHint').textContent,/已选 2 个航点 · odom x 0\.533/);
  page.elements.get('localTaskStart').dispatch('click');
  const request=ws.sent.at(-1);
  assert.equal(request.type,'navigation_task'); assert.equal(request.action,'start');
  assert.equal(request.frame,'odom'); assert.equal(request.domain,'7:3'); assert.equal(request.generation,null);
  assert.deepEqual(request.goals,[{x:.533,y:0},{x:.3,y:.233}],
    'a later global-view click is converted into the original odom frame');
  assert.equal('hold_yaw_rad' in request,false,'waypoints do not impose a heading');
  ws.message({v:1,type:'mission_result',action:'navigation_start',success:false,message:'模拟拒绝'});
  assert.match(page.elements.get('mapGoalHint').textContent,/已选 2 个航点/,'a rejected request keeps the draft');
  state({...nav,odom_domain:4});
  assert.match(page.elements.get('mapGoalHint').textContent,/已选 2 个航点/);
  assert.equal(page.elements.get('localTaskStart').disabled,false);
  page.elements.get('localTaskStart').dispatch('click');
  assert.equal(ws.sent.at(-1).domain,'7:3','Jetson can reject the original domain; the page must not silently retag it');
});

test('many telemetry messages update one latest display on the 250 ms presentation timer', async () => {
  const page=startDashboard(), socket=await openNetworkService(page);
  socket.message({v:1,type:'state',owner:null,armed:false,jetson:{online:true,age_ms:0},
    fpga:{ready:true},status:{bridge_id:'coalesce'},video:{}});
  const output=page.elements.get('d0'), writes=output.textWrites;
  for (let sequence=1;sequence<=40;sequence++)
    socket.message({v:1,type:'telemetry',bridge_id:'coalesce',stream:'fusion',epoch:1,valid:true,
      data:{type:'fusion',sequence,flags:1,updates:sequence}});
  assert.equal(output.textWrites,writes,'individual messages do not redraw the diagnostic display');
  page.runIntervals(250);
  assert.equal(output.textWrites,writes+1);
  assert.equal(output.textContent,'40','the presentation uses the latest accepted sample');
  assert.equal(socket.sent.some(item=>['arm','local_task','mission_command'].includes(item.type)),false);
});

test('map close and silent streams reconnect without reviving historical data', () => {
  MockWebSocket.instances=[];
  const page=startDashboard({protocol:'http:'});
  const ws=MockWebSocket.instances.find(socket=>socket.url.endsWith('/ws'));
  let map=MockWebSocket.instances.find(socket=>socket.url.endsWith('/map'));
  page.setWallClock(10000);
  ws.open(); map.open();
  ws.message({v:1,type:'hello',client_id:'map-reconnect',simulation:false,limits:{speed:100,turn:300}});
  const nav={boot_id:7,odom_domain:3,age_ms:0,stale:false,task_id_watermark_known:true,
    last_task_id:0,task_id:0,status:0,mode:1,active_task:false};
  const state=seq=>ws.message({v:1,type:'state',owner:null,armed:false,jetson:{online:true},fpga:{ready:true},
    status:{bridge_id:'map-reconnect',seq,emergency_stop:false},navigation:nav,video:{}});
  state(1); state(2);
  const snapshot=fpgaMapFixture();
  map.message(snapshot);
  const canvas=page.elements.get('radarCloud');
  canvas.dispatch('pointerdown',{button:0,pointerId:1,clientX:220,clientY:150});
  canvas.dispatch('pointerup',{pointerId:1,clientX:220,clientY:150});
  assert.equal(page.elements.get('localTaskStart').disabled,false);
  const old=map;
  old.close(); old.onclose?.(); old.onerror?.();
  assert.equal(page.pendingTimeouts(250),1,'one reconnect timer for repeated closure events');
  assert.match(page.elements.get('mapStatus').textContent,/连接已断开.*历史地图/);
  assert.equal(page.elements.get('localTaskStart').disabled,false);
  page.runTimeouts(250);
  map=MockWebSocket.latest;
  assert.notEqual(map,old);
  map.open();
  assert.match(page.elements.get('mapStatus').textContent,/等待新数据.*历史地图/);
  assert.equal(page.elements.get('localTaskStart').disabled,false,'a historical draft is still submittable; Jetson checks readiness');
  old.message({...snapshot,domain:'other:1',generation:99});
  assert.equal(canvas.dataset.generation,'1','a retired socket cannot overwrite the map');
  map.message(snapshot);
  assert.equal(page.elements.get('localTaskStart').disabled,false,'matching fresh data restores the selected goal');
  assert.equal(ws.sent.some(item=>item.type==='local_task'),false,'reconnection never sends motion');

  page.setWallClock(11501);
  map.message(snapshot);
  assert.equal(map.readyState,3,'a queued frame arriving after the 1.5s silence is discarded');
  assert.match(page.elements.get('mapStatus').textContent,/地图数据停更.*历史地图/);
  assert.equal(page.elements.get('localTaskStart').disabled,false);
  assert.equal(page.pendingTimeouts(250),1);
  page.runTimeouts(250);
  map=MockWebSocket.latest; map.open();
  assert.equal(page.elements.get('localTaskStart').disabled,false);
  map.message({...snapshot,revision:2});
  assert.equal(page.elements.get('localTaskStart').disabled,false);
  assert.equal(ws.sent.some(item=>item.type==='local_task'),false);
  page.setWallClock(13002);
  page.runTimeouts(1500);
  assert.equal(map.readyState,3,'the watchdog reconnects even if no more messages arrive');
  assert.match(page.elements.get('mapStatus').textContent,/地图数据停更.*历史地图/);
  assert.equal(page.pendingTimeouts(250),1);
  page.runTimeouts(250); map=MockWebSocket.latest; map.open();
  map.message({...snapshot,revision:3});
  assert.equal(page.elements.get('localTaskStart').disabled,false);
  page.elements.get('networkDisconnect').dispatch('click');
  assert.equal(map.readyState,3,'explicit dashboard disconnect closes the map');
  assert.equal(page.pendingTimeouts(250),0,'explicit dashboard disconnect cancels retries');
});

test('historical map goals keep their own domain and Jetson reports a mismatch', () => {
  MockWebSocket.instances=[];
  const page=startDashboard({protocol:'http:'});
  const ws=MockWebSocket.instances.find(socket=>socket.url.endsWith('/ws'));
  const map=MockWebSocket.instances.find(socket=>socket.url.endsWith('/map'));
  page.setWallClock(20000);
  ws.open(); map.open();
  ws.message({v:1,type:'hello',client_id:'map-domain',simulation:false,limits:{speed:100,turn:300}});
  const nav={boot_id:7,odom_domain:3,age_ms:0,stale:false,task_id_watermark_known:true,
    last_task_id:0,task_id:0,status:0,mode:1,active_task:false};
  const state=(seq,navigation)=>ws.message({v:1,type:'state',owner:null,armed:false,jetson:{online:true},fpga:{ready:true},
    status:{bridge_id:'map-domain',seq,emergency_stop:false},navigation,video:{}});
  state(1,nav); state(2,nav);
  const snapshot=fpgaMapFixture({domain:'7:4',status:'paused',pose_fresh:false,pose_age_ms:501});
  const canvas=page.elements.get('radarCloud');
  const click=x=>{canvas.dispatch('pointerdown',{button:0,pointerId:1,clientX:x,clientY:150});
    canvas.dispatch('pointerup',{pointerId:1,clientX:x,clientY:150});};
  map.message(snapshot); click(220);
  assert.match(page.elements.get('mapStatus').textContent,/位姿.*过期/);
  assert.match(page.elements.get('mapGoalHint').textContent,/已选 1 个航点/);
  assert.equal(page.elements.get('localTaskStart').disabled,false);
  page.elements.get('localTaskStart').dispatch('click');
  const request=ws.sent.find(item=>item.type==='navigation_task' && item.action==='start');
  assert.equal(request.domain,'7:4');
  ws.message({v:1,type:'mission_result',action:'navigation_start',success:false,message:'导航坐标域不匹配'});
  assert.match(page.elements.get('localTaskResult').textContent,/导航坐标域不匹配/);
  assert.match(page.elements.get('mapGoalHint').textContent,/已选 1 个航点/,'rejected target remains editable');
  state(3,{...nav,odom_domain:4});
  map.message({...snapshot,domain:'7:5'}); click(260);
  assert.match(page.elements.get('mapGoalHint').textContent,/同一坐标域/,'different coordinate domains cannot be mixed in one queue');
  assert.equal(ws.sent.filter(item=>item.type==='navigation_task').length,1);
});

test('map reconnection backs off and explicit disconnect or service switch cancels old retries', () => {
  MockWebSocket.instances=[];
  const page=startDashboard({protocol:'file:'});
  const view=page.context.BX72OccupancyMap.createView({document:page.document,window:page.window,WebSocketImpl:MockWebSocket});
  view.connect('ws://one/map');
  const initial=MockWebSocket.latest;
  page.runTimeouts(4000);
  assert.equal(initial.readyState,3,'a stuck connection attempt eventually closes');
  assert.equal(page.pendingTimeouts(250),1);
  page.runTimeouts(250);
  for (const delay of [500,1000,2000,4000,4000]) {
    const current=MockWebSocket.latest;
    current.close();
    assert.equal(page.pendingTimeouts(delay),1,`one bounded retry at ${delay}ms`);
    page.runTimeouts(delay);
    assert.notEqual(MockWebSocket.latest,current);
  }
  const previous=MockWebSocket.latest;
  previous.close();
  assert.equal(page.pendingTimeouts(4000),1);
  view.connect('ws://two/map');
  const next=MockWebSocket.latest;
  assert.equal(next.url,'ws://two/map');
  assert.equal(page.pendingTimeouts(4000),1,'only the new connection watchdog remains');
  previous.onclose?.(); previous.onerror?.();
  next.open();
  view.disconnect();
  next.onclose?.(); next.onerror?.();
  const count=MockWebSocket.instances.length;
  for (const delay of [250,500,1000,1500,2000,4000]) page.runTimeouts(delay);
  assert.equal(MockWebSocket.instances.length,count,'old timers and callbacks cannot reconnect after explicit disconnect');
  assert.match(page.elements.get('mapStatus').textContent,/连接已断开/);
});

test('all map views add to one map-frame queue and only the service advances it', () => {
  MockWebSocket.instances=[];
  let deliverGlobal;
  const globalMapSource={createSource:({onUpdate})=>{deliverGlobal=onUpdate;return {start(){}};}};
  const page=startDashboard({protocol:'http:',globalMapSource});
  const ws=MockWebSocket.instances.find(socket=>socket.url.endsWith('/ws'));
  const map=MockWebSocket.instances.find(socket=>socket.url.endsWith('/map'));
  ws.open(); map.open();
  ws.message({v:1,type:'hello',client_id:'waypoints',simulation:true,limits:{speed:100,turn:300}});
  const nav={protocol:'nav_grid_v1',boot_id:7,odom_domain:3,age_ms:0,stale:false,
    task_id_watermark_known:true,last_task_id:0,task_id:0,status:0,mode:1,active_task:false,
    goal_x_q16:0,goal_y_q16:0};
  let seq=0;
  const state=(navigation=nav,navigation_task=null)=>ws.message({v:1,type:'state',owner:null,armed:false,
    jetson:{online:true},fpga:{ready:true},status:{bridge_id:'waypoints',seq:seq++,emergency_stop:false},
    navigation,navigation_task,video:{}});
  state();state();map.message(fpgaMapFixture());
  const global={available:true,status:'building',reason:'SLAM',domain:'7:3',generation:'g',revision:1,
    width:40,height:40,resolution_m:.2,origin:{x:-4,y:-4},counts:{free:1600,occupied:0},map_age_ms:0,
    map_from_odom:{x:0,y:0,yaw:0},map_from_odom_age_ms:0};
  deliverGlobal({online:true,cells:vm.runInContext('new Uint8Array(1600)',page.context),map:global});
  const canvas=page.elements.get('radarCloud');
  const click=x=>{canvas.dispatch('pointerdown',{button:0,pointerId:1,clientX:x,clientY:150});
    canvas.dispatch('pointerup',{pointerId:1,clientX:x,clientY:150});};
  click(220); click(300);
  page.elements.get('mapLayerToggle').dispatch('click');
  click(240);
  assert.match(page.elements.get('mapGoalHint').textContent,/已选 3 个航点 · map/);
  assert.equal(page.elements.get('waypointList').children.length,3);
  assert.equal(page.elements.get('localTaskStart').disabled,false);
  assert.equal(page.elements.has('globalTaskStart'),false);
  assert.equal(page.elements.has('localMode'),false);
  page.elements.get('localTaskStart').dispatch('click');
  const tasks=()=>ws.sent.filter(item=>item.type==='navigation_task');
  assert.equal(tasks().length,1);
  const start=tasks()[0];
  assert.equal(start.action,'start');assert.equal(start.frame,'map');
  assert.equal(start.domain,'7:3');assert.equal(start.generation,'g');assert.equal(start.goals.length,3);
  assert.ok(start.goals[0].x<1.6 && start.goals[1].x>1.6,'the blue boundary does not choose a task type');
  assert.equal('hold_yaw_rad' in start,false);
  const task={id:'one',frame:'map',status:'following',active:true,goal_index:0,goal_count:3,
    goals:start.goals,route:[{x:0,y:0},start.goals[0]],message:'前往航点',
    domain:'7:3',generation:'g',replans:1};
  state({...nav,task_id:5,last_task_id:5,status:4},task);
  assert.equal(page.elements.get('localTaskCancel').disabled,false,'stop remains available if active state arrives before the start response');
  ws.message({v:1,type:'mission_result',action:'navigation_start',success:true,navigation_id:'one',message:'已接受'});
  state({...nav,task_id:5,last_task_id:5,status:4},{...task,goal_index:1});
  assert.equal(tasks().length,1,'local arrivals never send another waypoint from the page');
  assert.match(page.elements.get('localTaskState').textContent,/执行中 · 航点 2\/3/);
  assert.match(page.elements.get('waypointList').children[1].textContent,/当前/);
  assert.equal(page.elements.get('localTaskStart').disabled,true);
  assert.equal(page.elements.get('localTaskCancel').disabled,false);
  page.elements.get('localTaskCancel').dispatch('click');
  assert.equal(tasks().at(-1).action,'cancel');
  ws.message({v:1,type:'mission_result',action:'navigation_cancel',success:true,message:'已请求取消'});
  state(nav,{...task,status:'cancelled',active:false,route:[]});
  assert.match(page.elements.get('localTaskState').textContent,/已取消/);
  assert.equal(page.elements.get('localTaskCancel').disabled,false,'an explicit cancellation remains available even when telemetry reports idle');
});

for (const view of ['overlay','global']) test(`${view} view selects front, back, left and right beyond the known global map`, () => {
  MockWebSocket.instances=[];
  let deliverGlobal;
  const globalMapSource={createSource:({onUpdate})=>{deliverGlobal=onUpdate;return {start(){}};}};
  const page=startDashboard({protocol:'http:',globalMapSource});
  const ws=MockWebSocket.instances.find(socket=>socket.url.endsWith('/ws'));
  const map=MockWebSocket.instances.find(socket=>socket.url.endsWith('/map'));
  ws.open(); map.open();
  ws.message({v:1,type:'hello',client_id:'all-directions',simulation:true,limits:{speed:100,turn:300}});
  const nav={protocol:'nav_grid_v1',boot_id:7,odom_domain:3,age_ms:0,stale:false,
    task_id_watermark_known:true,last_task_id:0,task_id:0,status:0,mode:1,active_task:false,
    goal_x_q16:0,goal_y_q16:0};
  const state=seq=>ws.message({v:1,type:'state',owner:null,armed:false,jetson:{online:true},fpga:{ready:true},
    status:{bridge_id:'all-directions',seq,emergency_stop:false},navigation:nav,video:{}});
  state(1);state(2);
  map.message(fpgaMapFixture({cells_rle:[0,4096],counts:{free:0,occupied:0}}));
  const global={available:true,status:'building',domain:'7:3',generation:'g',revision:1,
    width:10,height:10,resolution_m:.1,origin:{x:0,y:0},counts:{free:0,occupied:0},map_age_ms:0,
    map_from_odom:{x:.5,y:-.3,yaw:Math.PI/2},map_from_odom_age_ms:0};
  deliverGlobal({online:true,cells:vm.runInContext('new Uint8Array(100)',page.context),map:global});
  if (view === 'global') page.elements.get('mapLayerToggle').dispatch('click');
  const canvas=page.elements.get('radarCloud');
  const click=(x,y)=>{canvas.dispatch('pointerdown',{button:0,pointerId:1,clientX:x,clientY:y});
    canvas.dispatch('pointerup',{pointerId:1,clientX:x,clientY:y});};
  const pixels=view === 'overlay' ? [[240,150],[160,150],[200,110],[200,190]]
    : [[200,172.5],[200,247.5],[162.5,210],[237.5,210]];
  for (const [x,y] of pixels) click(x,y);
  assert.equal(page.elements.get('waypointList').children.length,4,
    'an existing map queue accepts a back point even outside global bounds and in unknown space');
  assert.equal(page.elements.get('localTaskStart').disabled,false);
  page.elements.get('localTaskStart').dispatch('click');
  const request=ws.sent.find(item=>item.type==='navigation_task' && item.action==='start');
  assert.equal(request.frame,'map'); assert.equal(request.generation,'g');
  const distance=view === 'overlay' ? .533 : .5;
  const rounded=value=>Number(value.toFixed(3));
  assert.deepEqual(request.goals,[{x:.5,y:rounded(-.3+distance)},{x:.5,y:rounded(-.3-distance)},
    {x:rounded(.5-distance),y:-.3},{x:rounded(.5+distance),y:-.3}]);
});

test('local-only view accepts visible points outside the local grid as odom waypoints', () => {
  MockWebSocket.instances=[];
  const page=startDashboard({protocol:'http:'});
  const ws=MockWebSocket.instances.find(socket=>socket.url.endsWith('/ws'));
  const map=MockWebSocket.instances.find(socket=>socket.url.endsWith('/map'));
  ws.open(); map.open();
  ws.message({v:1,type:'hello',client_id:'all-directions-odom',simulation:true,limits:{speed:100,turn:300}});
  const nav={protocol:'nav_grid_v1',boot_id:7,odom_domain:3,age_ms:0,stale:false,
    task_id_watermark_known:true,last_task_id:0,task_id:0,status:0,mode:1,active_task:false,
    goal_x_q16:0,goal_y_q16:0};
  const state=seq=>ws.message({v:1,type:'state',owner:null,armed:false,jetson:{online:true},fpga:{ready:true},
    status:{bridge_id:'all-directions-odom',seq,emergency_stop:false},navigation:nav,video:{}});
  state(1);state(2);map.message(fpgaMapFixture({cells_rle:[0,4096],counts:{free:0,occupied:0}}));
  const canvas=page.elements.get('radarCloud');
  for (const [x,y] of [[360,150],[40,150],[200,20],[200,280]]) {
    canvas.dispatch('pointerdown',{button:0,pointerId:1,clientX:x,clientY:y});
    canvas.dispatch('pointerup',{pointerId:1,clientX:x,clientY:y});
  }
  assert.equal(page.elements.get('waypointList').children.length,4);
  assert.equal(page.elements.get('localTaskStart').disabled,false);
  page.elements.get('localTaskStart').dispatch('click');
  const request=ws.sent.find(item=>item.type==='navigation_task' && item.action==='start');
  assert.equal(request.frame,'odom'); assert.equal(request.generation,null);
  assert.deepEqual(request.goals,[{x:2.133,y:0},{x:-2.133,y:0},{x:0,y:1.733},{x:0,y:-1.733}]);
});

test('map-frame draft waits for its transform and is discarded on map generation change', () => {
  MockWebSocket.instances=[];
  let deliverGlobal;
  const globalMapSource={createSource:({onUpdate})=>{deliverGlobal=onUpdate;return {start(){}};}};
  const page=startDashboard({protocol:'http:',globalMapSource});
  page.window.performance=page.context.performance;
  const ws=MockWebSocket.instances.find(socket=>socket.url.endsWith('/ws'));
  const map=MockWebSocket.instances.find(socket=>socket.url.endsWith('/map'));
  ws.open(); map.open();
  ws.message({v:1,type:'hello',client_id:'map-frame',simulation:true,limits:{speed:100,turn:300}});
  const nav={protocol:'nav_grid_v1',boot_id:7,odom_domain:3,age_ms:0,stale:false,
    task_id_watermark_known:true,last_task_id:0,task_id:0,status:0,mode:1,active_task:false,
    goal_x_q16:0,goal_y_q16:0};
  let seq=0;
  const state=()=>ws.message({v:1,type:'state',owner:null,armed:false,jetson:{online:true},fpga:{ready:true},
    status:{bridge_id:'map-frame',seq:seq++,emergency_stop:false},navigation:nav,video:{}});
  state();state();
  const snapshot=fpgaMapFixture();map.message(snapshot);
  const global={available:true,status:'building',reason:'SLAM',domain:'7:3',generation:'g',revision:1,
    width:40,height:40,resolution_m:.2,origin:{x:-4,y:-4},counts:{free:1600,occupied:0},map_age_ms:0,
    map_from_odom:{x:0,y:0,yaw:0},map_from_odom_age_ms:0};
  const publish=value=>deliverGlobal({online:true,cells:vm.runInContext('new Uint8Array(1600)',page.context),map:value});
  publish(global);
  const canvas=page.elements.get('radarCloud');
  const click=x=>{canvas.dispatch('pointerdown',{button:0,pointerId:1,clientX:x,clientY:150});
    canvas.dispatch('pointerup',{pointerId:1,clientX:x,clientY:150});};
  click(220);
  assert.match(page.elements.get('mapGoalHint').textContent,/已选 1 个航点 · map/);
  page.advancePerf(751);map.message({...snapshot,revision:2});state();
  click(240);
  assert.match(page.elements.get('mapGoalHint').textContent,/等待同一坐标域和地图代次的坐标变换/);
  assert.equal(page.elements.get('localTaskStart').disabled,false,'the existing map-frame goal needs no transform to submit');
  publish(global);click(240);
  assert.match(page.elements.get('mapGoalHint').textContent,/已选 2 个航点 · map/);
  publish({...global,generation:'new'});
  assert.match(page.elements.get('mapGoalHint').textContent,/代次变化/);
  assert.equal(page.elements.get('localTaskStart').disabled,true);
});

test('grid freshness uses monotonic time and odom goals do not depend on grid generation', () => {
  MockWebSocket.instances=[];
  const page=startDashboard({protocol:'http:'});
  page.window.performance=page.context.performance;
  page.setWallClock(10000);
  const view=page.context.BX72OccupancyMap.createView({document:page.document,window:page.window,WebSocketImpl:MockWebSocket});
  view.connect('ws://isolated/map');
  const map=MockWebSocket.latest;map.open();map.message(fpgaMapFixture());
  view.setNavigation({frame:'odom',domain:'7:3',generation:null,goals:[{x:.5,y:0}],route:[]});
  map.message({v:1,type:'mapping',protocol:'fpga_grid_view_v1',source:'fpga',available:false,
    domain:'7:3',reason:'等待快照'});
  assert.equal(view.getInfo().fresh,false);
  map.message(fpgaMapFixture({generation:2}));
  assert.equal(view.getInfo().fresh,true);
  page.setWallClock(1);page.advancePerf(751);
  assert.equal(view.getInfo().fresh,false,'clock rollback does not renew an old snapshot');
});

test('waypoint submission reaches Jetson despite stale or emergency display state', () => {
  MockWebSocket.instances=[];
  const page=startDashboard({protocol:'http:'});
  const ws=MockWebSocket.instances.find(socket=>socket.url.endsWith('/ws'));
  const map=MockWebSocket.instances.find(socket=>socket.url.endsWith('/map'));
  ws.open(); map.open();
  ws.message({v:1,type:'hello',client_id:'waypoints-stop',simulation:false,limits:{speed:100,turn:300}});
  const nav={protocol:'nav_grid_v1',boot_id:7,odom_domain:3,age_ms:0,stale:false,
    task_id_watermark_known:true,last_task_id:0,task_id:0,status:0,mode:1,active_task:false,
    goal_x_q16:0,goal_y_q16:0};
  const state=(seq,navigation,emergency_stop=false,navigation_task=null)=>ws.message({v:1,type:'state',owner:null,armed:false,
    jetson:{online:true},fpga:{ready:true},status:{bridge_id:'waypoints-stop',seq,emergency_stop,control_source:'none'},navigation,navigation_task,video:{}});
  state(1,nav);state(2,nav);
  const snapshot=fpgaMapFixture();map.message(snapshot);
  const canvas=page.elements.get('radarCloud');
  const click=x=>{canvas.dispatch('pointerdown',{button:0,pointerId:1,clientX:x,clientY:150});
    canvas.dispatch('pointerup',{pointerId:1,clientX:x,clientY:150});};
  click(220);
  page.advancePerf(501);map.message({...snapshot,revision:2});
  const arrived={...nav,task_id:1,last_task_id:1,status:4,error:77,guard_error:13};
  state(2,arrived);
  assert.equal(page.elements.get('localTaskStart').disabled,false,'stale display state cannot veto forwarding');
  assert.match(page.elements.get('localTaskState').textContent,/保护原因 软件停车（13）.*原始错误 77/);
  const task={id:'other',active:true,status:'following',goal_count:1,goal_index:0,goals:[{x:3,y:0}],route:[],frame:'odom',domain:'7:3',generation:null};
  state(3,arrived,true,task);
  assert.equal(page.elements.get('localTaskStart').disabled,false,'Jetson decides whether an active task or emergency permits start');
  page.elements.get('localTaskStart').dispatch('click');
  assert.equal(ws.sent.filter(item=>item.type==='navigation_task'&&item.action==='start').length,1);
  assert.equal(page.elements.get('localTaskCancel').disabled,false,'cancel remains available while start is pending');
  state(4,arrived,true,task);
  assert.match(page.elements.get('mapGoalHint').textContent,/已选 1 个航点/,'an unrelated active state cannot clear the draft');
  ws.message({v:1,type:'mission_result',action:'navigation_start',success:false,message:'软件急停已激活'});
  assert.match(page.elements.get('localTaskResult').textContent,/软件急停已激活/);
  assert.match(page.elements.get('mapGoalHint').textContent,/已选 1 个航点/);
  page.elements.get('localTaskStart').dispatch('click');
  click(260);
  ws.message({v:1,type:'mission_result',action:'navigation_start',success:true,message:'目标已接收'});
  assert.match(page.elements.get('mapGoalHint').textContent,/已选 2 个航点/,'a delayed result cannot clear edits made after submission');
});

test('a map-frame goal needs map identity but no fresh map-to-odom transform', () => {
  MockWebSocket.instances=[];let deliverGlobal;
  const globalMapSource={createSource:({onUpdate})=>{deliverGlobal=onUpdate;return {start(){}};}};
  const page=startDashboard({protocol:'http:',globalMapSource});
  const ws=MockWebSocket.instances.find(socket=>socket.url.endsWith('/ws'));
  ws.open();
  ws.message({v:1,type:'hello',client_id:'map-no-transform',simulation:false,limits:{speed:100,turn:300}});
  const global={available:true,status:'paused',reason:'历史全局地图',domain:'7:3',generation:'g',revision:1,
    width:40,height:40,resolution_m:.2,origin:{x:-4,y:-4},counts:{free:1600,occupied:0},map_from_odom:null};
  deliverGlobal({online:true,cells:vm.runInContext('new Uint8Array(1600)',page.context),map:global});
  const canvas=page.elements.get('radarCloud');
  canvas.dispatch('pointerdown',{button:0,pointerId:1,clientX:240,clientY:150});
  canvas.dispatch('pointerup',{pointerId:1,clientX:240,clientY:150});
  assert.match(page.elements.get('mapGoalHint').textContent,/已选 1 个航点 · map/);
  assert.equal(page.elements.get('localTaskStart').disabled,false);
  page.elements.get('localTaskStart').dispatch('click');
  const task=ws.sent.find(item=>item.type==='navigation_task');
  assert.equal(task.frame,'map');assert.equal(task.domain,'7:3');assert.equal(task.generation,'g');
});

test('selected detection starts the requested vision behavior', () => {
  MockWebSocket.instances=[];
  const page=startDashboard({protocol:'http:'});
  const ws=MockWebSocket.instances.find(socket=>socket.url.endsWith('/ws'));
  const video=MockWebSocket.instances.find(socket=>socket.url.endsWith('/video'));
  ws.open(); video.open();
  ws.message({v:1,type:'hello',client_id:'vision-test',simulation:true,limits:{speed:100,turn:300}});
  const box={id:'track:17',class_id:'0',score:0.9,cx:400,cy:240,width:100,height:120};
  const visionState={v:1,type:'state',owner:null,armed:false,jetson:{online:true},fpga:{ready:true},
    status:{bridge_id:'vision-test',seq:1,emergency_stop:false},video:{},vision:{age_ms:1,stale:false,mission_state:'IDLE',
      control_source:'ros',ros_authorized:true,emergency_stop:false,width:800,height:480,
      detections:[box],selected_target:null}};
  ws.message(visionState);
  ws.message({...visionState,status:{...visionState.status,seq:2}});
  const image=page.elements.get('networkVideo');
  image.complete=true; image.naturalWidth=800; image.naturalHeight=480;
  video.onmessage({data:vm.runInContext('new Uint8Array([255,216,255,217]).buffer',page.context)});
  image.onload();
  image.complete=false; // The next JPEG can be decoding while the last one is still visible.
  const before=ws.sent.length;
  page.elements.get('visionOverlay').dispatch('click',{clientX:200,clientY:150});
  assert.match(page.elements.get('visionDraft').textContent,/track:17/);
  assert.equal(page.elements.get('visionStart').disabled,false);
  page.elements.get('visionBehavior').value='approach';
  page.elements.get('visionColor').value='red';
  page.elements.get('visionStart').dispatch('click');
  assert.deepEqual({...ws.sent.at(-1)}, {v:1,type:'mission_command',action:'start',
    target_id:'track:17',class_id:'0',behavior:'approach',color:'red'});
  image.naturalWidth=640; image.naturalHeight=360; image.onload();
  assert.match(page.elements.get('visionTarget').textContent,/尺寸不一致/);
  assert.equal(page.elements.get('visionDraftSelect').disabled,true);
});

for (const behavior of ['search', 'search_only']) test(`${behavior} may start by class without a detected target`, () => {
  MockWebSocket.instances=[];
  const page=startDashboard({protocol:'http:'});
  const ws=MockWebSocket.instances.find(socket=>socket.url.endsWith('/ws'));
  ws.open(); ws.message({v:1,type:'hello',client_id:'search',simulation:true,limits:{speed:100,turn:300}});
  const visionState={v:1,type:'state',owner:null,armed:false,jetson:{online:true},fpga:{ready:true},
    status:{bridge_id:'search',seq:1,emergency_stop:false},video:{},vision:{age_ms:0,stale:false,mission_state:'IDLE',
      control_source:'none',ros_authorized:false,emergency_stop:false,width:800,height:480,detections:[],selected_target:null}};
  ws.message(visionState);
  ws.message({...visionState,status:{...visionState.status,seq:2}});
  page.elements.get('visionBehavior').value=behavior;
  page.elements.get('visionClass').value='39';
  page.elements.get('visionBehavior').dispatch('change');
  assert.equal(page.elements.get('visionStart').disabled,false);
  page.elements.get('visionStart').dispatch('click');
  assert.deepEqual({...ws.sent.at(-1)}, {v:1,type:'mission_command',action:'start',target_id:'',class_id:'39',behavior,color:''});
});

test('visual start requires a fresh idle or stopped task and rejects active or unknown states', async () => {
  for (const missionState of ['TRACKING','FOLLOWING','APPROACHING','SEARCHING','FOUND','BLOCKED','LOST','ARRIVED','UNKNOWN']) {
    const page=startDashboard();
    const socket=await openNetworkService(page,{clientId:'visual-state'});
    const vision={age_ms:1,stale:false,mission_state:missionState,control_source:'ros',
      ros_authorized:true,emergency_stop:false,width:800,height:480,detections:[],selected_target:null};
    const state={v:1,type:'state',owner:null,armed:false,simulation:true,
      jetson:{online:true,age_ms:1},fpga:{ready:true,ack_age_ms:1},
      status:{bridge_id:'visual-state-bridge',seq:1,emergency_stop:false},video:{},vision};
    socket.message(state);
    socket.message({...state,status:{...state.status,seq:2}});
    page.elements.get('visionBehavior').value='search';
    page.elements.get('visionClass').value='39';
    page.elements.get('visionBehavior').dispatch('change');
    assert.equal(page.elements.get('visionStart').disabled,true,missionState);
    const before=socket.sent.length;
    page.elements.get('visionStart').dispatch('click');
    assert.equal(socket.sent.length,before,`${missionState} must not submit a start even if the handler is called`);
    assert.match(page.elements.get('visionResult').textContent,/视觉任务.*未确认/);
    page.elements.get('visionStop').dispatch('click');
    assert.equal(socket.sent.at(-1).action,'stop',`${missionState} must permit an explicit stop`);
    page.elements.get('visionEstop').dispatch('click');
    assert.equal(socket.sent.at(-1).action,'emergency_stop',`${missionState} must permit an explicit emergency stop`);
  }
  for (const missionState of ['IDLE','STOPPED']) {
    const page=startDashboard();
    const socket=await openNetworkService(page,{clientId:'visual-ready'});
    const vision={age_ms:1,stale:false,mission_state:missionState,control_source:'none',
      ros_authorized:false,emergency_stop:false,width:800,height:480,detections:[],selected_target:null};
    const state={v:1,type:'state',owner:null,armed:false,simulation:true,
      jetson:{online:true,age_ms:1},fpga:{ready:true,ack_age_ms:1},
      status:{bridge_id:'visual-ready-bridge',seq:1,emergency_stop:false},video:{},vision};
    socket.message(state);
    socket.message({...state,status:{...state.status,seq:2}});
    page.elements.get('visionBehavior').value='search';
    page.elements.get('visionClass').value='39';
    page.elements.get('visionBehavior').dispatch('change');
    assert.equal(page.elements.get('visionStart').disabled,false,missionState);
    page.elements.get('visionStart').dispatch('click');
    assert.equal(socket.sent.at(-1).action,'start',missionState);
    const before=socket.sent.length;
    page.elements.get('visionStart').dispatch('click');
    assert.equal(socket.sent.length,before,'a pending start must not be sent twice');
  }
});

test('direct visual start releases confirmed manual control after validation and never re-arms on focus', async () => {
  const page=startDashboard();
  const socket=await openNetworkService(page,{clientId:'handoff-controller'});
  const vision={age_ms:1,stale:false,mission_state:'IDLE',control_source:'none',
    ros_authorized:false,emergency_stop:false,width:800,height:480,detections:[],selected_target:null};
  const state={v:1,type:'state',owner:null,armed:false,simulation:true,
    jetson:{online:true,age_ms:1},fpga:{ready:true,ack_age_ms:1},
    status:{bridge_id:'handoff-bridge',seq:1,emergency_stop:false},video:{},vision};
  socket.message(state);
  socket.message({...state,status:{...state.status,seq:2}});
  page.elements.get('networkArm').dispatch('click');
  socket.message({...state,owner:'handoff-controller',armed:true,status:{...state.status,seq:3}});
  page.document.dispatch('keydown',{code:'KeyW',preventDefault(){}});
  assert.ok(socket.sent.some(message=>message.type==='drive' && message.vx===100));
  page.elements.get('visionBehavior').value='approach';
  const beforeInvalid=socket.sent.length;
  page.elements.get('visionStart').dispatch('click');
  assert.equal(socket.sent.length,beforeInvalid,'a missing target does not release manual control');
  page.elements.get('visionBehavior').value='search';
  page.elements.get('visionClass').value='39';
  page.elements.get('visionBehavior').dispatch('change');
  page.elements.get('visionStart').dispatch('click');
  assert.deepEqual(socket.sent.slice(beforeInvalid),[
    {v:1,type:'stop',reason:'切换到 ROS 视觉转向'},
    {v:1,type:'mission_command',action:'start',target_id:'',class_id:'39',behavior:'search',color:''}
  ]);
  socket.message({v:1,type:'mission_result',action:'start',success:true,message:'已接收'});
  socket.message({...state,status:{...state.status,seq:4},
    vision:{...vision,mission_state:'SEARCHING',control_source:'ros',ros_authorized:true}});
  const afterStart=socket.sent.length;
  page.window.dispatch('blur');
  page.document.hidden=true; page.document.dispatch('visibilitychange');
  page.document.hidden=false; page.document.dispatch('visibilitychange');
  page.window.focus(true); page.window.dispatch('focus');
  page.runIntervals(50); page.runIntervals(250);
  page.document.dispatch('keydown',{code:'KeyW',preventDefault(){}});
  assert.equal(socket.sent.length,afterStart,'focus events must not stop vision or restore manual motion');
});

test('emergency state follows advancing bridge status independently of vision freshness', () => {
  MockWebSocket.instances=[];
  const page=startDashboard({protocol:'http:'});
  const ws=MockWebSocket.instances.find(socket=>socket.url.endsWith('/ws'));
  ws.open(); ws.message({v:1,type:'hello',client_id:'emergency',simulation:false,limits:{speed:100,turn:300}});
  const vision={age_ms:0,stale:true,mission_state:'IDLE',control_source:'none',
    ros_authorized:false,emergency_stop:false,width:800,height:480,detections:[],selected_target:null};
  const base={v:1,type:'state',owner:null,armed:false,jetson:{online:true,age_ms:0},
    fpga:{ready:true},video:{},vision};
  const send=(seq,updates={},currentVision=vision)=>ws.message({...base,vision:currentVision,
    status:{bridge_id:'actual-bridge',session:'session-1',seq,emergency_stop:false,...updates}});
  page.elements.get('visionBehavior').value='search';
  send(0xfffffffe);
  assert.match(page.elements.get('visionStatus').textContent,/软件急停：未知/,'a cached first status has unknown age');
  send(0xffffffff);
  assert.match(page.elements.get('visionStatus').textContent,/视觉观测已过期.*软件急停：未激活/);
  assert.equal(page.elements.get('visionStart').disabled,true,'stale visual observations still block vision starts');
  send(0xffffffff,{}, {...vision,stale:false});
  assert.equal(page.elements.get('visionStart').disabled,false);
  page.advancePerf(501);
  send(0xffffffff,{}, {...vision,stale:false});
  assert.match(page.elements.get('visionStatus').textContent,/软件急停：未知/,
    'fresh service and vision messages cannot refresh an unchanged status sequence');
  assert.equal(page.elements.get('visionStart').disabled,true);
  const sentBefore=ws.sent.length;
  page.elements.get('visionStart').dispatch('click');
  assert.equal(ws.sent.length,sentBefore,'an unknown bridge stop state cannot send a start request');
  send(0,{emergency_stop:true}, {...vision,stale:false});
  assert.match(page.elements.get('visionStatus').textContent,/软件急停：已激活/);
  assert.equal(page.elements.get('visionStart').disabled,true,'sequence wrap preserves the active stop');
  send(1,{emergency_stop:false,control_source:'emergency_stop'}, {...vision,stale:false});
  assert.match(page.elements.get('visionStatus').textContent,/软件急停：未知.*反馈不一致/,'contradictory evidence remains unknown');
  assert.equal(page.elements.get('visionStart').disabled,true,'contradictory evidence still blocks starting');
  send(2,{emergency_stop:undefined,control_source:'none'}, {...vision,stale:false});
  assert.match(page.elements.get('visionStatus').textContent,/软件急停：未知/,
    'real bridge control_source cannot prove the stop is inactive');
  assert.equal(page.elements.get('visionStart').disabled,true);
  send(3,{emergency_stop:false},null);
  assert.match(page.elements.get('visionStatus').textContent,/视觉任务接口未连接.*软件急停：未激活/);
  assert.equal(page.elements.get('visionStop').disabled,false,'an explicit stop does not require a vision observation');
  page.elements.get('visionStop').dispatch('click');
  assert.deepEqual({...ws.sent.at(-1)},{v:1,type:'mission_command',action:'stop'});
  ws.message({v:1,type:'mission_result',action:'stop',success:true,message:'已响应'});
  ws.message({...base,vision:{...vision,stale:false},fpga:{ready:true,ack_age_ms:710},
    status:{bridge_id:'actual-bridge',session:'session-1',seq:4,emergency_stop:false,fpga_ack_age:0.01}});
  assert.match(page.elements.get('visionStatus').textContent,/软件急停：未知/,
    'an advancing status with a service-reported age over 500 ms is not fresh');
  assert.equal(page.elements.get('visionStart').disabled,true);
  ws.message({...base,vision:{...vision,emergency_stop:true},fpga:{ready:true,ack_age_ms:12},
    status:{bridge_id:'actual-bridge',session:'session-1',seq:5,emergency_stop:false,fpga_ack_age:0.01}});
  assert.match(page.elements.get('visionStatus').textContent,/软件急停：未激活/);
  assert.equal(page.elements.get('visionEnable').disabled,false,'a stale vision copy does not override the fresh gateway stop state');
  ws.close();
  assert.match(page.elements.get('visionStatus').textContent,/软件急停：未知/);
});

test('an explicit emergency release uses fresh bridge status even with stale or missing vision', () => {
  MockWebSocket.instances=[];
  const page=startDashboard({protocol:'http:'});
  const ws=MockWebSocket.instances.find(socket=>socket.url.endsWith('/ws'));
  ws.open(); ws.message({v:1,type:'hello',client_id:'release',simulation:false,limits:{speed:100,turn:300}});
  const state={v:1,type:'state',owner:null,armed:false,jetson:{online:true},fpga:{ready:true},video:{},
    status:{bridge_id:'release-bridge',seq:1,emergency_stop:true},
    vision:{age_ms:1000,stale:true,mission_state:'IDLE',control_source:'none',ros_authorized:false,
      emergency_stop:false,width:800,height:480,detections:[],selected_target:null}};
  const initialMessages=ws.sent.length;
  ws.message({...state,vision:null});
  assert.equal(page.elements.get('visionRelease').disabled,true,'the first cached status has unknown freshness');
  assert.equal(page.elements.get('visionEstop').disabled,false,'stopping needs neither vision nor a confirmed stop status');
  assert.equal(ws.sent.length,initialMessages,'an online state alone sends no stop request');
  page.elements.get('visionEstop').dispatch('click');
  assert.deepEqual({...ws.sent.at(-1)},{v:1,type:'mission_command',action:'emergency_stop'});
  ws.message({v:1,type:'mission_result',action:'emergency_stop',success:true,message:'已响应'});
  const sentBefore=ws.sent.length;
  ws.message({...state,status:{...state.status,seq:2}});
  assert.match(page.elements.get('visionStatus').textContent,/视觉观测已过期.*软件急停：已激活/);
  assert.equal(page.elements.get('visionRelease').disabled,false,'stale vision false cannot hide a fresh active stop');
  assert.equal(page.elements.get('visionStart').disabled,true,'stale vision still blocks task start');
  assert.equal(ws.sent.length,sentBefore,'state updates never release a stop automatically');
  page.elements.get('visionRelease').dispatch('click');
  assert.deepEqual({...ws.sent.at(-1)},{v:1,type:'mission_command',action:'release_emergency_stop'});
  assert.equal(page.elements.get('visionRelease').disabled,true,'a pending explicit request cannot be repeated');
  ws.message({v:1,type:'mission_result',action:'release_emergency_stop',success:true,message:'已响应'});
  page.advancePerf(501);
  ws.message({...state,status:{...state.status,seq:2}});
  assert.match(page.elements.get('visionStatus').textContent,/软件急停：未知/);
  assert.equal(page.elements.get('visionRelease').disabled,true,'an unchanged expired status cannot authorize release');
  const beforeUnknownClick=ws.sent.length;
  page.elements.get('visionRelease').dispatch('click');
  assert.equal(ws.sent.length,beforeUnknownClick,'an unknown stop state sends no release command');
  ws.message({...state,vision:null,status:{...state.status,seq:3}});
  assert.equal(page.elements.get('visionRelease').disabled,false,'fresh active stop is independent of vision presence');
  assert.equal(page.elements.get('visionEstop').disabled,false,'an online bridge can receive an explicit stop without vision');
  assert.equal(ws.sent.length,beforeUnknownClick,'missing vision still does not trigger an automatic command');
  page.elements.get('visionEstop').dispatch('click');
  assert.deepEqual({...ws.sent.at(-1)},{v:1,type:'mission_command',action:'emergency_stop'});
});

test('simulation conservatively blocks an emergency control source without claiming a boolean state', () => {
  MockWebSocket.instances=[];
  const page=startDashboard({protocol:'http:'});
  const ws=MockWebSocket.instances.find(socket=>socket.url.endsWith('/ws'));
  ws.open(); ws.message({v:1,type:'hello',client_id:'simulation-stop',simulation:true,limits:{speed:100,turn:300}});
  const state={v:1,type:'state',owner:null,armed:false,simulation:true,jetson:{online:true},fpga:{ready:true},video:{},
    vision:{age_ms:0,stale:false,mission_state:'IDLE',control_source:'none',ros_authorized:false,
      emergency_stop:false,width:800,height:480,detections:[],selected_target:null},
    status:{bridge_id:'simulation-stop',seq:1,control_source:'none'}};
  page.elements.get('visionBehavior').value='search';
  ws.message(state);
  ws.message({...state,status:{...state.status,seq:2}});
  assert.match(page.elements.get('visionStatus').textContent,/软件急停：未激活/);
  assert.equal(page.elements.get('visionStart').disabled,false);
  ws.message({...state,status:{...state.status,seq:3,control_source:'emergency_stop'}});
  assert.match(page.elements.get('visionStatus').textContent,/软件急停：未知.*反馈不一致/);
  assert.equal(page.elements.get('visionStart').disabled,true);
});

test('vision rejection distinguishes stale observations, unknown bridge, active stop, and missing video', () => {
  MockWebSocket.instances=[];
  const page=startDashboard({protocol:'http:'});
  const ws=MockWebSocket.instances.find(socket=>socket.url.endsWith('/ws'));
  ws.open(); ws.message({v:1,type:'hello',client_id:'vision-reasons',simulation:false,limits:{speed:100,turn:300}});
  const vision={age_ms:0,stale:true,mission_state:'IDLE',control_source:'none',ros_authorized:false,
    emergency_stop:false,width:800,height:480,detections:[],selected_target:null};
  const base={v:1,type:'state',owner:null,armed:false,jetson:{online:true},fpga:{ready:true},video:{}};
  const send=(seq,stop,view=vision,source='none')=>ws.message({...base,vision:view,
    status:{bridge_id:'vision-reasons',session:'session',seq,emergency_stop:stop,control_source:source}});
  page.elements.get('visionBehavior').value='search';
  send(1,false); send(2,false);
  const count=ws.sent.length;
  page.elements.get('visionStart').dispatch('click');
  assert.equal(page.elements.get('visionResult').textContent,'视觉观测已过期，不能启动。');
  assert.doesNotMatch(page.elements.get('visionResult').textContent,/急停/);
  assert.equal(ws.sent.length,count);

  page.advancePerf(501);
  send(2,false,{...vision,stale:false});
  page.elements.get('visionStart').dispatch('click');
  assert.match(page.elements.get('visionResult').textContent,/网桥状态未知或已过期/);
  assert.doesNotMatch(page.elements.get('visionResult').textContent,/已激活/);
  assert.equal(ws.sent.length,count,'unknown state sends no motion');

  send(3,true,{...vision,stale:false});
  page.elements.get('visionStart').dispatch('click');
  assert.match(page.elements.get('visionResult').textContent,/软件急停已激活/);
  assert.equal(ws.sent.length,count,'active stop sends no motion');

  send(4,false,{...vision,stale:false},'emergency_stop');
  page.elements.get('visionStart').dispatch('click');
  assert.match(page.elements.get('visionResult').textContent,/反馈不一致，状态未知/);
  assert.doesNotMatch(page.elements.get('visionResult').textContent,/已激活/);
  assert.equal(ws.sent.length,count,'conflicting evidence sends no motion');

  send(5,false,{...vision,stale:false});
  page.elements.get('visionBehavior').value='approach';
  assert.match(page.elements.get('visionTarget').textContent,/视频画面缺失/);
  page.elements.get('visionStart').dispatch('click');
  assert.match(page.elements.get('visionResult').textContent,/视频画面缺失/);
  assert.doesNotMatch(page.elements.get('visionResult').textContent,/急停/);
  assert.equal(ws.sent.length,count,'missing video sends no motion');

  const reason='已有 LOST 任务，请先结束原任务';
  ws.message({v:1,type:'mission_result',action:'start',success:false,message:reason});
  assert.match(page.elements.get('visionResult').textContent,/已有 LOST 任务，请先结束原任务/);
  assert.doesNotMatch(page.elements.get('visionResult').textContent,/急停/);
  assert.equal(ws.sent.length,count,'displaying a failure does not retry');
});

test('camera fault explanation uses only fresh health telemetry', () => {
  MockWebSocket.instances=[];
  const page=startDashboard({protocol:'http:'});
  const ws=MockWebSocket.instances.find(socket=>socket.url.endsWith('/ws'));
  ws.open(); ws.message({v:1,type:'hello',client_id:'camera',simulation:false,limits:{speed:100,turn:300}});
  ws.message({v:1,type:'state',owner:null,armed:false,jetson:{online:true},fpga:{ready:true},
    status:{bridge_id:'camera'},video:{}});
  const data={type:'health',sequence:1,time_us:1,camera_calibration:{state:13,fault:true,ready:false},
    camera_autocal_diag_valid:true,camera_autocal_diag:{reason_name:'bad_csi_burst',retry_count:3},video_packets:0};
  const health=data=>({v:1,type:'telemetry',bridge_id:'camera',stream:'health',epoch:1,valid:true,data});
  ws.message(health(data));
  assert.match(page.elements.get('videoMetrics').textContent,/相机采集校准故障.*状态 13.*CSI 突发异常.*重试 3 次.*FPGA视频包累计 0/);
  page.advancePerf(2001); page.runIntervals(250);
  assert.doesNotMatch(page.elements.get('videoMetrics').textContent,/相机采集校准故障/,
    'expired health cannot be presented as a current camera fault');
  ws.message(health({...data,sequence:2,camera_calibration:{state:0,fault:false,ready:true}}));
  assert.doesNotMatch(page.elements.get('videoMetrics').textContent,/相机采集校准故障/);
});

test('dashboard exposes only UDP control, local tasks and the QWE / ASD network layout', () => {
  MockWebSocket.latest=null;
  const page=startDashboard();
  assert.equal(page.elements.get('wbNetworkControls').tagName,'DIV');
  assert.equal(page.elements.has('wbBluetoothControls'),false);
  assert.equal(page.elements.has('wbControlSource'),false);
  assert.equal(page.elements.has('wbBluetoothConfig'),false);
  assert.deepEqual(page.document.querySelectorAll('[data-network-key]').map(button=>button.dataset.networkKey),
    ['q','w','e','a','s','d'],'network keypad DOM order is QWE on the first row and ASD on the second');
  assert.deepEqual(page.document.querySelectorAll('[data-drive-key]'),[]);
  assert.equal(MockWebSocket.latest,null,'opening the dashboard does not connect or arm the network service');
});

test('top workbench stop uses the shared stop path and clears network movement authorization', async () => {
  MockWebSocket.latest=null;
  const page=startDashboard();
  const socket=await openNetworkService(page,{clientId:'top-stop-page',simulation:true});
  const state={v:1,type:'state',owner:null,armed:false,simulation:true,
    jetson:{online:true,host:'127.0.0.1',port:16200,age_ms:5},
    fpga:{ready:true,ack_age_ms:5},status:{bridge_id:'top-stop-bridge',actual_mode:'manual'},video:{}};
  socket.message(state);
  page.elements.get('networkArm').dispatch('click');
  socket.message({...state,owner:'top-stop-page',armed:true});
  page.document.dispatch('keydown',{code:'KeyW',preventDefault(){}});
  assert.ok(socket.sent.some(message=>message.type==='drive'&&message.vx===100),
    'the armed page can issue the pre-stop movement command');

  page.elements.get('wbStop').dispatch('click');
  const stop=socket.sent.at(-1);
  assert.equal(stop.type,'stop');
  assert.equal(stop.reason,'用户从顶部请求停车并解除控制');
  assert.match(page.elements.get('networkStopRequest').textContent,/本页已发送/);
  assert.match(page.elements.get('networkDriveStatus').textContent,/仅监控/);

  const afterStop=socket.sent.length;
  page.advancePerf(250);
  page.runIntervals(250);
  page.document.dispatch('keydown',{code:'KeyW',preventDefault(){}});
  assert.equal(socket.sent.slice(afterStop).some(message=>message.type==='drive'&&message.vx!==0),false,
    'the existing 250 ms dashboard tick and later key input cannot restore movement');
});

test('opening connection settings stops an authorized link and never resumes held movement', async () => {
  MockWebSocket.latest=null;
  const page=startDashboard();
  page.elements.get('wbConfigure').dispatch('click');
  const dialog=page.elements.get('wbConnectionDialog');
  assert.equal(dialog.open,true);
  assert.equal(page.elements.get('wbNetworkConfig').hidden,false);
  const socket=await openNetworkService(page,{clientId:'configure-network-page',simulation:true});
  const state={v:1,type:'state',owner:null,armed:false,simulation:true,
    jetson:{online:true,host:'127.0.0.1',port:16200,age_ms:5},
    fpga:{ready:true,ack_age_ms:5},status:{bridge_id:'configure-network-bridge',actual_mode:'manual'},video:{}};
  socket.message(state);
  page.elements.get('wbConnectionClose').dispatch('click');
  assert.equal(dialog.open,false);
  page.elements.get('networkArm').dispatch('click');
  socket.message({...state,owner:'configure-network-page',armed:true});
  page.document.dispatch('keydown',{code:'KeyW',preventDefault(){}});
  assert.ok(socket.sent.some(message=>message.type==='drive'&&message.vx===100),
    'the network link is moving only after explicit owner authorization');

  page.elements.get('wbConfigure').dispatch('click');
  assert.equal(dialog.open,true);
  assert.equal(page.elements.get('wbNetworkConfig').hidden,false);
  assert.equal(socket.sent.at(-1).type,'stop');
  assert.equal(socket.sent.at(-1).reason,'打开连接配置');
  const afterConfigure=socket.sent.length;
  page.elements.get('wbConnectionClose').dispatch('click');
  page.document.dispatch('keyup',{code:'KeyW',preventDefault(){}});
  page.document.dispatch('keydown',{code:'KeyW',preventDefault(){}});
  assert.equal(socket.sent.slice(afterConfigure).some(message=>message.type==='drive'&&message.vx!==0),false,
    'closing configuration and pressing a formerly held key cannot resume motion');
  assert.equal(socket.sent.slice(afterConfigure).some(message=>message.type==='arm'),false,
    'opening and closing configuration do not re-arm the network link');

  const settingsLink=page.document.querySelectorAll('.console-nav .nav-item')
    .find(link=>link.getAttribute('href')==='#connectionSettings');
  assert.ok(settingsLink,'the navigation keeps a connection settings entry');
  const beforeNav=socket.sent.length;
  let defaultPrevented=false;
  settingsLink.dispatch('click',{preventDefault(){defaultPrevented=true;}});
  assert.equal(defaultPrevented,true,'the nav entry opens the dialog instead of navigating to a hidden section');
  assert.equal(dialog.open,true);
  assert.equal(page.elements.get('wbNetworkConfig').hidden,false);
  page.elements.get('wbConnectionClose').dispatch('click');
  assert.equal(dialog.open,false);
  assert.equal(socket.sent.slice(beforeNav).some(message=>message.type==='arm'),false,
    'the navigation entry does not re-arm the network link');
});

test('UDP telemetry and video remain independent while opening settings clears network authorization', async () => {
  MockWebSocket.latest=null;
  const page=startDashboard();
  const socket=await openNetworkService(page,{clientId:'udp-only-page',simulation:true});
  const state={v:1,type:'state',owner:null,armed:false,simulation:true,
    jetson:{online:true,host:'127.0.0.1',port:16200,age_ms:5},
    fpga:{ready:true,ack_age_ms:5},status:{bridge_id:'udp-only-bridge',actual_mode:'manual'},video:{online:true,frames:3}};
  socket.message(state);
  const udpTelemetry={
    imu:{type:'imu',sequence:71,who_am_i:0x6a,flags:3,sample_sequence:71,
      gyro_raw:{x:11,y:12,z:13},accel_raw:{x:14,y:15,z:16},gyro_rad_s_nominal:{x:0.1,y:0.2,z:0.3}},
    motor:{type:'motor',sequence:72,motor_channel_map:0xd2,encoder_channel_map:0xd2,
      motor_invert:0x0b,encoder_invert:0x0a,wheel_mm_s:{A:12.5,B:-7.5,C:0,D:40}},
    lidar:{type:'lidar',sequence:73,points:[{angle_cdeg:12000,distance_mm:500}]},
    fusion:{type:'fusion',sequence:74,flags:1,state:{x_m:1,y_m:2,yaw_rad:0,vx_m_s:0,vy_m_s:0,wz_rad_s:0,gyro_bias_rad_s:0},
      covariance_diag:[0,0,0,0,0,0,0]},
    health:{type:'health',sequence:75}
  };
  for (const [stream,data] of Object.entries(udpTelemetry))
    socket.message({v:1,type:'telemetry',bridge_id:'udp-only-bridge',stream,epoch:1,valid:true,data});
  page.runIntervals(250);
  for (const id of ['streamImu','streamMotor','streamLidar','streamFusion','streamHealth'])
    assert.equal(page.elements.get(id).dataset.state,'fresh',`${id} starts with fresh UDP telemetry`);
  assert.match(page.elements.get('imuRaw').textContent,/WHO_AM_I=0x6a/);
  assert.match(page.elements.get('networkWheelMapping').textContent,/motor_channel_map=0xD2/i);
  const videoBefore={text:page.elements.get('videoStatus').textContent,state:page.elements.get('videoStatus').dataset.state};
  page.elements.get('networkArm').dispatch('click');
  socket.message({...state,owner:'udp-only-page',armed:true});
  page.document.dispatch('keydown',{code:'KeyW',preventDefault(){}});
  assert.ok(socket.sent.some(message=>message.type==='drive'&&message.vx===100),
    'explicit network authorization enables the UDP control path');
  page.elements.get('wbConfigure').dispatch('click');
  assert.equal(page.elements.get('wbConnectionDialog').open,true);
  assert.equal(socket.sent.at(-1).type,'stop','opening settings clears network authorization');
  const afterStop=socket.sent.length;
  page.elements.get('wbConnectionClose').dispatch('click');
  page.document.dispatch('keyup',{code:'KeyW',preventDefault(){}});
  page.document.dispatch('keydown',{code:'KeyW',preventDefault(){}});
  assert.equal(socket.sent.slice(afterStop).some(message=>message.type==='drive'&&message.vx!==0),false,
    'closing settings cannot resume held network movement');
  assert.deepEqual({text:page.elements.get('videoStatus').textContent,state:page.elements.get('videoStatus').dataset.state},videoBefore,
    'UDP telemetry and video status remain independent');
});
test('navigation guard reasons distinguish known and unknown codes', async () => {
  const page=startDashboard();
  const socket=await openNetworkService(page,{clientId:'guard-reasons'});
  const nav={type:'navigation',boot_id:1,odom_domain:1,task_id:1,last_task_id:1,
    task_id_watermark_known:true,status:5,mode:2,active_task:true,goal_x_q16:0,goal_y_q16:0,
    expiry_us:0,now_us:0,age_ms:1,stale:false};
  for (const [guard,error,reason] of [[4,68,'轮速'],[12,76,'命令链路'],[14,78,'近障'],[103,103,'未知']]) {
    socket.message({v:1,type:'state',owner:null,armed:false,jetson:{online:true},fpga:{ready:true},
      status:{bridge_id:'guard-bridge'},navigation:{...nav,guard_error:guard,error},video:{}});
    const text=page.elements.get('localTaskState').textContent;
    assert.ok(text.includes(`保护原因 ${reason}（${guard}）`));
    assert.ok(text.includes(`原始错误 ${error}（0x${error.toString(16)}）`));
  }
});

test('navigation deadline at zero remains visible across the 32-bit clock wrap', async () => {
  const page=startDashboard();
  const socket=await openNetworkService(page,{clientId:'wrapped-deadline'});
  const nav={type:'navigation',boot_id:1,odom_domain:1,task_id:1,last_task_id:1,
    task_id_watermark_known:true,status:3,mode:2,active_task:true,expiry_us:0,
    deadline_known:true,now_us:2**32-2000000,age_ms:1,stale:false};
  for (const protocol of ['nav_full_v1','nav_polar_v2','nav_grid_v1']) for (const status of [3,8]) {
    socket.message({v:1,type:'state',owner:null,armed:false,jetson:{online:true},fpga:{ready:true},
      status:{bridge_id:'deadline-bridge'},navigation:{...nav,protocol,status},video:{}});
    assert.match(page.elements.get('localTaskState').textContent,/剩余 2\.00 s/);
  }
  socket.message({v:1,type:'state',owner:null,armed:false,jetson:{online:true},fpga:{ready:true},
    status:{bridge_id:'deadline-bridge'},navigation:{...nav,protocol:'nav_polar_v2',
      deadline_known:false,status:0,mode:1,active_task:false},video:{}});
  assert.match(page.elements.get('localTaskState').textContent,/无活动任务期限/);
});

test('workbench reports retired no-target auto separately from real local-task and guard status', async () => {
  MockWebSocket.latest=null;
  const page=startDashboard();
  const socket=await openNetworkService(page,{clientId:'navigation-state-page',simulation:true});
  const navigation={type:'navigation',sequence:1,boot_id:77,odom_domain:3,task_id:12,last_task_id:14,
    task_id_watermark_known:true,status:5,error:0x44,
    guard_error:4,planner_error:0,mode:2,flags:0,active_task:true,goal_x_q16:65536,goal_y_q16:-32768,
    expiry_us:40000000,now_us:10000000,age_ms:5,stale:false,candidate_count:2,map_age_us:10000,pose_age_us:5000};
  const state=(jetson,status,nav=navigation)=>({v:1,type:'state',owner:null,armed:false,simulation:true,
    jetson:{online:true,host:'127.0.0.1',port:16200,age_ms:5,...jetson},
    fpga:{ready:true,ack_age_ms:5},status,navigation:nav,video:{}});

  socket.message(state({},null,null));
  assert.equal(page.elements.get('wbSimulation').textContent,'软件模拟');
  assert.match(page.elements.get('wbConnection').textContent,/网关状态不可用/);
  assert.match(page.elements.get('wbActualMode').textContent,/不可用/);
  assert.match(page.elements.get('wbModeRequest').textContent,/已停用/);

  socket.message(state({}, {bridge_id:'navigation-bridge',actual_mode:'manual',auto_requested:false},null));
  assert.equal(page.elements.get('wbActualMode').textContent,'手动');
  assert.equal(page.elements.get('networkActualMode').textContent,'FPGA融合状态位：手动位');
  assert.match(page.elements.get('wbModeRequest').textContent,/无目标type5自动模式：已停用/);
  socket.message(state({}, {bridge_id:'navigation-bridge',actual_mode:'manual',auto_requested:false}));
  assert.match(page.elements.get('wbModeAck').textContent,/局部导航状态：受阻等待重试/);
  assert.match(page.elements.get('localTaskState').textContent,/受阻（等待重试）/);
  assert.match(page.elements.get('localTaskState').textContent,/保护原因 轮速（4）/);
  assert.equal(page.elements.get('localTaskStart').disabled,true,'a blocked active task cannot be duplicated');
  assert.equal(page.elements.get('localTaskStop').disabled,false,'a device task is stoppable after the page reconnects');
  assert.equal(page.elements.get('localTaskCancel').disabled,false,'a device task is cancellable without a web-owned task');

  socket.message(state({}, {bridge_id:'navigation-bridge',actual_mode:'manual',auto_requested:false},
    {...navigation,sequence:2,status:3,error:4,guard_error:0,planner_error:4}));
  assert.match(page.elements.get('localTaskState').textContent,/规划器错误原因码 4/);
  assert.doesNotMatch(page.elements.get('localTaskState').textContent,/保护原因/);

  socket.message(state({}, {bridge_id:'navigation-bridge',actual_mode:'manual',auto_requested:false},
    {...navigation,sequence:3,error:0x81,rx_rejection:true,rejected:true,rejected_task_id:99,task_id:12}));
  assert.match(page.elements.get('localTaskResult').textContent,/拒绝接收任务 99/);
  assert.match(page.elements.get('localTaskState').textContent,/收到新任务接收拒绝/);

  const legacy = {...navigation,sequence:4,status:0,mode:0,active_task:false,
    last_task_id:undefined,task_id_watermark_known:false};
  socket.message(state({}, {bridge_id:'navigation-bridge',actual_mode:'manual',auto_requested:false},legacy));
  assert.match(page.elements.get('localTaskFreshness').textContent,/等待 Jetson 确认执行器状态/);
  assert.equal(page.elements.get('localTaskStart').disabled,true,
    'no goal has been selected');

  page.advancePerf(501);
  page.runIntervals(250);
  assert.match(page.elements.get('wbActualMode').textContent,/不可用/,'stale service status cannot support an actual-mode claim');
  assert.match(page.elements.get('wbModeAck').textContent,/等待遥测/,'stale navigation state is not presented as current');
});
test('workbench trend consumes fresh UDP motor samples', async () => {
  MockWebSocket.latest=null;
  const page=startDashboard();
  const socket=await openNetworkService(page,{clientId:'trend-page',simulation:true});
  socket.message({v:1,type:'state',owner:null,armed:false,simulation:true,
    jetson:{online:true,host:'127.0.0.1',port:16200,age_ms:5},fpga:{ready:true,ack_age_ms:5},
    status:{bridge_id:'trend-bridge',actual_mode:'manual'},video:{}});
  socket.message({v:1,type:'telemetry',bridge_id:'trend-bridge',stream:'motor',epoch:1,valid:true,
    data:{type:'motor',sequence:1,time_us:10,wheel_mm_s:{A:12.5,B:-7.5,C:0,D:40}}});
  page.runIntervals(250);
  assert.match(page.elements.get('telemetryTrendStatus').textContent,/A 12\.5/);
  assert.match(page.elements.get('telemetryTrendStatus').textContent,/B -7\.5/);
  assert.match(page.elements.get('telemetryTrendStatus').textContent,/D 40/);
});
test('Jetson target comes from service state and changes only after an explicit Apply click', async () => {
  MockWebSocket.latest=null;
  const page=startDashboard();
  await page.elements.get('networkConnect').dispatch('click');
  const socket=MockWebSocket.latest;
  socket.open();
  socket.message({v:1,type:'hello',client_id:'browser-2',simulation:false,limits:{speed:80,turn:200}});
  socket.message({v:1,type:'state',owner:null,armed:false,simulation:false,
    jetson:{online:true,host:'10.0.0.8',port:6200,age_ms:5},fpga:{ready:true,ack_age_ms:5},status:{bridge_id:'bridge-live'},video:{}});
  assert.equal(page.elements.get('jetsonHost').value,'10.0.0.8');
  assert.equal(page.elements.get('jetsonPort').value,'6200');
  assert.equal(socket.sent.some(message=>message.type==='configure'),false);
  page.elements.get('jetsonHost').value='10.0.0.9';
  page.elements.get('jetsonPort').value='16200';
  page.elements.get('jetsonHost').dispatch('input');
  page.elements.get('jetsonPort').dispatch('input');
  assert.equal(page.elements.get('networkApplyTarget').disabled,false);
  page.elements.get('networkApplyTarget').dispatch('click');
  assert.deepEqual(socket.sent.at(-1),{v:1,type:'configure',jetson:'10.0.0.9',port:16200});
  socket.message({v:1,type:'state',owner:null,armed:false,simulation:false,
    jetson:{online:true,host:'10.0.0.9',port:16200,age_ms:5},fpga:{ready:true,ack_age_ms:5},status:{bridge_id:'bridge-live'},video:{}});
  assert.match(page.elements.get('networkEvent').textContent,/已确认新的 Jetson 目标/);
});

test('network blur, hidden-tab and pagehide events clear authorization and discard held keys', async () => {
  for (const event of ['blur','visibilitychange','pagehide']) {
    const page=startDashboard();
    await page.elements.get('networkConnect').dispatch('click');
    const socket=MockWebSocket.latest;
    socket.open();
    socket.message({v:1,type:'hello',client_id:'event-page',simulation:true,limits:{speed:100,turn:300}});
    const state={v:1,type:'state',owner:null,armed:false,simulation:true,
      jetson:{online:true,host:'127.0.0.1',port:16200,age_ms:1},fpga:{ready:true,ack_age_ms:1},
      status:{bridge_id:'event-bridge',auto_requested:false},video:{}};
    socket.message(state);
    page.elements.get('networkArm').dispatch('click');
    socket.message({...state,owner:'event-page',armed:true});
    page.document.dispatch('keydown',{code:'KeyW',preventDefault(){}});
    assert.ok(socket.sent.some(message=>message.type==='drive' && message.vx===100));
    if (event==='visibilitychange') {
      page.document.hidden=true;
      page.document.dispatch(event);
    } else page.window.dispatch(event);
    assert.equal(socket.sent.at(-1).type,'stop',event);
    const afterStop=socket.sent.length;
    page.document.dispatch('keydown',{code:'KeyW',preventDefault(){}});
    // 即使延迟到达的服务状态仍报告旧 owner，本页授权也已经清除。
    socket.message({...state,owner:'event-page',armed:true});
    page.document.dispatch('keydown',{code:'KeyW',preventDefault(){}});
    assert.equal(socket.sent.slice(afterStop).some(message=>message.type==='drive' && message.vx!==0),false,event);
    assert.match(page.elements.get('networkDriveStatus').textContent,/仅监控/);
  }
});

test('state timeout clears held keys and a late owner report cannot restore manual control', async () => {
  const page=startDashboard();
  const socket=await openNetworkService(page,{clientId:'timeout-page'});
  const state={v:1,type:'state',owner:null,armed:false,simulation:true,
    jetson:{online:true,age_ms:1},fpga:{ready:true,ack_age_ms:1},
    status:{bridge_id:'timeout-bridge'},video:{}};
  socket.message(state);
  page.elements.get('networkArm').dispatch('click');
  const owned={...state,owner:'timeout-page',armed:true};
  socket.message(owned);
  page.document.dispatch('keydown',{code:'KeyW',preventDefault(){}});
  assert.ok(socket.sent.some(m=>m.type==='drive' && m.vx!==0));
  page.advancePerf(501);
  page.runIntervals(50);
  assert.equal(socket.sent.at(-1).type,'stop');
  assert.match(page.elements.get('networkDriveStatus').textContent,/仅监控/);
  const afterStop=socket.sent.length;
  socket.message(owned);
  page.document.dispatch('keydown',{code:'KeyW',preventDefault(){}});
  page.runIntervals(50);
  assert.equal(socket.sent.slice(afterStop).some(m=>m.type==='drive'),false);
});

test('a running visual task survives blur and a hidden tab without automatic commands', async () => {
  for (const event of ['blur','visibilitychange']) {
    const page=startDashboard();
    const socket=await openNetworkService(page,{clientId:'visual-controller'});
    const vision={age_ms:1,stale:false,mission_state:'IDLE',control_source:'none',
      ros_authorized:false,emergency_stop:false,width:800,height:480,detections:[],selected_target:null};
    const state={v:1,type:'state',owner:null,armed:false,simulation:true,
      jetson:{online:true,age_ms:1},fpga:{ready:true,ack_age_ms:1},
      status:{bridge_id:'visual-bridge',seq:1,emergency_stop:false},video:{},vision};
    socket.message(state);
    socket.message({...state,status:{...state.status,seq:2}});
    page.elements.get('visionBehavior').value='search';
    page.elements.get('visionClass').value='39';
    page.elements.get('visionBehavior').dispatch('change');
    assert.equal(page.elements.get('visionStart').disabled,false);
    page.elements.get('visionStart').dispatch('click');
    assert.equal(socket.sent.at(-1).action,'start');
    socket.message({v:1,type:'mission_result',action:'start',success:true,message:'已接收'});
    socket.message({...state,status:{...state.status,seq:3},
      vision:{...vision,mission_state:'SEARCHING',control_source:'ros',ros_authorized:true}});
    const before=socket.sent.length;
    if (event==='visibilitychange') {
      page.document.hidden=true;
      page.document.dispatch(event);
      page.document.hidden=false;
      page.document.dispatch(event);
    } else {
      page.window.dispatch(event);
      page.window.focus(true);
      page.window.dispatch('focus');
    }
    page.runIntervals(250);
    assert.deepEqual(socket.sent.slice(before),[],`${event} must not stop or resume a visual task`);
    page.elements.get('visionStop').dispatch('click');
    assert.deepEqual({...socket.sent.at(-1)},{v:1,type:'mission_command',action:'stop'},
      'an explicit visual stop remains available');
  }
});

test('pagehide still sends a stop for an authorized visual task', async () => {
  const page=startDashboard();
  const socket=await openNetworkService(page,{clientId:'visual-controller'});
  socket.message({v:1,type:'state',owner:null,armed:false,simulation:true,
    jetson:{online:true,age_ms:1},fpga:{ready:true,ack_age_ms:1},
    status:{bridge_id:'visual-bridge',seq:1,emergency_stop:false},video:{},
    vision:{age_ms:1,stale:false,mission_state:'FOLLOWING',control_source:'ros',
      ros_authorized:true,emergency_stop:false,width:800,height:480,detections:[],selected_target:null}});
  const before=socket.sent.length;
  page.window.dispatch('pagehide');
  assert.deepEqual(socket.sent.slice(before),[{v:1,type:'stop',reason:'页面关闭，停车并清除授权'}]);
});

test('a pending manual authorization still stops on blur', async () => {
  const page=startDashboard();
  const socket=await openNetworkService(page,{clientId:'pending-controller'});
  socket.message({v:1,type:'state',owner:null,armed:false,simulation:true,
    jetson:{online:true,age_ms:1},fpga:{ready:true,ack_age_ms:1},
    status:{bridge_id:'manual-bridge'},video:{}});
  page.elements.get('networkArm').dispatch('click');
  assert.equal(socket.sent.at(-1).type,'arm');
  page.window.dispatch('blur');
  assert.equal(socket.sent.at(-1).type,'stop');
  page.window.focus(true);
  const before=socket.sent.length;
  page.document.dispatch('keydown',{code:'KeyW',preventDefault(){}});
  assert.equal(socket.sent.length,before,'focus does not restore a pending manual authorization');
});

test('an observing page does not cancel a local task on focus loss', async () => {
  for (const event of ['blur','visibilitychange','pagehide']) {
    const page=startDashboard();
    const socket=await openNetworkService(page,{clientId:'monitor-page'});
    const navigation={boot_id:7,odom_domain:3,task_id:8,last_task_id:8,task_id_watermark_known:true,
      status:3,mode:2,active_task:true,age_ms:1,stale:false,goal_x_q16:0,goal_y_q16:0};
    socket.message({v:1,type:'state',owner:null,armed:false,jetson:{online:true},fpga:{ready:true},
      status:{bridge_id:'monitor-bridge'},navigation,video:{}});
    const before=socket.sent.length;
    if (event==='visibilitychange') { page.document.hidden=true; page.document.dispatch(event); }
    else page.window.dispatch(event);
    assert.equal(socket.sent.slice(before).some(message=>message.type==='stop'),false,event);
  }
});

test('diagnostics retain per-stream fields, units, hexadecimal masks and stale clearing', async () => {
  const page=startDashboard();
  const socket=await openNetworkService(page);
  socket.message({v:1,type:'state',owner:null,armed:false,simulation:true,
    jetson:{online:true,age_ms:5},fpga:{ready:true,ack_age_ms:5},status:{bridge_id:'diagnostic-bridge'},video:{}});
  const data={
    motor:{queue_dropped:3,encoder_invalid_mask:10},
    imu:{identity_errors:4},health:{queue_dropped:null,cpu_ready:true,cpu_fault:false,cpu_fault_code:0},
    fusion:{flags:0xd0000001,updates:22,max_control_gap_us:22500,csm_accepted:7,csm_rejected:1,
      startup_algorithm_selftest_passed:true,imu_age_us:100,radar_age_us:250,
      pose_stamp_us:50000,lidar_drops:2,map_lidar_drops:2,map_ray_queue_depth:17,map_ray_drops:123,
      map_ray_overflow:true,map_shift_pending:false}
  };
  for(const [stream,values] of Object.entries(data))
    socket.message({v:1,type:'telemetry',bridge_id:'diagnostic-bridge',stream,epoch:1,valid:true,data:{type:stream,sequence:1,...values}});
  page.runIntervals(250);
  const expected={d0:'22',d2:'22500 µs',d3:'7',d4:'1',d5:'通过',d6:'100 µs',d7:'250 µs',
    d8:'50000 µs',d9:'2',d10:'0xd0000001',d11:'3',d12:'0x0a',d13:'4',d14:'不可用',
    d15:'17 /32',d16:'123',d17:'是',d18:'否',d19:'是',d20:'否',d21:'0'};
  for(const [id,value] of Object.entries(expected))assert.equal(page.elements.get(id).textContent,value,id);
  assert.equal(page.elements.get('d1').textContent,'不可用','missing counts are not invented');
  assert.match(page.elements.get('udpRawFlags').textContent,/0xd0000001/);
  page.advancePerf(1001);
  page.runIntervals(250);
  for(const id of Object.keys(expected))assert.equal(page.elements.get(id).textContent,'不可用',id+' clears after fusion expires');
  assert.equal(page.elements.get('udpRawFlags').textContent,'UDP fusion.flags 不可用');
});

test('single-page navigation keeps every work area visible and preserves the task draft', async () => {
  const page=startDashboard({initialHash:'#tasks'});
  const areas=['overview','tasks','remotePage','sensors','diagnosticsPage'];
  for (const id of areas) assert.equal(page.elements.get(id).hidden,false,id+' remains in the document flow');
  assert.equal(page.elements.get('tasks').lastScroll.block,'start','a deep link scrolls to the work area');
  const socket=await openNetworkService(page);
  page.elements.get('localDuration').value='45';
  const before=socket.sent.length;
  const sensorLink=page.document.querySelectorAll('a[data-nav]').find(link=>link.getAttribute('data-nav')==='sensors');
  sensorLink.dispatch('click',{preventDefault(){}});
  assert.equal(page.elements.get('sensors').lastScroll.block,'start');
  assert.equal(sensorLink.getAttribute('aria-current'),'location');
  assert.equal(page.elements.get('pageBreadcrumb').textContent,'传感器');
  assert.equal(page.elements.get('localDuration').value,'45','anchor navigation retains the task draft');
  for (const id of areas) assert.equal(page.elements.get(id).hidden,false);
  assert.equal(socket.sent.slice(before).some(message=>['arm','stop','local_task','mission_command'].includes(message.type)),false,
    'monitor navigation does not acquire control or interrupt an autonomous task');
});

test('navigating away from manual control releases held motion and returning does not re-arm', async () => {
  const page=startDashboard();
  const socket=await openNetworkService(page,{clientId:'anchor-control-page'});
  const state={v:1,type:'state',owner:null,armed:false,simulation:true,
    jetson:{online:true,age_ms:5},fpga:{ready:true,ack_age_ms:5},
    status:{bridge_id:'anchor-control-bridge',actual_mode:'manual'},video:{}};
  socket.message(state);
  page.elements.get('networkArm').dispatch('click');
  socket.message({...state,owner:'anchor-control-page',armed:true});
  page.document.dispatch('keydown',{code:'KeyW',preventDefault(){}});
  assert.ok(socket.sent.some(message=>message.type==='drive'&&message.vx===100));
  const link=id=>page.document.querySelectorAll('a[data-nav]').find(node=>node.getAttribute('data-nav')===id);
  link('tasks').dispatch('click',{preventDefault(){}});
  assert.equal(socket.sent.at(-1).type,'stop');
  assert.match(socket.sent.at(-1).reason,/切换工作区导航/);
  const after=socket.sent.length;
  link('remote').dispatch('click',{preventDefault(){}});
  page.document.dispatch('keydown',{code:'KeyW',preventDefault(){}});
  page.runIntervals(50);
  assert.equal(socket.sent.slice(after).some(message=>message.type==='arm'||message.type==='drive'&&message.vx!==0),false);
  assert.equal(page.elements.get('tasks').hidden,false);
});

test('speed limits and diagonal keys retain normalization and stale input still permits zero', async () => {
  const page=startDashboard();
  const socket=await openNetworkService(page,{clientId:'limits-page'});
  const state={v:1,type:'state',owner:null,armed:false,simulation:true,
    jetson:{online:true,age_ms:5},fpga:{ready:true,ack_age_ms:5},
    status:{bridge_id:'limits-bridge',actual_mode:'manual'},video:{}};
  socket.message(state);
  page.elements.get('networkArm').dispatch('click');
  socket.message({...state,owner:'limits-page',armed:true});
  page.elements.get('networkSpeed').value='80';
  page.elements.get('networkSpeed').dispatch('input');
  page.elements.get('networkTurn').value='275';
  page.elements.get('networkTurn').dispatch('input');
  for(const code of ['KeyW','KeyA','KeyQ'])page.document.dispatch('keydown',{code,preventDefault(){}});
  const drive=socket.sent.at(-1);
  assert.equal(drive.type,'drive');
  assert.equal(drive.vx,Math.round(80/Math.sqrt(2)));
  assert.equal(drive.vy,Math.round(80/Math.sqrt(2)));
  assert.equal(drive.wz,275);
  const beforeInvalid=socket.sent.length;
  page.elements.get('networkSpeed').value='not-a-number';
  page.elements.get('networkSpeed').dispatch('input');
  assert.equal(socket.sent.length,beforeInvalid,'invalid slider input sends no motion');
  assert.equal(page.elements.get('networkSpeedValue').textContent,'80 mm/s');
  page.advancePerf(501);
  for(const code of ['KeyQ','KeyA','KeyW'])page.document.dispatch('keyup',{code,preventDefault(){}});
  const afterExpiry=socket.sent.slice(beforeInvalid).filter(message=>message.type==='drive');
  assert.equal(afterExpiry.length,1,'stale state blocks every remaining nonzero vector');
  assert.equal(afterExpiry[0].vx,0);
  assert.equal(afterExpiry[0].vy,0);
  assert.equal(afterExpiry[0].wz,0,'releasing all keys can still request zero speed');
});

test('UDP control events require fresh state and explicit network authorization', async () => {
  MockWebSocket.latest=null;
  const page=startDashboard();
  const socket=await openNetworkService(page,{clientId:'dispatch-page',simulation:true});
  const state={v:1,type:'state',owner:null,armed:false,simulation:true,
    jetson:{online:true,host:'127.0.0.1',port:16200,age_ms:5},
    fpga:{ready:true,ack_age_ms:5},status:{bridge_id:'dispatch-bridge',actual_mode:'manual'},video:{}};
  socket.message(state);
  const before=socket.sent.length;
  page.document.dispatch('keydown',{code:'KeyW',preventDefault(){}});
  assert.equal(socket.sent.slice(before).some(message=>message.type==='drive'&&message.vx!==0),false,
    'monitoring and keyboard input do not authorize movement');
  assert.equal(socket.sent.slice(before).some(message=>message.type==='arm'),false);
  page.elements.get('networkArm').dispatch('click');
  socket.message({...state,owner:'dispatch-page',armed:true});
  page.document.dispatch('keydown',{code:'KeyW',preventDefault(){}});
  assert.ok(socket.sent.some(message=>message.type==='drive'&&message.vx===100),
    'an acknowledged network owner can move through the control event path');
  const beforeExpired=socket.sent.length;
  page.advancePerf(501);
  page.document.dispatch('keydown',{code:'KeyA',preventDefault(){}});
  assert.equal(socket.sent.slice(beforeExpired).some(message=>message.type==='drive'&&message.vx!==0),false,
    'stale service state cannot authorize further movement');
  page.elements.get('networkStop').dispatch('click');
  assert.equal(socket.sent.at(-1).type,'stop');
});

test('polar board telemetry renders reported error and ICP counters without legacy map labels', async () => {
  const page=startDashboard();
  const socket=await openNetworkService(page);
  socket.message({v:1,type:'state',owner:null,armed:false,simulation:false,
    jetson:{online:true,age_ms:5},fpga:{ready:true,ack_age_ms:5},status:{bridge_id:'polar-board'},video:{}});
  socket.message({v:1,type:'telemetry',bridge_id:'polar-board',stream:'fusion',epoch:1,valid:true,
    data:{type:'fusion',sequence:1,flags:0xd0004001,architecture:'polar_v2',
      state:{x_m:1,y_m:2,yaw_rad:0,vx_m_s:0,vy_m_s:0,wz_rad_s:0,gyro_bias_rad_s:0},
      error_estimate:[30,40,50],covariance_diag:[900,1600,2500,123456,654321],
      scan_match_accepted:7,scan_match_rejected:3,lidar_raw_samples:4500,lidar_reduced_samples:720}});
  page.runIntervals(250);
  assert.equal(page.elements.get('metricSigmaX').textContent,'估计误差 0.030 m');
  assert.equal(page.elements.get('dl3').textContent,'点线ICP 接受数');
  assert.equal(page.elements.get('d3').textContent,'7');
  assert.equal(page.elements.get('d4').textContent,'3');
  assert.match(page.elements.get('poseHeading').textContent,/位姿融合/);
  assert.equal(page.elements.get('d15').textContent,'不可用');
});

test('Jetson navigation reports its live local map instead of FPGA map placeholders', () => {
  MockWebSocket.instances=[];
  const page=startDashboard({protocol:'http:'});
  const ws=MockWebSocket.instances.find(socket=>socket.url.endsWith('/ws'));
  const map=MockWebSocket.instances.find(socket=>socket.url.endsWith('/map'));
  ws.open(); map.open();
  ws.message({v:1,type:'hello',client_id:'jetson-map',simulation:false,limits:{speed:100,turn:300}});
  const nav={protocol:'jetson_nav_v1',boot_id:7,odom_domain:3,age_ms:0,stale:false,
    task_id_watermark_known:true,last_task_id:0,task_id:0,status:0,mode:1,active_task:false,
    goal_x_q16:0,goal_y_q16:0,map_version:0,map_age_us:0xffffffff,pose_age_us:100};
  ws.message({v:1,type:'state',owner:null,armed:false,jetson:{online:true},fpga:{ready:true},
    status:{bridge_id:'jetson-map',emergency_stop:false},navigation:nav,video:{}});
  assert.match(page.elements.get('localTaskPose').textContent,/等待同坐标域的 Jetson 局部栅格/);
  map.message(fpgaMapFixture({protocol:'jetson_grid_view_v1',source:'jetson',navigation_protocol:'jetson_nav_v1'}));
  assert.match(page.elements.get('localTaskPose').textContent,/Jetson 局部栅格就绪/);
  assert.doesNotMatch(page.elements.get('localTaskPose').textContent,/尚无完整圈|地图版本 0/);
  map.message({v:1,type:'mapping',protocol:'jetson_grid_view_v1',source:'jetson',available:false,
    domain:'7:3',reason:'等待快照'});
  assert.match(page.elements.get('localTaskPose').textContent,/等待同坐标域的 Jetson 局部栅格/);
});

test('grid telemetry renders estimates, candidates and wrapped deadline without legacy A-star labels', async () => {
  const page=startDashboard();
  const socket=await openNetworkService(page);
  const nav={protocol:'nav_grid_v1',boot_id:7,odom_domain:3,task_id:5,last_task_id:5,
    task_id_watermark_known:true,status:3,mode:2,active_task:true,expiry_us:0,
    deadline_known:true,now_us:2**32-2000000,age_ms:1,stale:false,goal_x_q16:65536,goal_y_q16:0,
    candidates_evaluated:7,map_version:14,map_age_us:1234,pose_age_us:100};
  socket.message({v:1,type:'state',owner:null,armed:false,simulation:false,
    jetson:{online:true,age_ms:5},fpga:{ready:true},status:{bridge_id:'grid-board'},navigation:nav,video:{}});
  socket.message({v:1,type:'telemetry',bridge_id:'grid-board',stream:'fusion',epoch:1,valid:true,
    data:{type:'fusion',sequence:1,flags:0xd000c001,architecture:'local_grid_v1',
      state:{x_m:1,y_m:2,yaw_rad:0,vx_m_s:0,vy_m_s:0,wz_rad_s:0,gyro_bias_rad_s:0},
      error_estimate:[30,40,50],scan_match_accepted:7,scan_match_rejected:3,
      map_fresh:true,map_ready:true,map_gap_latched:false}});
  page.runIntervals(250);
  assert.equal(page.elements.get('metricSigmaX').textContent,'估计误差 0.030 m');
  assert.match(page.elements.get('poseHeading').textContent,/局部栅格/);
  assert.match(page.elements.get('localTaskPose').textContent,/已评分候选 7 · 地图版本 14/);
  assert.doesNotMatch(page.elements.get('localTaskPose').textContent,/A\*扩展/);
  assert.match(page.elements.get('localTaskState').textContent,/剩余 2\.00 s/);
});

test('grid packets decoded by the C++ gateway populate dashboard labels', async () => {
  const fixture=JSON.parse(fs.readFileSync(path.join(__dirname,'grid_telemetry_fixture.json'),'utf8'));
  const page=startDashboard();const socket=await openNetworkService(page);
  socket.message({v:1,type:'state',owner:null,armed:false,simulation:false,
    jetson:{online:true,age_ms:5},fpga:{ready:true},status:{bridge_id:'cpp-fixture'},
    navigation:{...fixture.navigation,task_id_watermark_known:true,age_ms:0,stale:false},video:{}});
  socket.message({v:1,type:'telemetry',bridge_id:'cpp-fixture',stream:'fusion',epoch:1,valid:true,data:fixture.fusion});
  page.runIntervals(250);
  assert.equal(page.elements.get('metricSigmaX').textContent,'估计误差 0.030 m');
  assert.match(page.elements.get('localTaskPose').textContent,/已评分候选 7 · 地图版本 42/);
  assert.match(page.elements.get('poseHeading').textContent,/局部栅格/);
});

test('grid planning shows a fresh map input cause or lost startup permission', async () => {
  const fixture=JSON.parse(fs.readFileSync(path.join(__dirname,'grid_telemetry_fixture.json'),'utf8'));
  const page=startDashboard();const socket=await openNetworkService(page);
  socket.message({v:1,type:'state',owner:null,armed:false,simulation:false,
    jetson:{online:true,age_ms:5},fpga:{ready:true},status:{bridge_id:'grid-causes'},
    navigation:{...fixture.navigation,status:2,task_id_watermark_known:true,age_ms:0,stale:false},video:{}});
  const telemetry=(sequence,overrides)=>socket.message({v:1,type:'telemetry',
    bridge_id:'grid-causes',stream:'fusion',epoch:1,valid:true,
    data:{...fixture.fusion,sequence,map_fresh:false,map_gap_latched:true,
      map_gap_reason_name:'fifo_overflow',...overrides}});
  telemetry(1,{});
  assert.match(page.elements.get('localTaskState').textContent,/等待新鲜局部地图（地图队列满）/);
  telemetry(2,{start_confirmation_lost:true});
  assert.match(page.elements.get('localTaskState').textContent,/启动区许可失效，等待新任务与地图恢复/);
  telemetry(3,{time_us:(fixture.navigation.time_us-600000)>>>0});
  assert.doesNotMatch(page.elements.get('localTaskState').textContent,/地图队列满/,
    'a different source time cannot explain the current planning state');
  assert.equal(socket.sent.filter(item=>item.type==='local_task').length,0);
});
