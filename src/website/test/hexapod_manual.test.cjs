const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const test = require('node:test');
const vm = require('node:vm');

const root = path.join(__dirname, '..', 'public');
const html = fs.readFileSync(path.join(root, 'index.html'), 'utf8');
const actions = ['turn_left', 'forward', 'turn_right', 'strafe_left', 'backward', 'strafe_right'];
const flush = async () => { for (let i=0; i<40; i++) await Promise.resolve(); };

async function dashboard() {
  const listeners = () => ({handlers:{},
    addEventListener(type, fn) { (this.handlers[type] ||= []).push(fn); },
    emit(type, event={}) { for (const fn of this.handlers[type] || []) fn(event); }});
  const element = id => {
    const classes = new Set();
    return Object.assign(listeners(), {id,tagName:'BUTTON',hidden:false,disabled:false,open:false,
      checked:false,dataset:{},attributes:{},textContent:'',
      classList:{add:v=>classes.add(v),remove:v=>classes.delete(v),contains:v=>classes.has(v)},
      setPointerCapture(){},setAttribute(key,value){this.attributes[key]=value;}});
  };
  const elements = Object.fromEntries([...html.matchAll(/\bid="([^"]+)"/g)].map(m => [m[1],element(m[1])]));
  const keys = {acg720:actions.map(action =>
    Object.assign(element(`acg720-${action}`), {dataset:{acg720Action:action}}))};
  const document = Object.assign(listeners(), {hidden:false,getElementById:id=>elements[id],
    querySelectorAll:selector=>selector === '[data-acg720-action]' ? keys.acg720 : []});
  const window = Object.assign(listeners(), {dispatchEvent(){}});
  let now=10000;
  const state = {
    acg720:{robot_id:'acg720',fpga_connected:true,teleop_state:'READY',teleop_age_s:.05,
      pwm_enabled:false,requested_command:0,active_gait:0,frame_index:0,
      ripple_selected:false,trajectory_ripple:false,homing:false,teleop_status_version:1,
      imu_ready:true,lidar_state:'READY',lidar_points:720,camera_ready:true,
      preview_version:1,complete_frames:1,navigation_reason:'等待定位',error:null},
    acgAvailable:true,delay:null,error:null
  };
  const calls=[], intervals=new Map(), timeouts=new Map();let timer=0;
  const response=(value,status=200)=>({ok:status>=200&&status<300,status,json:async()=>value});
  const fetch=async(url,options={})=>{
    if(options.method==='POST'){
      const call={url,body:JSON.parse(options.body)};calls.push(call);
      if(state.delay?.(call)) return new Promise(resolve=>{call.resolve=()=>resolve(response({accepted:true,delivery:'sent',acknowledged:false}));});
      const error=state.error?.(call);
      return error ? response({accepted:false,error},503) :
        response({accepted:true,delivery:'sent',acknowledged:false,status:{ripple_selected:true}});
    }
    if(url==='/api/fleet') return response({members:[{id:'car',name:'BX72 小车',host:'192.0.2.103'},
      {id:'acg720',name:'ACG720 六足',host:'192.0.2.54'}],acg720_ros:{age_ms:20}});
    if(url==='/acg720/api/status') return response({...state.acg720},state.acgAvailable?200:503);
    throw Error(`unexpected URL ${url}`);
  };
  class Clock extends Date { static now(){return now;} }
  const sandbox={document,window,fetch,Date:Clock,Math,console,AbortController,AbortSignal,
    CustomEvent:class{constructor(type, options){this.type=type;this.detail=options?.detail;}},
    setInterval:(fn,ms)=>{const id=++timer;intervals.set(id,{fn,ms});return id;},
    clearInterval:id=>intervals.delete(id),
    setTimeout:(fn,ms)=>{const id=++timer;timeouts.set(id,{fn,ms});return id;},
    clearTimeout:id=>timeouts.delete(id)};
  for(const file of ['hexapod_manual.js','acg720_app.js'])
    vm.runInNewContext(fs.readFileSync(path.join(root,'js',file),'utf8'),sandbox,{filename:file});
  await flush();
  const poll=async(ms=1000)=>{
    now+=ms;
    const scheduled=[...timeouts].filter(([,entry])=>entry.ms===1000);
    for(const [id,entry] of scheduled){timeouts.delete(id);entry.fn();}
    for(const entry of [...intervals.values()]) if(entry.ms===1000) entry.fn();
    await flush();
  };
  const renew=async()=>{for(const entry of [...intervals.values()])if(entry.ms===200)entry.fn();await flush();};
  const key=(type,key,other={})=>document.emit(type,{key,code:key===' '?'Space':`Key${key.toUpperCase()}`,
    target:{tagName:'BODY'},preventDefault(){},...other});
  const pointer=(id,action,type='pointerdown')=>keys[id][actions.indexOf(action)].emit(type,{preventDefault(){},pointerId:1});
  return {elements,keys,window,document,state,calls,poll,renew,key,pointer,
    manager:window.HexapodManualControls,advance:ms=>{now+=ms;}};
}

test('ACG720 held leases share a single keyboard owner with the car',async()=>{
  const d=await dashboard(), {elements:e,calls}=d;
  assert.equal(calls.length,0,'page load and status never send commands');
  assert.match(html,/ACG720 六足/);assert.doesNotMatch(html,/Tang Primer|Orange Pi|data-hex-action/);
  assert.equal(e.acg720Arm.disabled,false);
  e.acg720Arm.onclick();d.pointer('acg720','forward');await flush();await d.renew();
  assert.equal(calls.length,2);assert.ok(calls.every(c=>c.url==='/acg720/api/control'&&c.body.action==='forward'&&c.body.pressed));
  const client=calls[0].body.client_id;let carStops=0;
  d.manager.registerCar(()=>carStops++);d.manager.selectCar();await flush();
  assert.equal(calls.at(-1).body.pressed,false);assert.equal(calls.at(-1).body.client_id,client);
  assert.equal(d.keys.acg720[1].disabled,true);assert.equal(e.manualTarget.textContent,'BX72 小车');
  const switched=calls.length;await d.renew();assert.equal(calls.length,switched,'previous target no longer renews');
  e.acg720Arm.onclick();assert.equal(carStops,1);assert.equal(e.manualTarget.textContent,'ACG720 六足');
  d.key('keydown','w');await flush();await d.renew();d.key('keyup','w');await flush();
  assert.deepEqual(calls.slice(switched).map(c=>[c.body.action,c.body.pressed]),
    [['forward',true],['forward',true],['forward',false]]);
  assert.ok(calls.every(c=>c.url==='/acg720/api/control'));
  assert.equal(calls[switched].body.client_id,client);
  assert.equal(calls[switched].body.lease_ms,500);
  assert.ok(calls[switched+2].body.seq>calls[switched].body.seq);
  assert.match(e.acg720Preview.src,/^\/acg720\/frame.jpg\?v=1$/);
});

test('ACG720 HXS1 readiness distinguishes stop, release, modes and observed execution',async()=>{
  const d=await dashboard(), {elements:e,state,calls}=d;
  assert.equal(e.acg720Arm.disabled,false);
  e.acg720Arm.onclick();e.acg720Ripple.onclick();await flush();
  assert.deepEqual(calls.map(c=>[c.url,c.body.action]),[['/acg720/api/control','ripple']]);
  assert.match(e.acg720ControlResult.textContent,/已发送.*HXS1/);
  assert.equal(e.acg720Ripple.attributes['aria-pressed'],'false','send success does not invent device execution');
  assert.equal(e.acg720ManualPwm.textContent,'已关闭');
  state.acg720.ripple_selected=true;state.acg720.teleop_status_version=2;await d.poll();
  assert.equal(e.acg720Ripple.attributes['aria-pressed'],'true');
  assert.match(e.acg720Mode.textContent,/所选 涟漪 · 当前轨迹 标准三角/);
  e.acg720Stand.onclick();await flush();assert.equal(calls.at(-1).body.action,'stand');
  e.acg720Stop.onclick();await flush();assert.equal(calls.at(-1).body.action,'stop');
  assert.equal(d.keys.acg720[1].disabled,true);assert.equal(e.acg720ManualPwm.textContent,'已关闭');
  e.acg720Release.onclick();await flush();assert.equal(calls.at(-1).body.action,'release');
  assert.ok(calls.every(c=>c.url==='/acg720/api/control'));
});

test('status expiry, focus loss and input fields cannot keep a held target moving',async()=>{
  const d=await dashboard(),{elements:e,calls,state}=d;
  e.acg720Arm.onclick();d.key('keydown','w',{target:{tagName:'INPUT'}});d.key('keydown','w',{repeat:true});
  assert.equal(calls.length,0);
  d.key('keydown','w');await flush();
  d.advance(1600);await d.renew();
  assert.equal(calls.at(-1).body.pressed,false);assert.equal(d.keys.acg720[1].disabled,true);
  assert.equal(d.manager.keyboardOwner(),null);
  await d.poll();e.acg720Arm.onclick();d.pointer('acg720','strafe_left');await flush();
  d.window.emit('blur');await flush();
  assert.equal(calls.at(-1).body.action,'strafe_left');assert.equal(calls.at(-1).body.pressed,false);
  const blurred=calls.length;await d.renew();assert.equal(calls.length,blurred);
  let carStops=0;d.manager.registerCar(()=>carStops++);d.manager.selectCar();
  state.acgAvailable=false;await d.poll();
  assert.equal(d.manager.keyboardOwner(),'car','ACG720 outage does not change the car keyboard target');
  assert.equal(carStops,0);
  state.acgAvailable=true;await d.poll();e.acg720Arm.onclick();d.pointer('acg720','turn_right');await flush();
  d.document.hidden=true;d.document.emit('visibilitychange');await flush();
  assert.equal(calls.at(-1).url,'/acg720/api/control');assert.equal(calls.at(-1).body.pressed,false);
  assert.equal(d.manager.keyboardOwner(),null);
});

test('late movement and errors stay on their original target after selection changes',async()=>{
  const d=await dashboard(),{elements:e,calls,state}=d;
  state.delay=c=>c.url==='/acg720/api/control'&&c.body.action==='forward'&&c.body.pressed;
  e.acg720Arm.onclick();d.pointer('acg720','forward');await flush();
  const pending=calls[0];
  d.manager.selectCar();await flush();
  assert.equal(calls.at(-1).url,'/acg720/api/control');assert.equal(calls.at(-1).body.pressed,false);
  const selectedCount=calls.length;d.key('keydown','q');await flush();
  assert.equal(calls.length,selectedCount,'ACG720 does not receive keys while the car is selected');
  pending.resolve();await flush();
  assert.equal(calls.at(-1).url,'/acg720/api/control');assert.equal(calls.at(-1).body.pressed,false,'final release follows delayed movement');
  state.error=c=>c.url==='/acg720/api/control'?'ACG720 下发失败':null;
  e.acg720Arm.onclick();await flush();d.pointer('acg720','backward');await flush();
  assert.match(e.acg720ControlResult.textContent,/动作请求失败：ACG720 下发失败/);
  assert.equal(d.keys.acg720[4].disabled,true);assert.doesNotMatch(e.networkEvent.textContent,/ACG720/);
});

test('selecting car and ACG720 releases only the previous selected owner',async()=>{
  const d=await dashboard(),{elements:e,calls}=d;let carStops=0;
  d.manager.registerCar(()=>carStops++);
  d.manager.selectCar();assert.equal(e.manualTarget.textContent,'BX72 小车');
  e.acg720Arm.onclick();assert.equal(carStops,1);assert.equal(calls.length,0,'selecting an idle ACG720 issues no command');
  d.key('keydown','s');await flush();
  d.manager.selectCar();await flush();
  assert.equal(d.keys.acg720[4].disabled,true);assert.equal(calls.at(-1).body.pressed,false);
  const after=calls.length;d.key('keydown','w');await d.renew();assert.equal(calls.length,after);
  d.manager.releaseCar();assert.equal(d.manager.keyboardOwner(),null);
});

test('stand waits for delayed motion release while stop and PWM release remain the final command',async()=>{
  for(const [action,button] of [['stand','acg720Stand'],['stop','acg720Stop'],['release','acg720Release']]){
    const d=await dashboard(),{elements:e,calls,state}=d;
    state.delay=c=>c.body.action==='forward'&&c.body.pressed;
    e.acg720Arm.onclick();d.pointer('acg720','forward');await flush();
    const pending=calls[0];e[button].onclick();await flush();
    assert.equal(calls[1].body.pressed,false);
    assert.equal(calls.some(c=>c.body.action===action),action!=='stand',`${action} initial ordering`);
    pending.resolve();await flush();
    assert.equal(calls.at(-1).body.action,action,`${action} must follow the final neutral release`);
    assert.equal(calls.at(-1).body.pressed,true);
    assert.equal(calls.filter(c=>c.body.action===action).length,action==='stand'?1:2);
    assert.ok(calls.every(c=>c.url==='/acg720/api/control'));
  }
});

test('release before a new held action drains old requests without clearing the new pending identity',async()=>{
  const d=await dashboard(),{elements:e,calls,state}=d;
  state.delay=c=>c.body.pressed&&['forward','backward'].includes(c.body.action);
  e.acg720Arm.onclick();d.pointer('acg720','forward');await flush();
  const old=calls[0];d.pointer('acg720','forward','pointerup');d.pointer('acg720','backward');await flush();
  assert.equal(calls.some(c=>c.body.action==='backward'),false,'new movement waits for the final old release');
  old.resolve();await flush();
  const next=calls.at(-1);assert.equal(next.body.action,'backward');assert.equal(next.body.pressed,true);
  const busyCount=calls.length;await d.renew();assert.equal(calls.length,busyCount,'old finally cannot clear a new pending request');
  next.resolve();await flush();await d.renew();
  assert.equal(calls.length,busyCount+1);assert.equal(calls.at(-1).body.action,'backward');
  calls.at(-1).resolve();await flush();d.pointer('acg720','backward','pointerup');await flush();
  assert.equal(calls.at(-1).body.pressed,false);
});

test('mode selection preserves the held direction and switching targets cancels a queued stand',async()=>{
  const d=await dashboard(),{elements:e,calls,state}=d;
  e.acg720Arm.onclick();d.pointer('acg720','forward');await flush();
  e.acg720Ripple.onclick();await flush();await d.renew();
  assert.deepEqual(calls.map(c=>[c.body.action,c.body.pressed]),[['forward',true],['ripple',true],['forward',true]]);
  d.pointer('acg720','forward','pointerup');await flush();
  state.delay=c=>c.url==='/acg720/api/control'&&c.body.action==='forward'&&c.body.pressed;
  d.pointer('acg720','forward');await flush();
  const pending=calls.at(-1);e.acg720Stand.onclick();d.manager.selectCar();await flush();
  pending.resolve();await flush();
  assert.equal(calls.some(c=>c.body.action==='stand'),false,'a former target cannot start a queued pose after selection changes');
  assert.equal(d.manager.keyboardOwner(),'car');
});

test('ACG720 disconnect and control failures retain explicit release semantics',async()=>{
  const d=await dashboard(),{elements:e,calls,state}=d;
  e.acg720Arm.onclick();d.pointer('acg720','forward');await flush();
  state.acgAvailable=false;await d.poll(300);
  assert.equal(d.keys.acg720[1].disabled,true);
  assert.equal(calls.at(-1).body.pressed,false,'disconnect releases the held action');
  assert.ok(calls.every(c=>c.body.action!=='stand'),'disconnect does not request recovery');
  state.acgAvailable=true;await d.poll(300);e.acg720Arm.onclick();
  state.error=c=>!c.body.pressed?'UDP 松开失败':null;
  d.pointer('acg720','forward');await flush();d.pointer('acg720','forward','pointerup');await flush();
  assert.match(e.acg720ControlResult.textContent,/松开请求失败：UDP 松开失败/);
  state.error=c=>c.body.pressed?'UDP 动作失败':'FPGA 状态断流';
  d.pointer('acg720','forward');await flush();
  assert.equal(calls.at(-1).body.pressed,false,'failed movement still requests an ordinary release');
  assert.match(e.acg720ControlResult.textContent,/动作请求失败：UDP 动作失败/);
  assert.match(e.acg720ControlResult.textContent,/松开请求失败：FPGA 状态断流/);
  assert.equal(d.keys.acg720[1].disabled,true);
  assert.ok(calls.every(c=>c.url==='/acg720/api/control'));
});
