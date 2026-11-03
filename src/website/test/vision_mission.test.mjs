import assert from 'node:assert/strict';
import test from 'node:test';
import { DashboardBridge } from '../server.mjs';

function harness() {
  const packets = [], results = [];
  const bridge = Object.assign(Object.create(DashboardBridge.prototype), {
    session:'web', seq:0, owner:null, missionOwner:null, vision:null,
    visionSequence:null, visionAt:null, bridgeId:'bridge', pendingMission:new Map(),
    target:{port:6200,address:'127.0.0.1'}, clients:new Map(),
    globalNavigation:{snapshot:()=>null,acceptResult:()=>false,clientClosed:()=>{}},
    udp:{address:()=>({port:10000}), send:(packet)=>packets.push(JSON.parse(packet))},
    emitEvent:()=>{}, queueLatest:()=>{}, jetsonOnline:()=>true,
    deliverMissionResult:result=>results.push(result),
  });
  return {bridge,packets,results};
}

function vision(overrides = {}) {
  const box = {id:'session:1',class_id:'0',score:0.9,cx:400,cy:240,width:100,height:180};
  return {v:1,type:'vision',session:'web',bridge_id:'bridge',seq:1,mission_state:'TRACKING',
    control_source:'ros',ros_authorized:true,emergency_stop:false,
    frame_id:'camera',stamp:{sec:1,nanosec:0},width:800,height:480,age_ms:50,
    stale:false,detections:[box],selected_target:box,truncated:false,...overrides};
}

test('monitor heartbeat never requests manual control; an explicit owner does', () => {
  const {bridge,packets} = harness();
  bridge.sendControl([0,0,0],false,'monitor');
  assert.equal(packets[0].take_control,false);
  bridge.owner={id:'operator'};
  bridge.sendControl([0,0,0],false,'manual-zero-handshake');
  assert.equal(packets[1].take_control,true);
  assert.ok(packets[1].seq > packets[0].seq);
});

test('vision age advances and old or malformed snapshots cannot refresh it', () => {
  const {bridge} = harness();
  bridge.acceptVision(vision(),100);
  assert.equal(bridge.visionState(200).stale,false);
  assert.equal(bridge.visionState(200).age_ms,150);
  bridge.acceptVision(vision(),450); // Repeated seq must not refresh receive time.
  assert.equal(bridge.visionAt,100);
  assert.deepEqual(bridge.visionState(700).detections,[]);
  assert.equal(bridge.visionState(700).selected_target,null);
  bridge.acceptVision(vision({seq:2,bridge_id:'retired'}),710);
  assert.equal(bridge.visionSequence,1);
  bridge.acceptVision(vision({seq:2,age_ms:-1}),710);
  assert.equal(bridge.visionSequence,1);
  bridge.acceptVision(vision({seq:2,stale:true,age_ms:0}),720);
  assert.equal(bridge.visionState(720).selected_target,null);
});

test('live 640x360 FPGA observations reach the dashboard with matching pixel bounds', () => {
  const {bridge} = harness();
  bridge.acceptVision(vision({width:640,height:360}),100);
  assert.equal(bridge.visionState(100).width,640);
  const outside={id:'outside',class_id:'0',score:0.9,cx:700,cy:240,width:10,height:10};
  bridge.acceptVision(vision({seq:2,width:640,height:360,detections:[outside]}),110);
  assert.equal(bridge.visionSequence,1);
});

test('mission results match the pending session, action and request id', () => {
  const {bridge,results} = harness();
  bridge.pendingMission.set('request',{session:'web',action:'start',at:100});
  const response={v:1,type:'mission_result',session:'old',request_id:'request',
    action:'start',success:true,message:'accepted'};
  bridge.acceptMissionResult(response);
  assert.equal(bridge.pendingMission.size,1);
  bridge.acceptMissionResult({...response,session:'web',action:'stop'});
  assert.equal(bridge.pendingMission.size,1);
  bridge.acceptMissionResult({...response,session:'web'});
  assert.equal(bridge.pendingMission.size,0);
  assert.equal(results.length,1);
  bridge.acceptMissionResult({...response,session:'web'});
  assert.equal(results.length,1);
  bridge.pendingMission.set('unknown',{session:'web',action:'enable_ros',at:200});
  bridge.clearPendingMission('目标变化，结果未知');
  assert.equal(results[1].unknown,true);
  assert.equal(results[1].success,false);
  assert.equal(bridge.pendingMission.size,0);
});

test('search-only is forwarded unchanged and FOUND preserves the observation for handoff', () => {
  const {bridge,packets} = harness();
  bridge.acceptVision(vision({mission_state:'IDLE',selected_target:null}),performance.now());
  const requestId = bridge.sendMissionCommand(null,'start',{
    behavior:'search_only',target_id:'',class_id:'39',color:'red',
  });
  assert.ok(requestId);
  assert.equal(packets[0].behavior,'search_only');
  assert.equal(packets[0].target_id,'');
  assert.equal(packets[0].class_id,'39');
  bridge.acceptVision(vision({seq:2,mission_state:'FOUND'}),performance.now());
  const found=bridge.visionState();
  assert.equal(found.mission_state,'FOUND');
  assert.equal(found.selected_target.id,'session:1');
  assert.deepEqual(found.stamp,{sec:1,nanosec:0});
});

test('closing the explicit task controller revokes ROS; a monitor does not', () => {
  const {bridge,packets} = harness();
  const controller={}, monitor={};
  bridge.clients.set(controller,{kind:'json'});
  bridge.clients.set(monitor,{kind:'json'});
  bridge.missionOwner=controller;
  bridge.onClientClose(monitor);
  assert.equal(packets.length,0);
  bridge.onClientClose(controller);
  assert.equal(packets[0].type,'mission_command');
  assert.equal(packets[0].action,'disable_ros');
});
