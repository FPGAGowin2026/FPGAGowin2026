import assert from 'node:assert/strict';
import { EventEmitter } from 'node:events';
import { performance } from 'node:perf_hooks';
import test from 'node:test';
import { WebSocket } from 'ws';
import { DashboardBridge } from '../server.mjs';

class FakeSocket extends EventEmitter {
  constructor() { super(); this.readyState=WebSocket.OPEN; this.bufferedAmount=0; this.messages=[]; }
  send(text) { this.messages.push(JSON.parse(text)); }
  close(code,reason) { this.closed={code,reason}; this.readyState=WebSocket.CLOSED; }
}

function receive(bridge,socket,message) {
  bridge.onClientMessage(socket,Buffer.from(JSON.stringify({v:1,...message})),false);
}

test('delivery keeps one bounded batch in flight, latest samples, event order and private results', () => {
  const bridge=new DashboardBridge({log:false});
  const records=[], commands=[];
  bridge.log={write:value=>records.push(value)};
  bridge.sendMissionCommand=(_socket,action)=>commands.push(action);
  const owner=new FakeSocket(), observer=new FakeSocket();
  bridge.onWebSocket(owner,{},'json');
  bridge.onWebSocket(observer,{},'json'); // Legacy observer stays read-only.
  receive(bridge,owner,{type:'delivery_ready'});
  const client=bridge.clients.get(owner), now=performance.now();
  client.pendingState={};
  client.pendingTelemetry.set('fusion',{v:1,type:'telemetry',stream:'fusion',pc_rx_mono_ms:now,data:{sequence:1}});
  bridge.flushClients(now);
  const first=client.deliveryPending.id, count=owner.messages.length;
  for (let sequence=2; sequence<=300; sequence++)
    bridge.queueLatest('telemetry:fusion',{v:1,type:'telemetry',stream:'fusion',pc_rx_mono_ms:now,data:{sequence}});
  bridge.status={seq:10}; bridge.queueLatest('state');
  bridge.status={seq:11}; bridge.queueLatest('state');
  bridge.queueLatest('telemetry:lidar',{v:1,type:'telemetry',stream:'lidar',pc_rx_mono_ms:now-2000,data:{sequence:1}});
  bridge.emitEvent('test_event','important');
  bridge.deliverMissionResult({v:1,type:'mission_result',action:'local_start',success:false,
    message:'private-result',request_id:'private-id'},owner);
  bridge.flushClients(now+150);
  assert.equal(owner.messages.length,count,'no further batch is delivered before consumption acknowledgement');
  assert.equal(client.pendingTelemetry.size,2);
  assert.equal(client.pendingTelemetry.get('fusion').data.sequence,300);
  receive(bridge,owner,{type:'delivery_ack',id:first});
  bridge.flushClients(now+200);
  const sent=owner.messages.slice(count);
  assert.deepEqual(sent.map(item=>item.type),['event','mission_result','state','telemetry','delivery_probe']);
  assert.equal(sent[2].status.seq,11,'a delayed state reflects the latest bridge status');
  assert.equal(sent[3].data.sequence,300,'the latest unsent sample replaces the old samples');
  assert.equal(sent.some(item=>item.stream==='lidar'),false,'stale queued telemetry is discarded, not presented as fresh');
  assert.equal(observer.messages.some(item=>item.type==='mission_result'),false,'private results never reach observers');
  assert.ok(observer.messages.some(item=>item.event==='test_event'),'public events still reach the passive observer');
  const second=client.deliveryPending.id, acknowledgedAt=client.deliveryAckAt;
  receive(bridge,owner,{type:'delivery_ack',id:first});
  assert.equal(client.deliveryPending.id,second);
  assert.equal(client.deliveryAckAt,acknowledgedAt,'a duplicate ACK does not refresh consumption progress');
  bridge.missionOwner=owner;
  bridge.navigationTask={active:true};
  bridge.tick(now+701);
  assert.equal(owner.closed,undefined,'a brief page stall does not immediately cancel the task');
  bridge.tick(now+2201);
  assert.deepEqual(owner.closed,{code:1013,reason:'client consumption stalled'});
  assert.deepEqual(commands,['disable_ros'],'页面失联撤销视觉授权，不取消 Jetson 的导航任务');
  assert.equal(bridge.clients.has(observer),true,'a stalled owner never closes another monitor');
  assert.equal(records.find(item=>item.type==='client_disconnect').reason,'client consumption stalled');
});

test('transport high-water stalls are reported separately and never grow the JSON socket queue', () => {
  const bridge=new DashboardBridge({log:false}), logs=[];
  bridge.log={write:value=>logs.push(value)};
  const socket=new FakeSocket(); bridge.onWebSocket(socket,{},'json');
  const client=bridge.clients.get(socket), now=performance.now();
  client.pendingState={};
  socket.bufferedAmount=128*1024;
  const count=socket.messages.length;
  bridge.flushClients(now);
  bridge.flushClients(now+400);
  assert.equal(socket.messages.length,count);
  assert.equal(client.pendingState !== null,true);
  socket.bufferedAmount=0;
  bridge.tick(now+501);
  assert.equal(socket.closed,undefined,'a drained transport is not closed due to an old watermark timestamp');
  socket.bufferedAmount=128*1024;
  bridge.flushClients(now+600);
  bridge.tick(now+1101);
  assert.equal(socket.closed,undefined,'the temporary transport stall is within the delivery deadline');
  bridge.tick(now+2601);
  assert.deepEqual(socket.closed,{code:1013,reason:'client transport stalled'});
  assert.equal(logs.find(item=>item.type==='client_disconnect').reason,'client transport stalled');
});

test('each JSON batch stays below its byte budget when several large streams are ready', () => {
  const bridge=new DashboardBridge({log:false}), socket=new FakeSocket();
  bridge.onWebSocket(socket,{},'json');
  receive(bridge,socket,{type:'delivery_ready'});
  const client=bridge.clients.get(socket), now=performance.now();
  client.pendingState={};
  for (const stream of ['imu','motor','lidar','fusion','health','navigation'])
    client.pendingTelemetry.set(stream,{v:1,type:'telemetry',stream,pc_rx_mono_ms:now,data:{blob:'x'.repeat(40000)}});
  const before=socket.messages.length;
  bridge.flushClients(now);
  const batch=socket.messages.slice(before);
  const bytes=batch.reduce((total,message)=>total+Buffer.byteLength(JSON.stringify(message))+14,0);
  assert.ok(bytes < 128*1024);
  assert.ok(client.pendingTelemetry.size > 0,'oversized remainder waits for the next confirmed batch');
  assert.equal(batch.at(-1).type,'delivery_probe');
});
