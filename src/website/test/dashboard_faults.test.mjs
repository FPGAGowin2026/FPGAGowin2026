// 真实 localhost UDP 故障注入：丢包期间丢弃残帧，恢复后接受乱序/重复分片。
import assert from 'node:assert/strict';
import { once } from 'node:events';
import { createRequire } from 'node:module';
import { setTimeout as delay } from 'node:timers/promises';
import { createSimulator } from '../simulator.mjs';
import { createDashboardServer } from '../server.mjs';
const require=createRequire(new URL('../package.json',import.meta.url));
const WebSocket=require('ws');
async function until(predicate,label,timeout=6000) {
  const end=Date.now()+timeout;
  while(Date.now()<end) { if(predicate())return; await delay(10); }
  throw Error(`等待超时：${label}`);
}
const simulator=await createSimulator({port:0,quiet:true,dropEvery:7,duplicateEvery:11,reorderEvery:13});
let bridge,control,video;
let frames=0;
const sequences=new Map();
try {
  bridge=await createDashboardServer({httpPort:0,jetsonPort:simulator.socket.address().port,simulation:true,log:false});
  const port=bridge.http.address().port,origin=`http://127.0.0.1:${port}`;
  control=new WebSocket(`ws://127.0.0.1:${port}/ws`,{origin});
  control.on('message',bytes=>{
    const value=JSON.parse(bytes.toString());
    if(value.type!=='telemetry')return;
    const seq=value.data.sequence,previous=sequences.get(value.stream);
    if(previous!==undefined)assert.ok(((seq-previous)>>>0)>0 && ((seq-previous)>>>0)<0x80000000,'重复/乱序不得更新遥测');
    sequences.set(value.stream,seq);
  });
  await once(control,'open');
  video=new WebSocket(`ws://127.0.0.1:${port}/video`,{origin});
  video.on('message',bytes=>{ assert.equal(bytes.readUInt16BE(0),0xffd8); assert.equal(bytes.readUInt16BE(bytes.length-2),0xffd9); frames++; });
  await once(video,'open');
  await until(()=>sequences.size===6 && bridge.jpeg.dropped>0,'六类遥测及丢包残帧清理');
  await delay(400);
  assert.equal(frames,0,'每帧均有丢片时不得把不完整JPEG交付网页');
  const lossPhaseDropped=bridge.jpeg.dropped;
  simulator.options.dropEvery=0;
  await until(()=>frames>=3,'保留重复/乱序注入而恢复完整视频');
  assert.ok(bridge.jpeg.duplicates>0);
  assert.equal(bridge.state().armed,false,'故障恢复不得自动授权');
  console.log(JSON.stringify({result:'PASS',actual_udp_fault_injection:true,software_simulation_only:true,streams:[...sequences.keys()],loss_phase_incomplete_frames_dropped:lossPhaseDropped,duplicate_fragments_filtered:bridge.jpeg.duplicates,recovered_complete_frames:frames,recovery_armed:false},null,2));
} finally {
  control?.terminate(); video?.terminate();
  if(bridge)await bridge.close();
  await simulator.close();
}
