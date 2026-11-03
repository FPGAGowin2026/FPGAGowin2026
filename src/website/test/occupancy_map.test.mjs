import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import test from 'node:test';
import {createRequire} from 'node:module';
import {performance} from 'node:perf_hooks';
import {once} from 'node:events';
import {WebSocket} from 'ws';
import {DashboardBridge, createDashboardServer} from '../server.mjs';
const {decodeGrid} = createRequire(import.meta.url)('../public/js/occupancy_map.js');

const fusion = (stamp, x = 0, y = 0, yaw = 0) => ({flags:1, pose_stamp_us:stamp >>> 0,
  state:{x_m:x,y_m:y,yaw_rad:yaw,vx_m_s:0,vy_m_s:0,wz_rad_s:0}});
const lidar = (stamp, sequence, points) => ({time_us:stamp >>> 0,sequence,
  points:points.map(([angle_cdeg,distance_mm]) => ({angle_cdeg,distance_mm}))});

test('C++ decoded navigation and preview need a matching complete FPGA grid to appear on the map', () => {
  const fixture=JSON.parse(readFileSync(new URL('./grid_telemetry_fixture.json',import.meta.url),'utf8'));
  const bridge=new DashboardBridge({log:false});
  for (const stream of ['navigation','fusion','local_plan']) {
    bridge.acceptTelemetry({v:1,type:'telemetry',bridge_id:'cpp-fixture',stream,epoch:1,valid:true,
      jetson_rx_mono_s:1,jetson_tx_mono_s:1,data:fixture[stream]},10);
  }
  bridge.acceptTelemetry({v:1,type:'telemetry',bridge_id:'cpp-fixture',stream:'lidar',epoch:1,valid:true,
    jetson_rx_mono_s:1,jetson_tx_mono_s:1,data:{type:'lidar',...lidar(120000,1,[[0,2000]])}},11);
  assert.equal(bridge.mapping.snapshot(12).available,false,'navigation and lidar cannot create a substitute grid');
  const packed=Array(1024).fill(0);packed[0]=2;
  bridge.acceptTelemetry({v:1,type:'telemetry',bridge_id:'cpp-fixture',stream:'local_grid',epoch:1,valid:true,
    jetson_rx_mono_s:1,jetson_tx_mono_s:1,data:{type:'local_grid',protocol:'nav_grid_snapshot_v1',
      sequence:1,time_us:120000,boot_id:1,odom_domain:1,snapshot_id:1,capture_begin_us:120000,
      capture_end_us:120000,grid_generation:1,scan_generation:1,origin_x_cell:-32,origin_y_cell:-32,
      complete_scan_stamp_us:120000,last_ray_stamp_us:120000,grid_frame_epoch:1,transform_revision:1,
      flags:7,width:64,height:64,resolution_mm:50,packed_cells:packed,
      grid_from_odom:{map_anchor_x_mm:0,map_anchor_y_mm:0,odom_anchor_x_mm:0,odom_anchor_y_mm:0,yaw_offset_mrad:0}}},12);
  const snapshot=bridge.mapping.snapshot(12);
  assert.equal(snapshot.source,'fpga');
  assert.equal(snapshot.navigation_protocol,'nav_grid_v1');
  assert.equal(snapshot.planned_route.grid_direction_preview,true);
  assert.equal(snapshot.planned_route.map_version,42);
  assert.deepEqual(snapshot.planned_route.points,fixture.local_plan.points);
  assert.equal(snapshot.counts.occupied,1);
});

test('server publishes only a completely reassembled FPGA grid and /map remains read only', async () => {
  const bridge = await createDashboardServer({httpPort:0,udpBindHost:'127.0.0.1',jetsonPort:9,log:false});
  const port = bridge.http.address().port;
  const mapSocket = new WebSocket(`ws://127.0.0.1:${port}/map`, {origin:`http://127.0.0.1:${port}`});
  const frames = [];
  mapSocket.on('message', bytes => frames.push(JSON.parse(bytes.toString())));
  try {
    await once(mapSocket,'open');
    const at = performance.now();
    const feed = (stream,data) => bridge.acceptTelemetry({v:1,type:'telemetry',bridge_id:'map-test',
      epoch:1,valid:true,jetson_rx_mono_s:1,jetson_tx_mono_s:1,stream,
      data:{type:stream,sequence:1,time_us:100000,...data}},at);
    feed('navigation',{protocol:'nav_grid_v1',boot_id:1,odom_domain:1,last_task_id:5,task_id:5,status:3,now_us:100000});
    feed('fusion',fusion(100000));
    feed('local_plan',{protocol:'nav_local_segment_v1',boot_id:1,odom_domain:1,task_id:5,
      plan_stamp_us:100000,valid:true,guard_stop:true,grid_direction_preview:true,source:'fpga',
      points:[{x:0,y:0},{x:0.9,y:0}]});
    for (let n = 1; n <= 20; n++) feed('lidar',lidar(100000,n,[[n*500,2000]]));
    assert.equal(bridge.mapping.snapshot(at).available,false,'lidar never creates a replacement grid');
    const packed=Array(1024).fill(0);packed[0]=0b00100100;
    const grid={type:'local_grid',protocol:'nav_grid_snapshot_v1',sequence:1,time_us:100000,
      boot_id:1,odom_domain:1,snapshot_id:1,capture_begin_us:100000,capture_end_us:100000,
      grid_generation:1,scan_generation:1,origin_x_cell:-32,origin_y_cell:-32,
      complete_scan_stamp_us:100000,last_ray_stamp_us:100000,grid_frame_epoch:1,transform_revision:1,
      flags:7,width:64,height:64,resolution_mm:50,grid_from_odom:{map_anchor_x_mm:0,map_anchor_y_mm:0,
        odom_anchor_x_mm:0,odom_anchor_y_mm:0,yaw_offset_mrad:0}};
    const {type,sequence,time_us,flags,grid_from_odom,...metadata}=grid;
    const part=index=>({v:1,type:'telemetry',bridge_id:'map-test',epoch:1,valid:true,
      jetson_rx_mono_s:1,jetson_tx_mono_s:1,stream:'local_grid',part:index,parts:8,
      data:{type,sequence,time_us,flags,packed_cells:packed.slice(index*128,(index+1)*128),
        ...(index===0 ? {...metadata,grid_from_odom:{map_anchor_x_mm:0,map_anchor_y_mm:0}} : {}),
        ...(index===7 ? {grid_from_odom:{odom_anchor_x_mm:0,odom_anchor_y_mm:0,yaw_offset_mrad:0}} : {})}});
    for(let index=0;index<8;index++) {
      const fragment=part(index);
      if(index===4) fragment.data.grid_from_odom={...fragment.data.grid_from_odom,odom_anchor_x_mm:1};
      bridge.acceptTelemetry(fragment,at);
    }
    assert.equal(bridge.mapping.snapshot(at).available,false,'contradictory fragment metadata cannot form a grid');
    for(const index of [7,1,5,0,2,4,6]) bridge.acceptTelemetry(part(index),at);
    assert.equal(bridge.mapping.snapshot(at).available,false,'incomplete JSON fragments never enter the map');
    bridge.acceptTelemetry(part(3),at);
    assert.equal(bridge.mapping.snapshot(at).available,true);
    mapSocket.send(JSON.stringify({v:1,type:'arm'}));
    bridge.publishMap(); bridge.flushClients();
    await new Promise(resolve => setTimeout(resolve,80));
    assert.equal(bridge.owner,null);
    assert.ok(frames.some(frame => frame.source==='fpga' && frame.available===true));
    assert.ok(frames.some(frame => frame.planned_route?.task_id===5));
    const exported = await (await fetch(`http://127.0.0.1:${port}/map.json`)).json();
    assert.equal(exported.source,'fpga');
    assert.deepEqual(exported.planned_route.points,[{x:0,y:0},{x:0.9,y:0}]);
    assert.ok(exported.counts.occupied>0);
    const decoded=decodeGrid(exported);
    assert.equal(decoded.length,64*64);
    assert.deepEqual([...decoded.slice(0,4)],[0,1,2,0]);
    for (const bad of [
      {...exported,cells_rle:[0,1]},
      {...exported,cells_rle:[0,4097]},
      {...exported,cells_rle:[3,4096]},
      {...exported,protocol:'occupancy_odom_v1'},
    ]) assert.throws(()=>decodeGrid(bad),'invalid or obsolete grids must be rejected');
  } finally {
    mapSocket.close();
    await bridge.close();
  }
});
