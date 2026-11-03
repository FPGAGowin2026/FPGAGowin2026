import test from 'node:test';
import assert from 'node:assert/strict';
import {FpgaGridMap,gridFromOdom} from '../fpga_grid_map.mjs';

const transform={map_anchor_x_mm:0,map_anchor_y_mm:0,odom_anchor_x_mm:0,odom_anchor_y_mm:0,yaw_offset_mrad:0};
const navigation=(now=1000000,extra={})=>({type:'navigation',protocol:'nav_grid_v1',boot_id:1,odom_domain:2,
  task_id:7,status:3,now_us:now>>>0,...extra});
const fusion=(now=1000000,extra={})=>({type:'fusion',flags:1,pose_stamp_us:now>>>0,
  state:{x_m:0,y_m:0,yaw_rad:0},...extra});
const grid=(id=1,time=1000000,extra={})=>({type:'local_grid',protocol:'nav_grid_snapshot_v1',
  sequence:id,time_us:time>>>0,boot_id:1,odom_domain:2,snapshot_id:id,
  capture_begin_us:time>>>0,capture_end_us:time>>>0,grid_generation:id,scan_generation:1,
  origin_x_cell:-33,origin_y_cell:-2,complete_scan_stamp_us:time>>>0,last_ray_stamp_us:time>>>0,
  grid_frame_epoch:1,transform_revision:1,flags:7,width:64,height:64,resolution_mm:50,
  packed_cells:Array(1024).fill(0),grid_from_odom:{...transform},...extra});
function ready(m,at=1000,time=1000000) {m.consume('navigation',navigation(time),at);m.consume('fusion',fusion(time),at);}
function decode(runs) {const values=[];for(let i=0;i<runs.length;i+=2)values.push(...Array(runs[i+1]).fill(runs[i]));return values;}

test('Jetson grid uses the same map contract and changing planner clears the previous source',()=>{
  const m=new FpgaGridMap();ready(m);m.consume('local_grid',grid(),1000);
  m.consume('navigation',navigation(1020000,{protocol:'jetson_nav_v1'}),1020);
  assert.equal(m.snapshot(1020).available,false);
  m.consume('fusion',fusion(1020000),1020);
  m.consume('local_grid',grid(2,1020000),1020);
  assert.equal(m.snapshot(1020).available,false,'old FPGA grids cannot refresh a Jetson navigation session');
  m.consume('local_grid',grid(2,1020000,{protocol:'jetson_grid_snapshot_v1',source:'jetson'}),1020);
  assert.equal(m.snapshot(1020).source,'jetson');
  assert.equal(m.snapshot(1020).protocol,'jetson_grid_view_v1');
  assert.equal(m.snapshot(1020).status,'building');
});

test('only complete FPGA grid snapshots become the map with exact 2-bit order and negative origin',()=>{
  const m=new FpgaGridMap();ready(m);
  m.consume('lidar',{time_us:1000000,points:[{angle_cdeg:0,distance_mm:1000}]},1000);
  assert.equal(m.snapshot(1000).available,false,'lidar cannot manufacture an FPGA grid');
  const g=grid();g.packed_cells[0]=0b00100100;g.packed_cells[16]=2;
  m.consume('local_grid',g,1000);
  const s=m.snapshot(1000),cells=decode(s.cells_rle);
  assert.equal(s.source,'fpga');assert.equal(s.protocol,'fpga_grid_view_v1');
  assert.deepEqual(cells.slice(0,4),[0,1,2,0]);assert.equal(cells[64],2);
  assert.ok(Math.abs(s.origin.x+1.65)<1e-12 && Math.abs(s.origin.y+.1)<1e-12);
  assert.equal(s.width,64);assert.equal(s.height,64);assert.equal(s.resolution_m,.05);
  assert.equal(s.counts.occupied,2);assert.equal(s.status,'building');
});

test('FPGA obstacle cells already include the sensor offset, while raw lidar stays unprojected',()=>{
  const m=new FpgaGridMap();ready(m);
  const g=grid();
  // 雷达前方1米回波距离车体中心1.13米，位于1.10至1.15米的格内。
  const row=2,col=55,index=row*64+col;
  g.packed_cells[index>>2] |= 2 << (2*(index&3));
  m.consume('lidar',{time_us:1000000,points:[{angle_cdeg:0,distance_mm:1000}]},1000);
  m.consume('local_grid',g,1000);
  const snapshot=m.snapshot(1000),cells=decode(snapshot.cells_rle);
  assert.equal(snapshot.pose.x,0);
  assert.equal(snapshot.pose.y,0);
  assert.equal(cells[index],2);
  assert.ok(Math.abs(snapshot.origin.x+col*snapshot.resolution_m-1.10)<1e-12);
  assert.equal(cells[index+3],0,'the already transformed cell is not shifted forward again');
});

test('malformed, mixed-domain, duplicate and old snapshots cannot refresh the last real map',()=>{
  const m=new FpgaGridMap();ready(m);m.consume('local_grid',grid(),1000);
  for(const bad of [grid(2,1100000,{packed_cells:Array(1023).fill(0)}),
    grid(2,1100000,{packed_cells:[3,...Array(1023).fill(0)]}),
    grid(2,1100000,{odom_domain:3}),grid(2,1100000,{resolution_mm:100}),
    grid(2,1100000,{sequence:3}),grid(2,1100000,{flags:32})]) m.consume('local_grid',bad,1100);
  m.consume('local_grid',grid(),1400);
  m.consume('local_grid',grid(0,900000),1400);
  assert.equal(m.snapshot(1400).revision,1);
  m.consume('navigation',navigation(1750000),1750);m.consume('fusion',fusion(1750000),1750);
  assert.equal(m.snapshot(1750).status,'paused');
  assert.match(m.snapshot(1750).reason,/快照已过期/);
});

test('a future capture waits for FPGA navigation time; flags and scan ages remain distinct',()=>{
  const m=new FpgaGridMap();ready(m);m.consume('local_grid',grid(1,1020000),1000);
  assert.equal(m.snapshot(1000).available,false);
  m.consume('navigation',navigation(1020000),1020);m.consume('fusion',fusion(1020000),1020);
  assert.equal(m.snapshot(1020).status,'building');
  m.consume('navigation',navigation(1040000),1040);m.consume('local_grid',grid(2,1040000,{flags:11}),1040);
  let s=m.snapshot(1040);assert.equal(s.grid_fresh,true);assert.equal(s.grid_ready,false);assert.equal(s.grid_gap,true);
  assert.equal(s.status,'paused');
  m.consume('navigation',navigation(1060000),1060);m.consume('local_grid',grid(3,1060000,{flags:5}),1060);
  s=m.snapshot(1060);assert.equal(s.grid_fresh,false);assert.equal(s.grid_ready,true);assert.equal(s.grid_gap,false);
});

test('one control cycle of fusion lead remains fresh without extending the pose receipt timeout',()=>{
  const m=new FpgaGridMap(),now=1000000;
  ready(m,1000,now);m.consume('local_grid',grid(1,now),1000);
  const pose=fusion(now+20000,{state:{x_m:.1,y_m:.2,yaw_rad:.3}});
  m.consume('fusion',pose,1000);
  let snapshot=m.snapshot(1000);
  assert.equal(m.pose.stamp,now+20000,'source timestamp remains unchanged');
  assert.equal(snapshot.status,'building');
  assert.equal(snapshot.pose_fresh,true);
  assert.equal(snapshot.pose_age_ms,0);
  assert.deepEqual(snapshot.pose_odom,{x:.1,y:.2,yaw:.3});
  snapshot=m.snapshot(1500);
  assert.equal(snapshot.pose_fresh,true,'the existing 500 ms boundary remains inclusive');
  assert.equal(snapshot.pose_age_ms,500);
  snapshot=m.snapshot(1501);
  assert.equal(snapshot.pose_fresh,false,'no fusion updates cannot be revived by the clock tolerance');
  assert.equal(snapshot.pose_age_ms,501);
  assert.equal(snapshot.status,'paused');
});

test('future or invalid fusion frames preserve the last trusted pose without renewing it',()=>{
  const m=new FpgaGridMap(),now=1000000;
  ready(m,1000,now);m.consume('local_grid',grid(1,now),1000);
  const original=m.pose,receivedAt=m.poseAt;
  m.consume('fusion',fusion(now+20001,{state:{x_m:9,y_m:0,yaw_rad:0}}),1200);
  m.consume('fusion',fusion(now+10000,{flags:0}),1300);
  m.consume('fusion',fusion(now+10000,{state:{x_m:NaN,y_m:0,yaw_rad:0}}),1400);
  assert.strictEqual(m.pose,original);
  assert.equal(m.poseAt,receivedAt);
  assert.equal(m.pose.stamp,now);
  let snapshot=m.snapshot(1500);
  assert.equal(snapshot.pose_fresh,true);
  assert.equal(snapshot.pose_age_ms,500);
  assert.deepEqual(snapshot.pose_odom,{x:0,y:0,yaw:0});
  snapshot=m.snapshot(1501);
  assert.equal(snapshot.pose_fresh,false,'discarded frames cannot extend the existing 500 ms deadline');
  assert.equal(snapshot.pose_age_ms,501);
  assert.equal(snapshot.status,'paused');
});

test('navigation cannot revive a rejected future fusion frame; a new valid frame is required',()=>{
  const m=new FpgaGridMap(),now=1000000;
  ready(m,1000,now);m.consume('local_grid',grid(1,now),1000);
  const ahead=fusion(now+20001,{state:{x_m:9,y_m:0,yaw_rad:0}});
  m.consume('fusion',ahead,1001);
  assert.equal(m.pose.stamp,now);
  m.consume('navigation',navigation(now+1),1010);
  let snapshot=m.snapshot(1010);
  assert.equal(snapshot.pose_fresh,true);
  assert.equal(snapshot.pose_odom.x,0,'only the original pose remains after navigation advances');
  assert.equal(m.poseAt,1000,'navigation cannot refresh the original receipt time');
  m.consume('fusion',ahead,1011);
  snapshot=m.snapshot(1011);
  assert.equal(snapshot.pose_odom.x,9,'a newly received frame at the 20 ms limit is accepted');
  assert.equal(m.poseAt,1011);
  assert.equal(m.pose.stamp,now+20001);
});

test('the fusion lead boundary uses the wrapping FPGA microsecond clock',()=>{
  const m=new FpgaGridMap(),now=0xfffffff0;
  ready(m,1000,now);m.consume('local_grid',grid(1,now),1000);
  const boundary=(now+20000)>>>0;
  m.consume('fusion',fusion(boundary),1000);
  assert.equal(m.pose.stamp,boundary);
  assert.equal(m.snapshot(1000).pose_fresh,true);
  m.consume('fusion',fusion((boundary+1)>>>0),1001);
  assert.equal(m.pose.stamp,boundary,'a 20.001 ms lead is discarded across wrap');
  assert.equal(m.poseAt,1000);
  assert.equal(m.snapshot(1001).pose_fresh,true);
  m.consume('navigation',navigation((now+1)>>>0),1002);
  assert.equal(m.pose.stamp,boundary,'navigation cannot revive a discarded frame');
  m.consume('fusion',fusion((boundary+1)>>>0),1003);
  assert.equal(m.pose.stamp,(boundary+1)>>>0,'a new frame is valid after the source clock catches up');
});

test('CSM rebase changes pose and route coordinates without changing the grid-frame epoch',()=>{
  const m=new FpgaGridMap();ready(m);
  m.consume('fusion',fusion(1000000,{state:{x_m:1,y_m:2,yaw_rad:.3}}),1000);
  const a={map_anchor_x_mm:1000,map_anchor_y_mm:2000,odom_anchor_x_mm:1000,odom_anchor_y_mm:2000,yaw_offset_mrad:0};
  m.consume('local_grid',grid(1,1000000,{grid_from_odom:a}),1000);
  m.consume('local_plan',{type:'local_plan',protocol:'nav_local_segment_v1',boot_id:1,odom_domain:2,
    task_id:7,plan_stamp_us:1000000,valid:true,guard_stop:false,grid_direction_preview:true,source:'fpga',
    points:[{x:1,y:2},{x:2,y:2}]},1000);
  assert.equal(m.snapshot(1000).planned_route.points[1].x,2);
  const b={map_anchor_x_mm:1000,map_anchor_y_mm:2000,odom_anchor_x_mm:500,odom_anchor_y_mm:500,yaw_offset_mrad:1571};
  m.consume('navigation',navigation(1100000),1100);m.consume('fusion',fusion(1100000,{state:{x_m:.5,y_m:.5,yaw_rad:.1}}),1100);
  m.consume('local_grid',grid(2,1100000,{transform_revision:2,grid_from_odom:b}),1100);
  const s=m.snapshot(1100);assert.equal(s.generation,1);assert.equal(s.transform_revision,2);
  assert.ok(Math.abs(s.pose.x-1)<1e-9 && Math.abs(s.pose.y-2)<1e-9);
  assert.equal(s.pose_odom.yaw,.1);assert.ok(Math.abs(s.pose.yaw-1.671)<1e-9);
  assert.deepEqual(s.planned_route.points[0],gridFromOdom({x:1,y:2},b));
  m.consume('navigation',navigation(1200000,{boot_id:3}),1200);
  assert.equal(m.snapshot(1200).available,false,'a new board domain cannot retain the old grid');
});

test('observed task goal, fused trace and sampled FPGA segments remain comparable after stopping',()=>{
  const m=new FpgaGridMap(), now=1000000;
  const active={active_task:true,goal_x_q16:32768,goal_y_q16:6554};
  m.consume('navigation',navigation(now,active),1000);m.consume('fusion',fusion(now),1000);
  m.consume('local_grid',grid(1,now),1000);
  m.consume('navigation',navigation(now+10000,active),1010);
  const plan={type:'local_plan',protocol:'nav_local_segment_v1',boot_id:1,odom_domain:2,
    task_id:7,plan_stamp_us:now+10000,valid:true,guard_stop:false,
    grid_direction_preview:true,source:'fpga',points:[{x:0,y:0},{x:.2,y:.1}]};
  m.consume('local_plan',plan,1010);
  m.consume('navigation',navigation(now+20000,active),1020);
  m.consume('fusion',fusion(now+20000,{state:{x_m:.025,y_m:0,yaw_rad:0}}),1020);
  m.consume('local_grid',grid(2,now+20000),1020);
  let snapshot=m.snapshot(1020);
  assert.equal(snapshot.active_goal.task_id,7);
  assert.equal(snapshot.active_goal.x,.5);
  assert.equal(snapshot.active_goal.y,6554/65536);
  assert.equal(snapshot.trajectory.at(-1).task_id,7);
  assert.deepEqual(snapshot.planned_history[0].points,plan.points);
  assert.equal(snapshot.planned_history[0].task_id,7);
  assert.equal(snapshot.planned_history[0].snapshot_id,1);
  m.consume('navigation',navigation(now+30000,{status:7,active_task:false}),1030);
  snapshot=m.snapshot(1030);
  assert.equal(snapshot.active_goal,null);
  assert.equal(snapshot.planned_route,null);
  assert.equal(snapshot.planned_history.length,1,'historical observations are not presented as an active plan');
  m.consume('navigation',navigation(now+40000,{status:7,active_task:false}),1040);
  m.consume('fusion',fusion(now+40000),1040);
  m.consume('local_grid',grid(3,now+40000,{transform_revision:2}),1040);
  assert.equal(m.snapshot(1040).planned_history.length,1,'CSM rebase keeps map-frame history');
  m.consume('navigation',navigation(now+60000,{status:7,active_task:false}),1060);
  m.consume('fusion',fusion(now+60000),1060);
  m.consume('local_grid',grid(4,now+60000,{grid_frame_epoch:2}),1060);
  assert.equal(m.snapshot(1060).planned_history.length,0,'a new grid frame clears old geometry');
});

test('only a fresh nonzero device segment is sampled and repeated geometry is not duplicated',()=>{
  const m=new FpgaGridMap(), now=1000000;
  ready(m,1000,now);m.consume('local_grid',grid(1,now),1000);
  const plan={type:'local_plan',protocol:'nav_local_segment_v1',boot_id:1,odom_domain:2,
    task_id:7,plan_stamp_us:now,valid:true,guard_stop:false,grid_direction_preview:true,
    source:'fpga',points:[{x:0,y:0},{x:0,y:0}]};
  m.consume('local_plan',plan,1000);
  m.consume('navigation',navigation(now+10000),1010);m.consume('local_grid',grid(2,now+10000),1010);
  assert.equal(m.snapshot(1010).planned_history.length,0,'coincident endpoints do not invent a line');
  m.consume('local_plan',{...plan,plan_stamp_us:now+10000,points:[{x:0,y:0},{x:.2,y:0}]},1010);
  m.consume('navigation',navigation(now+20000),1020);m.consume('local_grid',grid(3,now+20000),1020);
  m.consume('navigation',navigation(now+30000),1030);m.consume('local_grid',grid(4,now+30000),1030);
  assert.equal(m.snapshot(1030).planned_history.length,1);
  m.consume('navigation',navigation(now+600000),1600);m.consume('fusion',fusion(now+600000),1600);
  m.consume('local_grid',grid(5,now+600000),1600);
  assert.equal(m.snapshot(1600).planned_history.length,1,'an expired plan is not sampled again');
});

test('successive waypoint tasks keep their own sampled segments and trajectory labels',()=>{
  const m=new FpgaGridMap(), base=1000000;
  const task=(id,time)=>navigation(time,{task_id:id,active_task:true,goal_x_q16:32768,goal_y_q16:0});
  const plan=(id,time,start,end)=>({type:'local_plan',protocol:'nav_local_segment_v1',boot_id:1,odom_domain:2,
    task_id:id,plan_stamp_us:time,valid:true,guard_stop:false,grid_direction_preview:true,
    source:'fpga',points:[start,end]});
  m.consume('navigation',task(7,base),1000);m.consume('fusion',fusion(base),1000);
  m.consume('local_grid',grid(1,base),1000);
  m.consume('local_plan',plan(7,base,{x:0,y:0},{x:.2,y:0}),1000);
  m.consume('navigation',task(7,base+20000),1020);m.consume('fusion',fusion(base+20000),1020);
  m.consume('local_grid',grid(2,base+20000),1020);
  m.consume('navigation',navigation(base+30000,{status:4,active_task:false}),1030);
  m.consume('navigation',task(8,base+40000),1040);
  m.consume('local_plan',plan(8,base+40000,{x:.2,y:0},{x:.2,y:.2}),1040);
  m.consume('navigation',task(8,base+60000),1060);
  m.consume('fusion',fusion(base+60000,{state:{x_m:.2,y_m:0,yaw_rad:0}}),1060);
  m.consume('local_grid',grid(3,base+60000),1060);
  const snapshot=m.snapshot(1060);
  assert.deepEqual(snapshot.planned_history.map(segment=>segment.task_id),[7,8]);
  assert.deepEqual(snapshot.trajectory.map(point=>point.task_id),[7,8]);
  assert.equal(snapshot.active_goal.task_id,8);
});

test('FPGA timestamp and snapshot identifiers wrap without accepting old frames',()=>{
  const m=new FpgaGridMap(),before=0xfffffff0,after=0x20;
  ready(m,1000,before);m.consume('local_grid',grid(0xffffffff,before),1000);
  m.consume('navigation',navigation(after),1001);m.consume('fusion',fusion(after),1001);
  m.consume('local_grid',grid(0,after),1001);assert.equal(m.snapshot(1001).revision,0);
  m.consume('local_grid',grid(0xffffffff,before),1002);assert.equal(m.snapshot(1002).revision,0);
});

test('real-grid plan ages across timestamp wrap and clears on invalidation or task termination',()=>{
  const m=new FpgaGridMap(),time=10000;
  ready(m,1000,time);m.consume('local_grid',grid(1,time),1000);
  const plan={type:'local_plan',protocol:'nav_local_segment_v1',boot_id:1,odom_domain:2,
    task_id:7,plan_stamp_us:(time-20000)>>>0,valid:true,guard_stop:false,
    grid_direction_preview:true,source:'fpga',points:[{x:0,y:0},{x:.9,y:0}]};
  m.consume('local_plan',plan,1000);
  assert.equal(m.snapshot(1479).planned_route.age_ms,499);
  assert.equal(m.snapshot(1480).planned_route,null);
  const current={...plan,plan_stamp_us:time};
  m.consume('local_plan',current,1481);
  assert.ok(m.snapshot(1481).planned_route);
  m.consume('local_plan',{...current,valid:false},1482);
  assert.equal(m.snapshot(1482).planned_route,null);
  m.consume('local_plan',current,1483);
  m.consume('navigation',navigation(time,{status:7}),1484);
  assert.equal(m.snapshot(1484).planned_route,null);
});
