// 汇集设备或 Jetson 已完成的局部栅格快照，不在网页服务器重建占用格。
const u32 = value => Number.isInteger(value) && value >= 0 && value <= 0xffffffff;
const i32 = value => Number.isInteger(value) && value >= -0x80000000 && value <= 0x7fffffff;
const newer = (value, previous) => { const d = (value - previous) >>> 0; return d !== 0 && d < 0x80000000; };
const deltaUs = (later, earlier) => (later - earlier) | 0;
const normalize = value => Math.atan2(Math.sin(value), Math.cos(value));
const MAX_POSE_LEAD_US = 20000; // 同域 fusion 与 navigation 报文可相差一个控制拍。
const validPose = data => (data?.flags & 1) !== 0 && u32(data?.pose_stamp_us) &&
  ['x_m','y_m','yaw_rad'].every(key => Number.isFinite(data?.state?.[key]));

export function gridFromOdom(point, transform) {
  const angle = transform.yaw_offset_mrad / 1000, c = Math.cos(angle), s = Math.sin(angle);
  const dx = point.x - transform.odom_anchor_x_mm / 1000;
  const dy = point.y - transform.odom_anchor_y_mm / 1000;
  return {x:transform.map_anchor_x_mm / 1000 + c*dx - s*dy,
    y:transform.map_anchor_y_mm / 1000 + s*dx + c*dy,
    ...(Number.isFinite(point.yaw) ? {yaw:normalize(point.yaw + angle)} : {})};
}

function unpack(data) {
  if (!Array.isArray(data) || data.length !== 1024 ||
      !data.every(value => Number.isInteger(value) && value >= 0 && value <= 255)) return null;
  const cells = new Uint8Array(4096), counts = {unknown:0,free:0,occupied:0};
  for (let i=0;i<cells.length;i++) {
    const value=(data[i >> 2] >> (2*(i & 3))) & 3;
    if (value === 3) return null;
    cells[i]=value;
    counts[['unknown','free','occupied'][value]]++;
  }
  const cells_rle=[];
  for (let i=0;i<cells.length;) {
    let end=i+1; while (end<cells.length && cells[end]===cells[i]) end++;
    cells_rle.push(cells[i],end-i); i=end;
  }
  return {cells_rle,counts};
}

function validateGrid(data) {
  const transform=data?.grid_from_odom;
  const source = data?.protocol === 'nav_grid_snapshot_v1' ? 'fpga' :
    data?.protocol === 'jetson_grid_snapshot_v1' ? 'jetson' : null;
  if (data?.type !== 'local_grid' || !source || (data.source && data.source !== source) ||
      !['sequence','time_us','boot_id','odom_domain','snapshot_id','capture_begin_us','capture_end_us',
        'grid_generation','scan_generation','complete_scan_stamp_us','last_ray_stamp_us',
        'grid_frame_epoch','transform_revision','flags'].every(key => u32(data[key])) ||
      !data.boot_id || !data.odom_domain || data.sequence !== data.snapshot_id ||
      data.time_us !== data.capture_end_us || data.width !== 64 || data.height !== 64 ||
      data.resolution_mm !== 50 || (data.flags & ~31) !== 0 ||
      !i32(data.origin_x_cell) || !i32(data.origin_y_cell) ||
      !['map_anchor_x_mm','map_anchor_y_mm','odom_anchor_x_mm','odom_anchor_y_mm','yaw_offset_mrad']
        .every(key => i32(transform?.[key]))) return null;
  const duration=deltaUs(data.capture_end_us,data.capture_begin_us);
  if (duration < 0 || duration > 750000) return null;
  const decoded=unpack(data.packed_cells);
  return decoded ? {...data,source,grid_from_odom:{...transform},...decoded,packed_cells:undefined} : null;
}

export class FpgaGridMap {
  constructor() { this.reset('等待局部栅格'); }
  reset(reason = '等待局部栅格') {
    this.reason=reason; this.domain=null; this.nav=null; this.navAt=null;
    this.pose=null; this.poseAt=null; this.grid=null; this.gridAt=null;
    this.pending=null; this.lastSnapshot=null; this.plan=null; this.planAt=null;
    this.activeGoal=null; this.trajectory=[]; this.plannedHistory=[];
  }
  consume(stream,data,at) {
    if (!Number.isFinite(at)) return;
    if (stream === 'navigation') {
      if (!u32(data?.boot_id) || !data.boot_id || !u32(data?.odom_domain) || !data.odom_domain ||
          !u32(data?.now_us) || !['nav_grid_v1','jetson_nav_v1'].includes(data.protocol)) return;
      const domain=`${data.boot_id}:${data.odom_domain}`;
      if (this.domain !== domain) { this.reset('板卡启动或里程计域变化，等待新栅格'); this.domain=domain; }
      else if (this.nav && this.nav.protocol !== data.protocol) {
        this.reset('局部导航来源已变化，等待新栅格'); this.domain=domain;
      }
      if (this.nav && deltaUs(data.now_us,this.nav.now_us) < 0) return;
      this.nav={...data}; this.navAt=at;
      this.activeGoal=data.active_task === true && u32(data.task_id) &&
        i32(data.goal_x_q16) && i32(data.goal_y_q16) ?
        {task_id:data.task_id,x:data.goal_x_q16/65536,y:data.goal_y_q16/65536} : null;
      if (![1,2,3,5,9].includes(data.status) || this.plan?.task_id !== data.task_id) this.plan=null;
      this.commitPending();
      return;
    }
    if (!this.domain || !this.nav || this.navAt === null || at-this.navAt > 750) return;
    if (stream === 'fusion') {
      // 非法帧或超出一个控制拍的源时钟领先不得覆盖上一可信位姿。
      if (!validPose(data) || deltaUs(this.nav.now_us,data.pose_stamp_us) < -MAX_POSE_LEAD_US) return;
      this.pose={x:data.state.x_m,y:data.state.y_m,yaw:data.state.yaw_rad,stamp:data.pose_stamp_us};
      this.poseAt=at; return;
    }
    if (stream === 'local_plan') {
      if (!['nav_local_segment_v1','jetson_local_segment_v1'].includes(data?.protocol) ||
          data.boot_id !== this.nav.boot_id ||
          data.odom_domain !== this.nav.odom_domain || data.task_id !== this.nav.task_id ||
          ![1,2,3,5,9].includes(this.nav.status) || data.grid_direction_preview !== true ||
          typeof data.valid !== 'boolean' || typeof data.guard_stop !== 'boolean' ||
          !u32(data.plan_stamp_us) || !Array.isArray(data.points) || data.points.length !== 2 ||
          !data.points.every(point => Number.isFinite(point?.x) && Number.isFinite(point?.y))) return;
      this.plan=data.valid ? {...data,points:data.points.map(point=>({...point}))} : null;
      this.planAt=at;
      if (this.plan && this.grid) {
        const view=this.snapshot(at);
        if (view.status === 'building') this.rememberPlan(view.planned_route,this.grid);
      }
      return;
    }
    if (stream !== 'local_grid') return;
    const grid=validateGrid(data);
    if (!grid || `${grid.boot_id}:${grid.odom_domain}` !== this.domain ||
        (grid.source === 'jetson') !== (this.nav.protocol === 'jetson_nav_v1')) return;
    const latest=this.pending?.grid ?? this.grid;
    if (latest && (!newer(grid.snapshot_id,latest.snapshot_id) ||
        !newer(grid.capture_end_us,latest.capture_end_us))) return;
    if (this.grid && grid.grid_frame_epoch === this.grid.grid_frame_epoch) {
      const old=this.grid;
      if (grid.transform_revision !== old.transform_revision && !newer(grid.transform_revision,old.transform_revision)) return;
      if (grid.transform_revision === old.transform_revision &&
          Object.keys(old.grid_from_odom).some(key => grid.grid_from_odom[key] !== old.grid_from_odom[key])) return;
    }
    this.pending={grid,at};
    this.commitPending();
  }
  rememberPlan(route,grid) {
    if (!route || !['fpga','jetson'].includes(route.source) || !u32(route.task_id) ||
        Math.hypot(route.points[1].x-route.points[0].x,route.points[1].y-route.points[0].y) === 0) return;
    const last=this.plannedHistory.at(-1);
    if (!last || last.task_id !== route.task_id || last.guard_stop !== route.guard_stop ||
        route.points.some((point,i)=>Math.hypot(point.x-last.points[i].x,point.y-last.points[i].y) >= .02)) {
      this.plannedHistory.push({task_id:route.task_id,plan_stamp_us:route.plan_stamp_us,
        snapshot_id:grid.snapshot_id,transform_revision:grid.transform_revision,
        guard_stop:route.guard_stop,points:route.points});
      if (this.plannedHistory.length > 300) this.plannedHistory.shift();
    }
  }
  commitPending() {
    if (!this.pending || !this.nav) return;
    const {grid,at}=this.pending;
    const age=deltaUs(this.nav.now_us,grid.capture_end_us);
    if (age < 0) return; // 导航源时钟追上捕获结束前不显示该图。
    this.pending=null;
    if (age > 750000) return;
    if (this.grid && grid.grid_frame_epoch !== this.grid.grid_frame_epoch) {
      this.trajectory=[]; this.plannedHistory=[];
    }
    this.grid=grid; this.gridAt=at; this.lastSnapshot=grid.snapshot_id;
    if ((grid.flags & 1) && this.pose && this.poseAt !== null && at-this.poseAt <= 500 &&
        deltaUs(this.nav.now_us,this.pose.stamp) >= 0) {
      const point={...gridFromOdom(this.pose,grid.grid_from_odom),task_id:this.activeGoal?.task_id ?? null};
      const previous=this.trajectory.at(-1);
      if (!previous || point.task_id !== previous.task_id || Math.hypot(point.x-previous.x,point.y-previous.y)>.02 ||
          Math.abs(normalize(point.yaw-previous.yaw))>.035) this.trajectory.push(point);
      if (this.trajectory.length>2000) this.trajectory.shift();
    }
    const view=this.snapshot(Math.max(at,this.navAt));
    if (view.status === 'building') this.rememberPlan(view.planned_route,grid);
  }
  snapshot(now) {
    if (!this.grid || this.gridAt === null || !this.nav || this.navAt === null) return {
      v:1,type:'mapping',protocol:'fpga_grid_view_v1',source:'fpga',available:false,
      status:'paused',reason:this.reason,domain:this.domain};
    const g=this.grid, elapsedNav=Math.max(0,now-this.navAt), elapsedGrid=Math.max(0,now-this.gridAt);
    const sourceAge=deltaUs(this.nav.now_us,g.capture_begin_us)/1000 + elapsedNav;
    const snapshotAge=Math.max(elapsedGrid,sourceAge);
    const scanAge=g.scan_generation ? deltaUs(this.nav.now_us,g.complete_scan_stamp_us)/1000 + elapsedNav : Infinity;
    const rayAge=g.scan_generation ? deltaUs(this.nav.now_us,g.last_ray_stamp_us)/1000 + elapsedNav : Infinity;
    const poseSourceAge=this.pose ? deltaUs(this.nav.now_us,this.pose.stamp)/1000 + elapsedNav : Infinity;
    const poseAge=this.poseAt === null ? Infinity : Math.max(now-this.poseAt,poseSourceAge);
    const poseFresh=!!(this.pose && poseAge<=500 && elapsedNav<=750);
    const initialized=(g.flags&1)!==0, fresh=(g.flags&2)!==0 && scanAge>=0 && rayAge>=0 && scanAge<=1500 && rayAge<=1500;
    const ready=(g.flags&4)!==0, gap=(g.flags&8)!==0;
    const transportFresh=snapshotAge>=0 && snapshotAge<750 && elapsedGrid<750 && elapsedNav<=750;
    const available=initialized && transportFresh && fresh && ready && !gap && poseFresh;
    const pose=poseFresh && initialized ? gridFromOdom(this.pose,g.grid_from_odom) : null;
    const activeGoal=available && this.activeGoal ? {
      ...gridFromOdom(this.activeGoal,g.grid_from_odom),task_id:this.activeGoal.task_id} : null;
    const planAge=this.plan && this.planAt !== null ? Math.max(now-this.planAt,
      deltaUs(this.nav.now_us,this.plan.plan_stamp_us)/1000 + elapsedNav) : Infinity;
    const plan=this.plan && planAge>=0 && planAge<500 && initialized ? {
      ...this.plan,frame_id:g.source === 'jetson' ? 'jetson_grid' : 'fpga_grid',domain:this.domain,age_ms:planAge,
      points:this.plan.points.map(point=>gridFromOdom(point,g.grid_from_odom))} : null;
    const label=g.source === 'jetson' ? 'Jetson' : 'FPGA';
    const reason=!initialized ? `${label} 栅格或坐标变换无效` : !transportFresh ? `${label} 栅格快照已过期` :
      !fresh ? `${label} 地图观测已过期` : !ready ? `${label} 完整扫描未就绪` :
      gap ? `${label} 地图观测存在缺口` : !poseFresh ? '位姿无效或已过期' : `${label} 局部栅格`;
    return {v:1,type:'mapping',protocol:g.source === 'jetson' ? 'jetson_grid_view_v1' : 'fpga_grid_view_v1',
      source:g.source,available:true,
      status:available ? 'building' : 'paused',reason,domain:this.domain,navigation_protocol:this.nav.protocol,
      generation:g.grid_frame_epoch,revision:g.snapshot_id,grid_generation:g.grid_generation,scan_generation:g.scan_generation,
      transform_revision:g.transform_revision,grid_from_odom:g.grid_from_odom,
      width:64,height:64,resolution_m:.05,origin:{x:g.origin_x_cell*.05,y:g.origin_y_cell*.05},
      bounds:{min_x:0,min_y:0,max_x:63,max_y:63},cells_rle:g.cells_rle,counts:g.counts,
      snapshot_age_ms:snapshotAge,grid_age_ms:Math.max(scanAge,rayAge),
      grid_fresh:fresh,grid_ready:ready,grid_gap:gap,start_confirmation_lost:(g.flags&16)!==0,
      pose,pose_odom:poseFresh ? {x:this.pose.x,y:this.pose.y,yaw:this.pose.yaw} : null,
      pose_fresh:poseFresh,pose_age_ms:poseAge,trajectory:this.trajectory.slice(),active_goal:activeGoal,
      planned_route:plan,planned_history:this.plannedHistory.slice()};
  }
}
