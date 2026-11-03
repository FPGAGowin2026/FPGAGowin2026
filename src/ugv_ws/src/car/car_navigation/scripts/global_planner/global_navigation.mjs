import {randomUUID} from 'node:crypto';
import localTaskValidation from './public/js/local_task_validation.js';
import {checkGlobalSegment, planGlobalRoute} from './global_route.mjs';

const REPLAN_MS = 1000;
const TRANSFORM_MAX_AGE_MS = 750;
const SEGMENT_LENGTH_M = 0.75;
/* 260×250 mm车体的181 mm外接半径加10 mm余量；实际运动另经局部扫掠。 */
const RADIUS_M = 0.191;
/* 常规半径无路时允许生成窄通道候选，实际执行仍由局部车体和制动扫掠决定。 */
const FALLBACK_RADIUS_M = 0.135;
const BLOCKED_DETOUR_MS = 2000;
const DETOUR_RETRY_MS = 5000;
const MAX_GOALS = 64;
const MAX_PUBLISHED_ROUTE_POINTS = 256;
const MAX_PUBLISHED_STATE_BYTES = 24 * 1024;
// FPGA 分片 ACK 只说明入队，等待同一事务的真实导航状态。
const START_CONFIRMATION_MS = 2000;
/* 地图每格50mm；检查期间超过半格的位姿变化需要重新扫掠。 */
const SEGMENT_POSE_RECHECK_M = 0.025;

const reasonText = {
  invalid_map:'全局地图无效', invalid_local_map:'局部地图不可用',
  invalid_start:'当前位置无效', invalid_goal:'目标无效',
  start_outside:'当前位置不在全局地图内', goal_outside:'目标不在全局地图内',
  start_blocked:'当前位置与地图障碍冲突', goal_unknown:'目标位于未知区域',
  goal_blocked:'目标所在栅格被障碍占据', no_path:'当前地图中没有可用路线',
  route_too_complex:'当前地图路线过于复杂', map_too_large:'目标与地图范围超过规划容量',
};

/* 忽略只改变版本号的重复快照，仅对局部占用或坐标变换的实际变化复核。 */
function sameLocalGeometry(a, b) {
  if (a === b) return true;
  if (a.origin?.x !== b.origin?.x || a.origin?.y !== b.origin?.y ||
      JSON.stringify(a.grid_from_odom) !== JSON.stringify(b.grid_from_odom) ||
      !Array.isArray(a.cells_rle) || !Array.isArray(b.cells_rle) ||
      a.cells_rle.length !== b.cells_rle.length) return false;
  return a.cells_rle.every((value, index) => value === b.cells_rle[index]);
}

function mapPose(map, local) {
  const pose = local.pose_odom, t = map.map_from_odom;
  if (!pose || ![pose.x,pose.y,pose.yaw].every(Number.isFinite)) return map.pose;
  const c = Math.cos(t.yaw), s = Math.sin(t.yaw);
  return {x:t.x+c*pose.x-s*pose.y, y:t.y+s*pose.x+c*pose.y,
    yaw:Math.atan2(Math.sin(t.yaw+pose.yaw), Math.cos(t.yaw+pose.yaw))};
}

/* 全局路线只持有一个局部事务；取消事务确认前不发下一段。 */
export class GlobalNavigation {
  constructor({getContext, fetchMap, sendLocal, onUpdate = () => {},
               plan = args => planGlobalRoute(args), checkSegment = args => checkGlobalSegment(args),
               now = () => performance.now()}) {
    this.getContext = getContext;
    this.fetchMap = fetchMap;
    this.sendLocal = sendLocal;
    this.onUpdate = onUpdate;
    this.plan = plan;
    this.checkSegment = checkSegment;
    this.now = now;
    this.state = null;
    this.fields = null;
    this.task = null;
    this.cancel = null;
    this.requests = new Map();
    this.epoch = 0;
    this.lastPlanAt = -Infinity;
    this.planCount = 0;
    this.planning = null;
    this.lastNavStatus = null;
    this.blockedSince = null;
    this.lastBlockedAt = null;
    this.nextDetourAt = -Infinity;
    this.detourPending = false;
    this.verifyGoal = false;
    this.ending = null;
  }

  snapshot() {
    return this.state ? structuredClone(this.state) : null;
  }

  publish(status, message) {
    if (status !== undefined) this.state.status = status;
    if (message !== undefined) this.state.message = message;
    this.onUpdate(this.snapshot());
  }

  async start(_owner, fields) {
    if (this.state?.active || this.task || this.cancel) throw new Error('已有全局或局部任务尚未结束');
    if (fields?.allow_disconnect !== undefined && fields.allow_disconnect !== true)
      throw new Error('当前全局任务由 Jetson 独立执行，不支持断开网页时自动取消');
    if (typeof fields?.domain !== 'string' || !fields.domain ||
        !['map','odom'].includes(fields.frame) ||
        (fields.frame === 'map' ? typeof fields.generation !== 'string' || !fields.generation : fields.generation !== null) ||
        !Array.isArray(fields.goals) || fields.goals.length === 0 || fields.goals.length > MAX_GOALS ||
        fields.goals.some(p => !Number.isFinite(p?.x) || !Number.isFinite(p?.y) ||
          Math.abs(p.x) > 32767.999 || Math.abs(p.y) > 32767.999))
      throw new Error('导航坐标系、坐标域、地图版本或航点无效');
    const params = {goal_x_m:0, goal_y_m:0, hold_yaw_rad:0, final_yaw_rad:0,
      max_speed_mm_s:fields.max_speed_mm_s, max_accel_mm_s2:fields.max_accel_mm_s2,
      tolerance_mm:fields.tolerance_mm, yaw_tolerance_mrad:fields.yaw_tolerance_mrad,
      duration_s:fields.duration_s, mode:'independent', allow_disconnect:true, position_only:true};
    const invalid = localTaskValidation.validate(params);
    if (invalid) throw new Error(invalid.message);
    const context = this.getContext();
    if (!context.online || !context.ready || context.emergency || !context.nav ||
        context.ownerReady === false || context.nav.active_task === true ||
        `${context.nav.boot_id}:${context.nav.odom_domain}` !== fields.domain)
      throw new Error('导航状态、页面连接或坐标域不允许开始任务');

    this.fields = params;
    this.epoch++;
    this.lastPlanAt = -Infinity;
    this.planCount = 0;
    this.lastNavStatus = null;
    this.blockedSince = this.lastBlockedAt = null;
    this.nextDetourAt = -Infinity;
    this.detourPending = false;
    this.verifyGoal = false;
    this.ending = null;
    this.state = {id:randomUUID(), status:'planning', active:true, goal_index:0,
      goal_count:fields.goals.length, goals:fields.goals.map(p => ({x:p.x,y:p.y})),
      route:[], message:fields.frame === 'map' ? '正在规划全局路线' : '准备执行航点',
      frame:fields.frame, domain:fields.domain,
      generation:fields.generation, replans:0};
    this.publish();
    // 接收目标与路线搜索/设备执行分开确认，服务应能立即返回。
    return this.snapshot();
  }

  stop(reason = '用户取消导航任务') {
    if (!this.state?.active) return this.snapshot();
    // A failed or unknown transaction is not an outstanding cancellation.
    // A later explicit stop may issue a new transaction; ticks never retry it.
    if (this.cancel?.failed) this.cancel = null;
    if (!this.ending) this.beginEnding('cancelled', reason);
    else if (this.task && !this.cancel) this.requestCancel();
    return this.snapshot();
  }

  resetConnection(reason = '连接会话已改变') {
    const unfinished = this.state?.active || this.task || this.cancel;
    this.epoch++;
    this.task = this.cancel = this.ending = null;
    this.detourPending = false;
    this.requests.clear();
    if (this.state && unfinished) {
      this.state.active = false;
      this.state.route = [];
      this.publish('failed', `${reason}；旧任务执行结果未知`);
    }
    return this.snapshot();
  }

  finish(status, message) {
    this.epoch++;
    this.state.active = false;
    this.state.route = [];
    this.ending = null;
    this.detourPending = false;
    this.publish(status, message);
  }

  beginEnding(status, message) {
    this.epoch++;
    this.ending = {status, message};
    this.detourPending = false;
    this.state.route = [];
    if (!this.task) {
      this.finish(status, message);
      return;
    }
    this.publish('blocked', `${message}；正在停止局部段`);
    if (!this.cancel) this.requestCancel();
  }

  liveContext() {
    const context = this.getContext(), nav = context.nav;
    if (nav && `${nav.boot_id}:${nav.odom_domain}` !== this.state.domain) {
      this.resetConnection('FPGA坐标域已变化');
      return null;
    }
    if (!context.online || context.emergency || context.ownerReady === false || !nav) {
      this.stop('连接、急停或坐标域变化，导航任务已取消');
      return null;
    }
    return context;
  }

  requireLocalMap(context) {
    const local = context.localMap;
    if (local?.status === 'building' && local.available && local.domain === this.state.domain) return true;
    this.block('局部地图不可用，等待新鲜观测');
    return false;
  }

  block(message) {
    this.state.route = [];
    if (this.state.status !== 'blocked' || this.state.message !== message) this.publish('blocked', message);
    if (this.task) this.requestCancel();
  }

  requestCancel() {
    if (!this.task || this.cancel) return;
    this.epoch++;
    const request = this.sendLocal('local_cancel', {});
    if (!request) {
      if (!this.ending) this.ending = {status:'failed',message:'局部任务取消请求未发送'};
      this.publish('blocked', '局部任务取消请求未发送，停车状态未确认');
      return;
    }
    this.cancel = {requestId:request.requestId, taskId:request.taskId, ack:false};
    this.requests.set(request.requestId, {action:'local_cancel', taskId:request.taskId});
    this.publish(undefined, '已请求停止当前局部段，等待导航确认');
  }

  acceptResult(result) {
    const request = this.requests.get(result?.request_id);
    if (!request || result.action !== request.action) return false;
    this.requests.delete(result.request_id);
    if (request.action === 'local_start') {
      if (!this.task || this.task.requestId !== result.request_id) return true;
      if (!result.success || result.unknown === true ||
          (result.task_id !== undefined && result.task_id !== this.task.taskId)) {
        if (this.state.active && !this.ending) {
          if (!result.unknown && result.success === false) {
            this.task = null;
            this.finish('failed', '局部段被拒绝');
          } else {
            this.beginEnding('failed', result.unknown ? '局部段结果未知' : '局部段事务号不一致');
          }
        } else if (this.ending && !result.unknown && result.success === false) {
          this.task.rejected = true;
          if (!this.cancel) {
            this.task = null;
            const ending = this.ending;
            this.finish(ending.status, ending.message);
          }
        }
      } else {
        this.task.ack = true;
        this.task.ackAt = this.now();
      }
    } else {
      if (!this.cancel || this.cancel.requestId !== result.request_id) return true;
      if (!result.success || result.unknown === true ||
          (result.transaction_task_id !== undefined && result.transaction_task_id !== this.cancel.taskId)) {
        this.cancel.failed = true;
        this.epoch++;
        if (!this.ending) this.ending = {status:'failed',message:'停止失败'};
        if (!result.unknown && result.success === false && this.task?.rejected) {
          this.task = this.cancel = null;
          const ending = this.ending;
          this.finish(ending.status, ending.message);
        } else this.publish('blocked', result.unknown ? '停止结果未知，停车状态未确认' : '停止被拒绝或事务号不一致');
      } else this.cancel.ack = true;
    }
    return true;
  }

  async tick() {
    if (!this.state) return null;
    let context = this.getContext();
    const nav = context.nav;
    if (this.state.active && nav && `${nav.boot_id}:${nav.odom_domain}` !== this.state.domain) {
      this.resetConnection('FPGA坐标域已变化');
      return this.snapshot();
    }

    /* 停止结果和inactive遥测都必须对应取消事务；单靠ACK不算停稳。 */
    if (this.cancel && (this.cancel.ack || this.ending) && nav?.active_task === false &&
        nav.last_task_id === this.cancel.taskId) {
      this.task = this.cancel = null;
      this.blockedSince = this.lastBlockedAt = null;
      if (this.ending) {
        const ending = this.ending;
        this.finish(ending.status, `${ending.message}；导航已确认停止`);
        return this.snapshot();
      } else if (this.state.active) {
        this.lastPlanAt = -Infinity;
        this.publish('planning', '当前局部段已停止，重新规划');
      }
    }
    if (this.ending) {
      if (!this.cancel && this.task && nav?.active_task === false &&
          ((nav.task_id === this.task.taskId && [4,6,7,8].includes(nav.status)) ||
           `${nav.boot_id}:${nav.odom_domain}` !== this.state.domain)) {
        this.task = null;
        const ending = this.ending;
        this.finish(ending.status, `${ending.message}；导航已确认停止`);
      }
      return this.snapshot();
    }
    if (!this.state.active) return this.snapshot();
    context = this.liveContext();
    if (!context) return this.snapshot();
    if (this.cancel) return this.snapshot();

    if (this.task) {
      const rejectedId = Number.isInteger(nav.rejected_task_id) ? nav.rejected_task_id : nav.task_id;
      if ((nav.rx_rejection || nav.rejected) && rejectedId === this.task.taskId) {
        if (nav.active_task === false) {
          this.task = null;
          this.finish('failed', '局部导航拒绝了当前局部段');
        } else this.beginEnding('failed', '局部导航拒绝了当前局部段');
        return this.snapshot();
      }
      if (nav.active_task === true && nav.task_id !== this.task.taskId) {
        this.task = null;
        this.finish('failed', '局部导航正在执行其他任务，已终止导航调度');
        return this.snapshot();
      }
      if (nav.active_task === false && nav.task_id === this.task.taskId &&
          [6,7,8].includes(nav.status)) {
        this.task = null;
        this.finish('failed', '当前局部段已拒绝、取消或过期');
        return this.snapshot();
      }
      if (nav.active_task === false && nav.task_id === this.task.taskId &&
          nav.last_task_id === this.task.taskId && nav.status === 4) this.task.arrived = true;
      if (nav.task_id === this.task.taskId && nav.active_task === true &&
          [1,2,3,5,9].includes(nav.status)) {
        const firstObservation = !this.task.observed;
        this.task.observed = true;
        if (firstObservation && nav.status !== 5) this.publish('following', '设备已确认并执行局部段');
      }
      // 很短的段可能在分片事务回执前就到达；等待网桥释放该事务，
      // 再派下一段，避免真实终态早于 ACK 时新请求被并发事务拒绝。
      if (this.task.arrived && !this.task.ack) {
        this.publish('planning', '设备已报告局部段到达，等待发送事务结束');
        return this.snapshot();
      }
      if (this.task.arrived) {
        const final = this.task.final;
        this.requests.delete(this.task.requestId);
        this.epoch++;
        this.task = null;
        this.blockedSince = this.lastBlockedAt = null;
        this.lastPlanAt = -Infinity;
        this.verifyGoal = final;
        this.publish('planning', final ? '局部段已到达，复核航点位置' : '局部段已到达，规划下一段');
      } else if (this.task.ack && !this.task.observed &&
                 this.now() - this.task.ackAt >= START_CONFIRMATION_MS) {
        this.beginEnding('failed', '局部段已入队但设备未确认，执行结果未知');
        return this.snapshot();
      } else if (nav.task_id === this.task.taskId && nav.active_task === true) {
        if (Number.isFinite(nav.progress)) {
          if (Number.isFinite(this.task.bestProgress) && nav.progress > this.task.bestProgress)
            this.blockedSince = this.lastBlockedAt = null;
          this.task.bestProgress = Math.max(this.task.bestProgress ?? nav.progress, nav.progress);
        }
        if (nav.status === 5) {
          if (this.blockedSince === null) this.blockedSince = this.now();
          this.lastBlockedAt = this.now();
          if (this.lastNavStatus !== 5) this.lastPlanAt = -Infinity;
        } else if (this.lastBlockedAt !== null && this.now() - this.lastBlockedAt > REPLAN_MS) {
          this.blockedSince = this.lastBlockedAt = null;
        }
        if (nav.status === 5 && this.state.status !== 'blocked')
          this.publish('blocked', '局部段受阻，正在检查新地图');
        else if (nav.status !== 5 && this.state.status === 'blocked')
          this.publish('following', '正在执行局部段');
        this.lastNavStatus = nav.status;
      }
    }

    context = this.liveContext();
    if (!context || !this.requireLocalMap(context)) return this.snapshot();
    if (this.state.frame === 'odom') {
      this.runOdom(context);
      return this.snapshot();
    }
    if (this.planning || this.now() - this.lastPlanAt < REPLAN_MS) return this.snapshot();
    this.lastPlanAt = this.now();
    const epoch = this.epoch;
    const pending = this.runPlan(epoch);
    this.planning = pending;
    try { await pending; }
    finally { if (this.planning === pending) this.planning = null; }
    return this.snapshot();
  }

  runOdom(context) {
    const local = context.localMap, pose = local.pose_odom;
    if (!local.pose_fresh || !pose || ![pose.x,pose.y,pose.yaw].every(Number.isFinite)) {
      this.block('当前位置不可用，等待新鲜观测');
      return;
    }
    if (this.verifyGoal) {
      const goal = this.state.goals[this.state.goal_index];
      this.verifyGoal = false;
      if (Math.hypot(pose.x-goal.x, pose.y-goal.y) <= this.fields.tolerance_mm/1000) {
        this.state.goal_index++;
        if (this.state.goal_index === this.state.goal_count) {
          this.finish('arrived', '全部航点已到达');
          return;
        }
        this.publish('planning', '航点已到达，准备下一个航点');
      } else {
        this.publish('planning', '设备已结束局部段，但当前位置未进入航点容差，继续规划');
      }
    }
    const goal = this.state.goals[this.state.goal_index];
    this.state.route = [{x:pose.x,y:pose.y},{...goal}];
    if (this.task) return;
    if (!context.ready || context.nav.active_task === true) {
      this.publish('planning', '等待局部导航空闲及控制握手');
      return;
    }
    const yaw = Math.atan2(Math.sin(pose.yaw),Math.cos(pose.yaw));
    const request = this.sendLocal('local_start', {...this.fields,
      goal_x_m:goal.x, goal_y_m:goal.y, hold_yaw_rad:yaw, final_yaw_rad:yaw});
    if (!request) {
      this.finish('failed', '局部航点请求未发送');
      return;
    }
    this.task = {requestId:request.requestId, taskId:request.taskId, ack:false,
      endOdom:{...goal}, final:true};
    this.requests.set(request.requestId, {action:'local_start', taskId:request.taskId});
    this.lastNavStatus = null;
    this.publish('planning', '已提交局部航点，等待设备确认');
  }

  async readMap(epoch) {
    const started = this.now();
    let map;
    try { map = await this.fetchMap(); }
    catch (error) {
      if (epoch === this.epoch && this.state.active) {
        if ((typeof error.domain === 'string' && error.domain !== this.state.domain) ||
            (typeof error.generation === 'string' && error.generation !== this.state.generation))
          this.stop('全局地图或坐标域已变化，请重新选择目标');
        else this.block('全局地图暂不可用，等待恢复');
      }
      return null;
    }
    if (epoch !== this.epoch || !this.state.active) return null;
    if (map.domain !== this.state.domain || map.generation !== this.state.generation) {
      this.stop('全局地图或坐标域已变化，请重新选择目标');
      return null;
    }
    if (map.status !== 'building' || !Number.isFinite(map.map_from_odom_age_ms) ||
        map.map_from_odom_age_ms < 0 ||
        map.map_from_odom_age_ms + Math.max(0, this.now() - started) > TRANSFORM_MAX_AGE_MS) {
      this.block('全局地图或坐标变换已过期，等待恢复');
      return null;
    }
    return {map, started};
  }

  async verifiedSegment(epoch, used, context, next, afterNext, radius) {
    const endpoints = local => {
      const start = mapPose(used.map, local);
      let end;
      if (this.task) {
        const t = used.map.map_from_odom, c = Math.cos(t.yaw), s = Math.sin(t.yaw);
        end = {x:t.x+c*this.task.endOdom.x-s*this.task.endOdom.y,
               y:t.y+s*this.task.endOdom.x+c*this.task.endOdom.y};
      } else {
        const goal = this.state.goals[this.state.goal_index];
        let target = next;
        if (Math.hypot(target.x-start.x,target.y-start.y) <= .001 &&
            (target.x !== goal.x || target.y !== goal.y)) {
          if (!afterNext) return null;
          target = afterNext;
        }
        const dx = target.x-start.x, dy = target.y-start.y, distance = Math.hypot(dx,dy);
        const fraction = distance > SEGMENT_LENGTH_M ? SEGMENT_LENGTH_M / distance : 1;
        end = fraction === 1 ? {x:target.x,y:target.y} : {x:start.x+dx*fraction,y:start.y+dy*fraction};
      }
      return {start,end};
    };
    for (let attempt = 0; attempt < 2; attempt++) {
      const local = context.localMap, segment = endpoints(local);
      if (!segment) return {clear:false};
      const {start,end} = segment;
      const checked = await this.checkSegment({map:used.map, localMap:local,
        segment:[start,end], radius});
      if (epoch !== this.epoch || !this.state.active) return null;
      if (!checked.segmentClear) return {clear:false};
      context = this.liveContext();
      if (!context || !this.requireLocalMap(context)) return null;
      const latestPose = mapPose(used.map, context.localMap);
      const moved = Math.hypot(latestPose.x-start.x, latestPose.y-start.y);
      if (sameLocalGeometry(local, context.localMap) && moved <= 1e-9)
        return {clear:true,start,end,context};
      if (moved > SEGMENT_POSE_RECHECK_M) break;
      /* 位姿变化时从新起点重新检查，避免沿旧线段派发。 */
    }
    /* 最后一次在当前快照上同步复核短线段，避免异步检查期间无关格变化造成饥饿。
       限制工作量，极高分辨率时等待下一次复核，不能阻塞心跳。 */
    const segment = endpoints(context.localMap);
    if (!segment) return {clear:false};
    const {start,end} = segment, resolution = used.map.resolution_m;
    const width = (Math.abs(end.x-start.x)+2*radius)/resolution+5;
    const height = (Math.abs(end.y-start.y)+2*radius)/resolution+5;
    const range = 2*Math.ceil(radius/resolution)+3;
    const work = width*height+(Math.hypot(end.x-start.x,end.y-start.y)/resolution+2)*range*range;
    if (work > 1000000) {
      this.lastPlanAt = this.now() - REPLAN_MS + 250;
      this.publish('planning', '地图分辨率过高，等待路线复核');
      return null;
    }
    const checked = checkGlobalSegment({map:used.map,localMap:context.localMap,
      segment:[start,end],radius});
    return {clear:checked.segmentClear,start,end,context};
  }

  async planWithFallback(epoch, args) {
    let radius = RADIUS_M;
    let result = await this.plan({...args, radius});
    if (epoch !== this.epoch || !this.state.active) return null;
    if (!result.path.length && ['no_path', 'start_blocked', 'goal_blocked'].includes(result.reason)) {
      radius = FALLBACK_RADIUS_M;
      result = await this.plan({...args, radius});
      if (epoch !== this.epoch || !this.state.active) return null;
    }
    return {result, radius};
  }

  async runPlan(epoch) {
    try {
      const first = await this.readMap(epoch);
      if (!first) return;
      let context = this.liveContext();
      if (!context || !this.requireLocalMap(context)) return;
      let used = first;
      if (this.verifyGoal) {
        const goal = this.state.goals[this.state.goal_index];
        const current = mapPose(used.map, context.localMap);
        this.verifyGoal = false;
        if (Math.hypot(current.x-goal.x, current.y-goal.y) <=
            this.fields.tolerance_mm / 1000) {
          this.state.goal_index++;
          if (this.state.goal_index === this.state.goal_count) {
            this.state.active = false;
            this.state.route = [];
            this.publish('arrived', '全部全局航点已到达');
            return;
          }
          this.publish('planning', '航点已到达，规划下一个航点');
        }
      }
      const initialLocal = context.localMap;
      const transform = used.map.map_from_odom;
      const c = Math.cos(transform.yaw), s = Math.sin(transform.yaw);
      const current = mapPose(used.map, initialLocal);
      const currentEnd = this.task ? {
        x:transform.x+c*this.task.endOdom.x-s*this.task.endOdom.y,
        y:transform.y+s*this.task.endOdom.x+c*this.task.endOdom.y} : null;
      if (this.task?.endMap && currentEnd) {
        // 地图目标固定在用户选点坐标中。CSM/SLAM修正使旧odom段偏离时，
        // 先取消并等设备确认，再按新变换重新下发；odom任务不做此转换。
        const tolerance = Math.max(0.01, Math.min(0.05, this.fields.tolerance_mm / 2000));
        if (Math.hypot(currentEnd.x-this.task.endMap.x, currentEnd.y-this.task.endMap.y) > tolerance) {
          this.state.route = [];
          this.publish('blocked', '地图坐标修正，等待当前局部段停止后重新规划');
          this.requestCancel();
          return;
        }
      }
      const args = {map:used.map, localMap:context.localMap,
        start:current, goal:this.state.goals[this.state.goal_index],
        currentSegment:this.task ? [current, currentEnd] : null};
      const planned = await this.planWithFallback(epoch, args);
      if (!planned) return;
      const {result, radius} = planned;
      if (this.planCount++ > 0) this.state.replans++;
      if (result.path.length > MAX_PUBLISHED_ROUTE_POINTS) {
        this.block('全局路线过于复杂，无法完整展示；等待重新规划');
        return;
      }

      /* 规划后重新获取变换，并在最新局部栅格上独立检查实际待执行线段。 */
      const latest = await this.readMap(epoch);
      if (!latest) return;
      context = this.liveContext();
      if (!context || !this.requireLocalMap(context)) return;
      const plannedMap = used.map, plannedLocal = initialLocal;
      used = latest;
      if (!this.task && !result.path.length) {
        this.block(reasonText[result.reason] || '当前没有安全路线');
        return;
      }
      const detourSegment = this.task?.detour || (!this.task && this.detourPending);
      const segment = await this.verifiedSegment(epoch, used, context,
        result.path[1] || result.path[0], result.path[2], this.task?.radius ?? radius);
      if (!segment) return;
      if (!segment.clear) {
        if (this.task?.detour) this.detourPending = true;
        this.block(this.task ? '当前局部段出现障碍，先停止再重规划' : '当前路线段不可通行，等待重规划');
        return;
      }
      context = segment.context;
      if (used.map.map_from_odom_age_ms + Math.max(0, this.now() - used.started) > TRANSFORM_MAX_AGE_MS) {
        this.block('规划耗时过长，等待新的坐标变换');
        return;
      }
      const actualTransform = used.map.map_from_odom, plannedTransform = plannedMap.map_from_odom;
      const routeCurrent = used.map.revision === plannedMap.revision &&
        actualTransform.x === plannedTransform.x && actualTransform.y === plannedTransform.y &&
        actualTransform.yaw === plannedTransform.yaw && sameLocalGeometry(plannedLocal, context.localMap);
      this.state.route = result.path.length ?
        (routeCurrent ? [segment.start,...result.path.slice(1)] : [segment.start,segment.end]) : [];
      if (Buffer.byteLength(JSON.stringify(this.state), 'utf8') > MAX_PUBLISHED_STATE_BYTES) {
        this.block('全局路线状态超过传输容量，无法完整显示；等待重新规划');
        return;
      }
      if (this.task) {
        if (this.lastNavStatus === 5 && this.blockedSince !== null &&
            this.now() - this.blockedSince >= BLOCKED_DETOUR_MS && this.now() >= this.nextDetourAt) {
          this.nextDetourAt = this.now() + DETOUR_RETRY_MS;
          const taskId = this.task.taskId;
          const plannedDetour = await this.planWithFallback(epoch, {map:used.map,localMap:context.localMap,
            start:segment.start,goal:this.state.goals[this.state.goal_index],
            currentSegment:[segment.start,segment.end]});
          if (!plannedDetour) return;
          const detour = plannedDetour.result;
          if (epoch !== this.epoch || !this.state.active || !this.task || this.task.taskId !== taskId) return;
          const latestContext = this.liveContext();
          if (!latestContext || latestContext.nav.task_id !== taskId ||
              latestContext.nav.active_task !== true || latestContext.nav.status !== 5) return;
          if (Number.isFinite(latestContext.nav.progress) && Number.isFinite(this.task.bestProgress) &&
              latestContext.nav.progress > this.task.bestProgress) {
            this.task.bestProgress = latestContext.nav.progress;
            this.blockedSince = this.lastBlockedAt = null;
            return;
          }
          if (detour.path.length > 1 &&
              used.map.map_from_odom_age_ms + Math.max(0,this.now()-used.started) <= TRANSFORM_MAX_AGE_MS) {
            const next = detour.path[1], dx=next.x-segment.start.x,dy=next.y-segment.start.y;
            const distance=Math.hypot(dx,dy),fraction=distance>SEGMENT_LENGTH_M ? SEGMENT_LENGTH_M/distance : 1;
            const oldX=segment.end.x-segment.start.x,oldY=segment.end.y-segment.start.y;
            const oldLength=Math.hypot(oldX,oldY);
            const sideways=oldLength ? Math.abs(dx*oldY-dy*oldX)*fraction/oldLength : 0;
            const reverse=oldLength && dx*oldX+dy*oldY<0 && distance*fraction>.05;
            if (sideways > .05 || reverse) {
              this.detourPending = true;
              this.requestCancel();
              if (!this.cancel) this.detourPending = false;
              return;
            }
          }
        }
        this.publish(this.lastNavStatus === 5 ? 'blocked' : this.task.observed ? 'following' : 'planning',
          this.lastNavStatus === 5 ? '局部段受阻，等待通路变化' :
            !this.task.observed ? '已提交局部段，等待设备确认' :
            !result.path.length ? `${reasonText[result.reason] || '后续路线受阻'}；当前安全段继续执行` : '正在执行局部段');
        return;
      }
      if (!context.ready || context.nav.active_task === true) {
        this.publish('planning', '等待局部导航空闲及控制握手');
        return;
      }
      const {start,end} = segment;
      const dx = end.x-start.x, dy = end.y-start.y, distance = Math.hypot(dx,dy);
      const goal = this.state.goals[this.state.goal_index];
      const final = end.x === goal.x && end.y === goal.y;
      const currentTransform = used.map.map_from_odom;
      const currentCosine = Math.cos(currentTransform.yaw), currentSine = Math.sin(currentTransform.yaw);
      const ox = end.x-currentTransform.x, oy = end.y-currentTransform.y;
      const endOdom = {x:currentCosine*ox+currentSine*oy, y:-currentSine*ox+currentCosine*oy};
      if (!Number.isFinite(endOdom.x) || !Number.isFinite(endOdom.y) ||
          Math.abs(endOdom.x) > 32767.999 || Math.abs(endOdom.y) > 32767.999) {
        this.epoch++;
        this.state.active = false;
        this.publish('failed', '局部段目标超出坐标表示范围');
        return;
      }
      const heading = distance ? Math.atan2(dy,dx) : used.map.pose.yaw;
      const odomYaw = Math.atan2(Math.sin(heading-currentTransform.yaw), Math.cos(heading-currentTransform.yaw));
      /* 折点短于用户容差时也要实际前移，避免一直重复同一段。 */
      const tolerance = final ? this.fields.tolerance_mm :
        Math.min(this.fields.tolerance_mm, 100, Math.max(1, Math.floor(distance*500)));
      const request = this.sendLocal('local_start', {...this.fields,
        goal_x_m:endOdom.x, goal_y_m:endOdom.y,
        hold_yaw_rad:odomYaw, final_yaw_rad:odomYaw, tolerance_mm:tolerance});
      if (!request) {
        this.epoch++;
        this.state.active = false;
        this.publish('failed', '局部段请求未发送');
        return;
      }
      this.task = {requestId:request.requestId, taskId:request.taskId, ack:false, endOdom,
        endMap:{x:end.x,y:end.y}, final, radius,
        detour:!!detourSegment};
      // 绕行可能有多个折点，直到车宽路线可直连用户目标才回到普通中心线规划。
      if (result.path.length <= 2) this.detourPending = false;
      this.requests.set(request.requestId, {action:'local_start', taskId:request.taskId});
      this.lastNavStatus = null;
      this.publish('planning', '已提交局部段，等待设备确认');
    } catch (error) {
      if (epoch === this.epoch && this.state.active) {
        this.block(`全局路线暂不可用：${error.message}`);
      }
    }
  }
}
