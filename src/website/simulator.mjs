import crypto from 'node:crypto';
import dgram from 'node:dgram';
import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { EventEmitter } from 'node:events';
import { performance } from 'node:perf_hooks';

const HERE = path.dirname(fileURLToPath(import.meta.url));
const IMAGE_FILE = path.join(HERE, 'assets', 'simulator.jpg');
const U32 = 0x1_0000_0000;
const JPEG_CHUNK = 1200;
const HEADER_BYTES = 12;

function u32(value) { return Number.isInteger(value) && value >= 0 && value < U32; }
function newer(candidate, previous) {
  const delta = (candidate - previous + U32) % U32;
  return delta > 0 && delta < U32 / 2;
}
function monoSeconds(startAt) { return Number(process.hrtime.bigint() - startAt) / 1e9; }
function makeId() { return crypto.randomUUID(); }
function jsonBuffer(value) { return Buffer.from(JSON.stringify(value)); }

function imuData(sequence, timeUs) {
  const z = Math.round(Math.sin(sequence / 20) * 8);
  return {
    type: 'imu', sequence, time_us: timeUs, flags: 15, who_am_i: 0x70,
    sample_sequence: sequence & 0xffff,
    gyro_raw: { x: 0, y: 0, z }, accel_raw: { x: 0, y: 0, z: 16384 },
    gyro_rad_s_nominal: { x: 0, y: 0, z: z * 0.0175 * Math.PI / 180 },
    samples: sequence + 1, identity_errors: 0, queue_dropped: 0,
  };
}

function motorData(sequence, timeUs, command) {
  const [vx, vy, wz] = command.velocity;
  const rotation = Math.trunc((wz * 178) / 1000);
  const speeds = {
    A: vx - vy - rotation,
    B: vx + vy + rotation,
    C: vx + vy - rotation,
    D: vx - vy + rotation,
  };
  const wheels = ['A', 'B', 'C', 'D'];
  const ticks = Object.fromEntries(wheels.map((wheel, index) => [wheel, (sequence * (index + 1)) | 0]));
  const physicalTicks = {};
  for (let logical = 0; logical < wheels.length; logical += 1) {
    const physical = (0xd2 >> (logical * 2)) & 0x03;
    physicalTicks[`MOTOR${physical + 1}`] = ticks[wheels[logical]];
  }
  return {
    type: 'motor', sequence, time_us: timeUs,
    motor_channel_map: 0xd2, encoder_channel_map: 0xd2, motor_invert: 0x0b, encoder_invert: 0x0a,
    command: command.velocity.some(Boolean) ? 'F' : 'S', pwm_percent: command.velocity.some(Boolean) ? 18 : 0,
    active_physical_mask: command.velocity.some(Boolean) ? 15 : 0, encoder_invalid_mask: 0,
    count_ticks: ticks,
    physical_count_ticks: physicalTicks,
    wheel_mm_s: speeds, target_mm_s: speeds,
    odometry: { vx_mm_s: vx, vy_mm_s: vy, wz_mrad_s: wz }, queue_dropped: 0,
  };
}

function lidarData(sequence, timeUs) {
  const points = Array.from({ length: 16 }, (_, index) => ({
    angle_cdeg: index * 2200,
    distance_mm: 1200 + Math.round(200 * Math.sin((sequence + index) / 8)),
    strength: 120 + (index % 8),
  }));
  return { type: 'lidar', sequence, time_us: timeUs, packets: sequence + 1, crc_errors: 0, length_errors: 0, queue_dropped: 0, points };
}

function fusionData(sequence, timeUs, command, grid = false) {
  const yaw = (sequence % 628) / 100;
  const state = [1.25, -0.5, yaw, 0, 0, 0, 0];
  const diag = [0.01, 0.01, 0.02, 0.03, 0.03, 0.04, 0.001];
  if (grid) return {
    type:'fusion', sequence, time_us:timeUs, flags:0xd000c001, architecture:'local_grid_v1',
    state:{x_m:state[0],y_m:state[1],yaw_rad:yaw,vx_m_s:0,vy_m_s:0,wz_rad_s:0,gyro_bias_rad_s:0},
    error_estimate:[30,40,50], lidar_raw_samples:sequence*16, lidar_reduced_samples:sequence,
    scan_match_accepted:sequence, scan_match_rejected:0, map_update_mode:'local_grid',
    map_fresh:true, map_ready:true, map_gap_latched:false, candidate_planning:false,
    progress_blocked:false, unknown_path:false, map_recovery_required:false, grid_v1_diagnostic:true,
    updates:sequence+1, pose_stamp_us:timeUs, imu_age_us:10000, radar_age_us:25000, lidar_drops:0,
  };
  return {
    type: 'fusion', sequence, time_us: timeUs,
    state_q16: state.map(value => Math.round(value * 65536)),
    covariance_q24: diag.map(value => Math.round(value * 16777216)),
    covariance_diag: diag,
    state: { x_m: 1.25, y_m: -0.5, yaw_rad: yaw, vx_m_s: 0, vy_m_s: 0, wz_rad_s: 0, gyro_bias_rad_s: 0 },
    flags: 3 | (command?.auto ? (1 << 11) : 0), updates: sequence + 1, ekf_rejected: 0, saturated: 0,
    result_time_us: timeUs, input_dropped: 0, lidar_rejected: 0, scans: sequence + 1,
    gyro_age_us: 10000, lidar_age_us: 25000, lidar_score: 90, queue_dropped: 0,
  };
}

function healthData(sequence, timeUs) {
  const names = ['video_flags', 'camera_frames', 'ethernet_packets', 'video_packets', 'video_errors', 'video_dropped_frames', 'video_skipped_frames'];
  const health = { type: 'health', sequence, time_us: timeUs };
  names.forEach((name, index) => { health[name] = index === 0 ? 31 : sequence * (index + 1); });
  for (let index = 0; index < 10; index += 1) health[`ddr_gray_${index}`] = (sequence + index * 127) >>> 0;
  Object.assign(health, {
    radar_uart_bytes: sequence * 24, radar_framing_errors: 0, radar_packets: sequence + 1,
    radar_crc_errors: 0, radar_length_errors: 0, radar_points: 16,
    imu_flags: 15, imu_who_am_i: 0x70, imu_samples: sequence + 1, imu_identity_errors: 0, queue_dropped: 0,
    video_ready: { pll_locked: true, camera_initialized: true, ddr_calibrated: true, camera_frame_seen: true, capture_enabled: true },
    ddr_counters: Array.from({ length: 10 }, (_, index) => (sequence + index * 131) >>> 0),
  });
  return health;
}

function navigationData(sequence, timeUs, task, grid = false) {
  const active = (task.status >= 1 && task.status <= 3) || task.status === 5;
  if (grid) return {
    type:'navigation', sequence, time_us:timeUs >>> 0, protocol:'nav_grid_v1',
    boot_id:task.boot_id, odom_domain:task.odom_domain, task_id:task.task_id, last_task_id:task.last_task_id,
    map_version:1, now_us:timeUs >>> 0, expiry_us:task.expiry_us >>> 0,
    deadline_known:task.expiry_us !== 0, status:task.status, error:task.error,
    mode:task.mode, flags:0xa6, vx_mm_s:0, vy_mm_s:0, wz_mrad_s:0,
    rejected:false, rx_rejection:false, active_task:active,
    cancelled_task_id:task.cancelled_task_id ?? 0, cancel_transaction_id:task.cancel_transaction_id ?? 0,
    goal_x_q16:task.goal_x_q16, goal_y_q16:task.goal_y_q16, candidates_evaluated:active ? 7 : 0,
    map_age_us:0, pose_age_us:0,
  };
  return {
    type:'navigation', sequence, time_us:timeUs, boot_id:task.boot_id, odom_domain:task.odom_domain,
    task_id:task.task_id, last_task_id:task.last_task_id, map_version:1, now_us:timeUs, expiry_us:task.expiry_us,
    status:task.status, error:task.error, mode:task.mode, flags:task.flags,
    vx_mm_s:0, vy_mm_s:0, wz_mrad_s:0, rejected:false, rx_rejection:false, active_task:active,
    cancelled_task_id:task.cancelled_task_id ?? 0, cancel_transaction_id:task.cancel_transaction_id ?? 0,
    goal_x_q16:task.goal_x_q16, goal_y_q16:task.goal_y_q16, candidate_count:0,
    map_age_us:0, pose_age_us:0,
  };
}

// 离线模拟只提供两点方向预览，不能视为真实设备通过碰撞检查的曲线。
function gridPlanData(sequence, timeUs, task) {
  const start={x:1.25,y:-0.5};
  const dx=task.goal_x_q16/65536-start.x,dy=task.goal_y_q16/65536-start.y;
  const fraction=Math.min(1,0.9/(Math.hypot(dx,dy)||1));
  return {type:'local_plan',sequence,time_us:timeUs,protocol:'nav_local_segment_v1',
    boot_id:task.boot_id,odom_domain:task.odom_domain,task_id:task.task_id,plan_stamp_us:timeUs,
    flags:task.status===3 ? 9 : 8,valid:task.status===3,guard_stop:false,grid_direction_preview:true,map_version:1,
    frame_id:'odom',source:'fpga',points:[start,{x:start.x+dx*fraction,y:start.y+dy*fraction}]};
}

// 仅供离线仿真，模拟设备发送的完整 2-bit 栅格快照。
function localGridData(sequence,timeUs,task) {
  const packed=Array(1024).fill(0);
  for (let y=8;y<56;y++) for (let x=8;x<56;x++) {
    const value=(x===52 && y>=16 && y<49) ? 2 : 1, index=y*64+x;
    packed[index>>2] |= value << (2*(index&3));
  }
  return {type:'local_grid',protocol:'nav_grid_snapshot_v1',sequence,time_us:timeUs,
    boot_id:task.boot_id,odom_domain:task.odom_domain,snapshot_id:sequence,
    capture_begin_us:timeUs,capture_end_us:timeUs,grid_generation:sequence,
    scan_generation:sequence+1,origin_x_cell:-7,origin_y_cell:-42,
    complete_scan_stamp_us:timeUs,last_ray_stamp_us:timeUs,grid_frame_epoch:1,
    transform_revision:1,flags:7,width:64,height:64,resolution_mm:50,packed_cells:packed,
    grid_from_odom:{map_anchor_x_mm:0,map_anchor_y_mm:0,odom_anchor_x_mm:0,odom_anchor_y_mm:0,yaw_offset_mrad:0}};
}

export class JetsonSimulator extends EventEmitter {
  constructor(options = {}) {
    super();
    this.options = {
      bind: '127.0.0.1', port: 16200, videoFps: 10, telemetryIntervalMs: 100,
      statusIntervalMs: 100, dropEvery: 0, duplicateEvery: 0, reorderEvery: 0,
      stopStreamAfterMs: 0, stopTelemetryAfterMs: 0, stopVideoAfterMs: 0, restartAfterMs: 0,
      navigationArchitecture:'legacy', quiet: false, ...options,
    };
    this.socket = dgram.createSocket('udp4');
    this.socket.on('message', (data, remote) => this.onControl(data, remote));
    this.socket.on('error', error => this.emitEvent('socket_error', { message: error.message }));
    this.image = options.image ?? fs.readFileSync(IMAGE_FILE);
    this.bridgeId = makeId();
    this.epoch = 1;
    this.session = null;
    this.retiredSessions = new Set();
    this.commandSequence = null;
    this.commandAt = null;
    this.peer = null;
    this.command = { seq: 0, velocity: [0, 0, 0], auto: false };
    this.autoRequested = false;
    this.autoAcknowledged = false;
    this.autoAckAt = 0;
    this.pendingAck = null;
    this.lastAck = null;
    this.ackCount = 0;
    this.timeouts = 0;
    this.lastConsoleCommandKey = null;
    this.lastConsoleCommandAt = 0;
    this.statusSequence = 0;
    this.visionSequence = 0;
    this.rosAuthorized = false;
    this.emergencyStop = false;
    this.missionState = 'IDLE';
    this.pcControl = false;
    this.streamSequences = new Map(['imu', 'motor', 'lidar', 'fusion', 'health', 'navigation', 'local_plan', 'local_grid'].map(stream => [stream, 0]));
    this.navigationTask = {boot_id:(Date.now() >>> 0) || 1, odom_domain:1, task_id:0, last_task_id:0, status:0,
      error:0, mode:0, flags:0, expiry_us:0, goal_x_q16:0, goal_y_q16:0};
    this.navigationTaskIdWatermark = 0;
    this.navigationStartedAt = process.hrtime.bigint();
    this.videoId = crypto.randomBytes(4).readUInt32BE(0);
    this.timers = [];
    this.reorderHold = { telemetry: null, video: null };
    this.faultCounters = { telemetry: 0, video: 0 };
    this.closed = false;
    this.telemetryEnabled = true;
    this.videoEnabled = true;
    this.statusEnabled = true;
    this.videoPackets = 0;
    this.videoFrames = 0;
    this.videoDropped = 0;
  }

  async start() {
    await new Promise((resolve, reject) => {
      this.socket.once('error', reject);
      this.socket.bind(this.options.port, this.options.bind, () => {
        this.socket.off('error', reject);
        resolve();
      });
    });
    this.startedAt = process.hrtime.bigint();
    this.startedWallAt = Date.now();
    this.timers.push(setInterval(() => this.sendStatus(), this.options.statusIntervalMs));
    this.timers.push(setInterval(() => this.sendTelemetry(), this.options.telemetryIntervalMs));
    this.timers.push(setInterval(() => this.sendVideo(), 1000 / this.options.videoFps));
    this.timers.push(setInterval(() => this.enforceLease(), 25));
    this.timers.push(setInterval(() => this.applyScheduledFaults(), 25));
    if (this.options.stopStreamAfterMs > 0) this.timers.push(setTimeout(() => this.stopStreams(), this.options.stopStreamAfterMs));
    if (this.options.stopTelemetryAfterMs > 0) this.timers.push(setTimeout(() => { this.telemetryEnabled = false; this.emitEvent('telemetry_stopped', {}); }, this.options.stopTelemetryAfterMs));
    if (this.options.stopVideoAfterMs > 0) this.timers.push(setTimeout(() => { this.videoEnabled = false; this.emitEvent('video_stopped', {}); }, this.options.stopVideoAfterMs));
    if (this.options.restartAfterMs > 0) this.timers.push(setTimeout(() => this.restartBridge(), this.options.restartAfterMs));
    this.emitEvent('simulator_started', { bind: this.options.bind, port: this.socket.address().port, bridge_id: this.bridgeId, jpeg_bytes: this.image.length });
    return this.socket.address();
  }

  async close() {
    if (this.closed) return;
    this.closed = true;
    for (const timer of this.timers) clearInterval(timer);
    for (const timer of this.timers) clearTimeout(timer);
    this.timers = [];
    await new Promise(resolve => this.socket.close(() => resolve()));
    this.emitEvent('simulator_stopped', {});
  }

  emitEvent(event, detail) {
    const record = { at: new Date().toISOString(), event, ...detail };
    this.emit('event', record);
    let writeToConsole = true;
    if (event === 'command_received') {
      const key = JSON.stringify([record.session, record.vx, record.vy, record.wz, record.auto]);
      const now = Date.now();
      writeToConsole = key !== this.lastConsoleCommandKey || now - this.lastConsoleCommandAt >= 1000;
      if (writeToConsole) {
        this.lastConsoleCommandKey = key;
        this.lastConsoleCommandAt = now;
      }
    }
    if (!this.options.quiet && writeToConsole) process.stdout.write(`[SIM] ${JSON.stringify(record)}\n`);
  }

  getSnapshot() {
    const expired = this.commandAt === null || performance.now() - this.commandAt > 250;
    return {
      bridge_id: this.bridgeId,
      session: this.session,
      command: expired ? { ...this.command, velocity: [0, 0, 0], auto: false } : structuredClone(this.command),
      auto_requested: expired ? false : this.autoRequested,
      auto_acknowledged: expired ? false : this.autoAcknowledged,
      last_ack: this.lastAck ? structuredClone(this.lastAck) : null,
      ack_count: this.ackCount,
      timeouts: this.timeouts,
      video_frames: this.videoFrames,
      video_packets: this.videoPackets,
      telemetry_sequences: Object.fromEntries(this.streamSequences),
      peer: this.peer,
    };
  }

  waitForCommand(predicate, timeoutMs = 1500) {
    const current = this.getSnapshot();
    if (predicate(current)) return Promise.resolve(current);
    return new Promise((resolve, reject) => {
      const timer = setTimeout(() => { this.off('command', listener); reject(new Error('simulator command wait timed out')); }, timeoutMs);
      const listener = snapshot => {
        if (!predicate(snapshot)) return;
        clearTimeout(timer);
        this.off('command', listener);
        resolve(snapshot);
      };
      this.on('command', listener);
    });
  }

  acceptNavigationTaskId(value) {
    if (!u32(value) || value === 0 ||
        (this.navigationTaskIdWatermark !== 0 && !newer(value, this.navigationTaskIdWatermark))) return null;
    this.navigationTaskIdWatermark = value;
    return value;
  }

  onControl(packet, remote) {
    let message;
    try { message = JSON.parse(packet.toString('utf8')); } catch { this.emitEvent('command_rejected', { reason: 'invalid-json' }); return; }
    if (message?.type === 'mission_command') {
      const actions = ['enable_ros','disable_ros','start','stop','emergency_stop','release_emergency_stop',
        'local_start','local_stop','local_cancel'];
      if (!actions.includes(message.action) || typeof message.request_id !== 'string' || message.request_id.length > 64 ||
          (message.session === this.session && this.commandSequence !== null && !newer(message.seq,this.commandSequence))) return;
      this.onControl(jsonBuffer({...message, type:'control', vx:0, vy:0, wz:0, auto:false, take_control:false}),remote);
      if (this.session !== message.session || this.commandSequence !== message.seq) return;
      let success = true, detail = '离线模拟只确认请求处理，不代表真实 FPGA 执行或机械运动';
      let transactionTaskId = null, cancelledTaskId = 0;
      if (message.action === 'enable_ros') {
        success = !this.emergencyStop;
        if (success) this.rosAuthorized = true;
      } else if (message.action === 'disable_ros') this.rosAuthorized = false;
      else if (message.action === 'start') {
        success = !this.emergencyStop && ['follow','approach','search','search_only'].includes(message.behavior) &&
          typeof message.class_id === 'string' && message.class_id.length > 0 &&
          (message.target_id === null || typeof message.target_id === 'string') && ['', 'red'].includes(message.color);
        if (success) { this.rosAuthorized = true; this.missionState = {follow:'FOLLOWING',approach:'APPROACHING',search:'SEARCHING',search_only:'FOUND'}[message.behavior]; }
      } else if (message.action === 'stop' || message.action === 'emergency_stop') {
        if (message.action === 'emergency_stop') this.emergencyStop=true;
        this.missionState='STOPPED'; this.rosAuthorized=false;
        if ((this.navigationTask.status >= 1 && this.navigationTask.status <= 3) || this.navigationTask.status === 5)
          this.navigationTask = {...this.navigationTask, status:this.options.navigationArchitecture === 'grid' ? 7 : 9, mode:0, flags:0};
      }
      else if (message.action === 'release_emergency_stop') this.emergencyStop=false;
      else if (message.action === 'local_start') {
        success = !this.emergencyStop &&
          !((this.navigationTask.status >= 1 && this.navigationTask.status <= 3) || this.navigationTask.status === 5);
        if (success) {
          transactionTaskId = this.acceptNavigationTaskId(message.task_id);
          success = transactionTaskId !== null;
        }
        if (success) {
          this.rosAuthorized = false; this.pcControl = false; this.missionState = 'STOPPED';
          const nowUs = this.navigationNowUs();
          this.navigationTask = {boot_id:this.navigationTask.boot_id, odom_domain:this.navigationTask.odom_domain,
            task_id:transactionTaskId, last_task_id:transactionTaskId, status:this.options.navigationArchitecture === 'grid' ? 3 : 5, error:0,
            mode:message.mode === 'independent' ? 3 : 2, flags:message.allow_disconnect === true ? 1 : 0,
            expiry_us:nowUs + Math.round(Number(message.duration_s) * 1e6),
            goal_x_q16:Math.round(Number(message.goal_x_m) * 65536),
            goal_y_q16:Math.round(Number(message.goal_y_m) * 65536)};
        }
      } else if (message.action === 'local_stop' || message.action === 'local_cancel') {
        success = (this.navigationTask.status >= 1 && this.navigationTask.status <= 3) || this.navigationTask.status === 5;
        if (success) {
          cancelledTaskId = this.navigationTask.task_id;
          transactionTaskId = this.acceptNavigationTaskId(message.task_id);
          success = transactionTaskId !== null;
          if (!success) cancelledTaskId = 0;
        }
        if (success) {
          this.navigationTask = {...this.navigationTask, last_task_id:transactionTaskId,
            task_id:this.options.navigationArchitecture === 'grid' ? transactionTaskId : this.navigationTask.task_id,
            status:this.options.navigationArchitecture === 'grid' ? 7 : 9, error:0, mode:0, flags:0,
            cancelled_task_id:cancelledTaskId, cancel_transaction_id:transactionTaskId};
        }
      }
      if (!success) detail='离线模拟：请检查授权、急停和 FPGA 局部任务状态';
      this.sendPacket(jsonBuffer({v:1,type:'mission_result',session:this.session,
        request_id:message.request_id,action:message.action,success,message:detail,
        ...(transactionTaskId ? {transaction_task_id:transactionTaskId} : {}),
        ...(message.action === 'local_start' && transactionTaskId ? {task_id:transactionTaskId} : {}),
        ...(cancelledTaskId ? {cancelled_task_id:cancelledTaskId} : {})}),remote);
      return;
    }
    if (!message || message.v !== 1 || message.type !== 'control' || typeof message.session !== 'string' ||
        !u32(message.seq) || !['vx', 'vy', 'wz'].every(key => Number.isSafeInteger(message[key])) ||
        (message.auto !== undefined && typeof message.auto !== 'boolean')) {
      this.emitEvent('command_rejected', { reason: 'invalid-control' });
      return;
    }
    const velocity = [message.vx, message.vy, message.wz];
    if (message.take_control !== undefined && typeof message.take_control !== 'boolean') return;
    const takeControl = message.take_control !== false;
    if (message.auto === true) {
      this.emitEvent('command_rejected', { reason:'legacy-auto-disabled', detail:'请提交FPGA局部odom任务' });
      return;
    }
    if ((!takeControl && (velocity.some(Boolean) || message.auto === true)) || (takeControl && this.emergencyStop)) return;
    const sessionChanged = message.session !== this.session;
    if (sessionChanged) {
      if (velocity.some(Boolean) || this.retiredSessions.has(message.session)) {
        this.emitEvent('command_rejected', { reason: 'session-change-needs-zero', session: message.session });
        return;
      }
      if (this.session) {
        this.retiredSessions.add(this.session);
        if (this.retiredSessions.size > 32) this.retiredSessions.delete(this.retiredSessions.values().next().value);
      }
      this.session = message.session;
      this.commandSequence = null;
      this.command = { seq: message.seq, velocity: [0, 0, 0], auto: false };
      this.autoRequested = false;
      this.autoAcknowledged = false;
      this.emitEvent('session_established', { session: message.session });
    }
    if (this.commandSequence !== null && !newer(message.seq, this.commandSequence)) return;
    if (this.commandSequence === null && velocity.some(Boolean)) return;
    if (velocity.some((value, index) => Math.abs(value) > (index === 2 ? 1000 : 400))) {
      this.emitEvent('command_rejected', { reason: 'hard-limit', session: message.session });
      return;
    }
    this.peer = { address: remote.address, port: remote.port };
    this.pcControl = takeControl;
    if (takeControl) this.rosAuthorized = false;
    this.commandSequence = message.seq;
    this.commandAt = performance.now();
    this.command = { seq: message.seq, velocity, auto: false };
    this.autoRequested = false;
    this.autoAcknowledged = false;
    this.autoAckAt = performance.now() + 20;
    this.ackCount += 1;
    this.pendingAck = {
      type: 3,
      fpga_seq: this.ackCount >>> 0,
      pc_seq: message.seq,
      pc_session: message.session,
      source: takeControl ? 'pc' : 'none',
      velocity,
      jetson_rx_mono_s: monoSeconds(this.startedAt),
    };
    this.emitEvent('command_received', { session: message.session, seq: message.seq, vx: velocity[0], vy: velocity[1], wz: velocity[2], auto: false });
    const snapshot = this.getSnapshot();
    this.emit('command', snapshot);
  }

  enforceLease() {
    if (this.rosAuthorized && this.commandAt !== null && performance.now()-this.commandAt > 500) this.rosAuthorized=false;
    if (this.commandAt !== null && performance.now() - this.commandAt > 250 &&
        (this.command.velocity.some(Boolean) || this.autoRequested)) {
      this.command = { ...this.command, velocity: [0, 0, 0], auto: false };
      this.autoRequested = false;
      this.autoAcknowledged = false;
      this.timeouts += 1;
      this.emitEvent('lease_expired', { session: this.session, seq: this.command.seq });
    }
    if (this.pendingAck && performance.now() >= this.autoAckAt) {
      this.lastAck = this.pendingAck;
      this.pendingAck = null;
      if (this.autoRequested) this.autoAcknowledged = true;
    }
  }

  applyScheduledFaults() {
    if (this.options.stopStreamAfterMs > 0 && Date.now() - this.startedWallAt >= this.options.stopStreamAfterMs && this.telemetryEnabled) this.stopStreams();
  }

  stopStreams() {
    this.telemetryEnabled = false;
    this.videoEnabled = false;
    this.statusEnabled = false;
    this.emitEvent('streams_stopped', {});
  }

  restartBridge() {
    this.visionSequence=0; this.rosAuthorized=false; this.emergencyStop=false; this.missionState='IDLE'; this.pcControl=false;
    this.bridgeId = makeId();
    this.epoch = (this.epoch + 1) >>> 0;
    this.session = null;
    this.retiredSessions.clear();
    this.commandSequence = null;
    this.commandAt = null;
    this.command = { seq: 0, velocity: [0, 0, 0], auto: false };
    this.autoRequested = false;
    this.autoAcknowledged = false;
    this.pendingAck = null;
    this.lastAck = null;
    this.statusSequence = 0;
    for (const stream of this.streamSequences.keys()) this.streamSequences.set(stream, 0);
    this.startedAt = process.hrtime.bigint();
    this.startedWallAt = Date.now();
    this.telemetryEnabled = true;
    this.videoEnabled = true;
    this.statusEnabled = true;
    this.videoId = crypto.randomBytes(4).readUInt32BE(0);
    this.reorderHold = { telemetry: null, video: null };
    this.emitEvent('bridge_restarted', { bridge_id: this.bridgeId, epoch: this.epoch });
  }

  sendPacket(packet, peer, lane = null) {
    if (!peer || this.closed) return;
    if (!lane) { this.socket.send(packet, peer.port, peer.address); return; }
    const counter = ++this.faultCounters[lane];
    if (this.options.dropEvery > 0 && counter % this.options.dropEvery === 0) {
      this.emitEvent('fault_drop', { lane, counter });
      return;
    }
    const delayed = this.reorderHold[lane];
    if (this.options.reorderEvery > 0 && counter % this.options.reorderEvery === 0) {
      if (delayed) this.socket.send(delayed.packet, delayed.peer.port, delayed.peer.address);
      this.reorderHold[lane] = { packet: Buffer.from(packet), peer };
      this.emitEvent('fault_reorder_hold', { lane, counter });
      return;
    }
    this.socket.send(packet, peer.port, peer.address);
    if (delayed) {
      this.socket.send(delayed.packet, delayed.peer.port, delayed.peer.address);
      this.reorderHold[lane] = null;
      this.emitEvent('fault_reorder_release', { lane, counter });
    }
    if (this.options.duplicateEvery > 0 && counter % this.options.duplicateEvery === 0) {
      this.socket.send(packet, peer.port, peer.address);
      this.emitEvent('fault_duplicate', { lane, counter });
    }
  }

  sendStatus() {
    if (!this.peer || !this.session || !this.statusEnabled) return;
    this.enforceLease();
    this.statusSequence = (this.statusSequence + 1) >>> 0;
    const age = this.commandAt === null ? 1 : Math.max(0, (performance.now() - this.commandAt) / 1000);
    const velocity = this.command.velocity;
    const status = {
      v: 1, type: 'status', session: this.session, pc_seq: this.command.seq,
      ready: true, pc_timeout: age > 0.25, fpga_ack_age: Math.min(age, 0.05),
      ack_count: this.ackCount, timeouts: this.timeouts, velocity,
      frames: this.videoFrames, video_dropped: this.videoDropped, video_gaps: 0,
      video_age: this.videoFrames ? Math.max(0, (Date.now() - this.lastVideoAt) / 1000) : null,
      jpeg_frames: this.videoFrames, motor: motorData(this.streamSequences.get('motor'), (Date.now() * 1000) >>> 0, this.command),
      bridge_id: this.bridgeId, seq: this.statusSequence,
      jetson_tx_mono_s: monoSeconds(this.startedAt),
      auto_requested: this.autoRequested,
      auto_acknowledged: this.autoAcknowledged,
      control_source: this.emergencyStop ? 'emergency_stop' : this.rosAuthorized ? 'ros' : this.pcControl ? 'pc' : 'none',
      last_ack: this.lastAck ? structuredClone(this.lastAck) : null,
      // 离线源自身生成 fusion 的模式位；停掉遥测时不继续报告有效实际模式。
      actual_mode: this.telemetryEnabled ? (this.autoRequested ? 'auto' : 'manual') : null,
    };
    this.socket.send(jsonBuffer(status), this.peer.port, this.peer.address);
    const timestamp = Date.now();
    const box = {id:'simulated:1',class_id:'0',score:0.9,cx:500,cy:240,width:120,height:180};
    this.sendPacket(jsonBuffer({v:1,type:'vision',bridge_id:this.bridgeId,session:this.session,
      seq:this.visionSequence++ >>> 0,mission_state:this.missionState,
      control_source:this.emergencyStop?'emergency_stop':this.rosAuthorized?'ros':this.pcControl?'pc':'none',
      ros_authorized:this.rosAuthorized,emergency_stop:this.emergencyStop,frame_id:'simulated_camera',
      stamp:{sec:Math.floor(timestamp/1000),nanosec:(timestamp%1000)*1e6},
      width:this.options.visionWidth ?? 800,height:this.options.visionHeight ?? 480,
      age_ms:0,stale:false,detections:this.options.visionDetections ?? [box],
      selected_target:this.options.visionDetections ? null : ['TRACKING','FOLLOWING','APPROACHING','FOUND'].includes(this.missionState)?box:null,
      truncated:false}),this.peer);
  }

  sendTelemetry() {
    if (!this.peer || !this.telemetryEnabled) return;
    const tx = monoSeconds(this.startedAt);
    const timeUs64 = this.navigationNowUs();
    const timeUs = timeUs64 >>> 0;
    const grid=this.options.navigationArchitecture === 'grid';
    if (this.navigationTask.expiry_us > 0 && timeUs64 >= this.navigationTask.expiry_us &&
        ((this.navigationTask.status >= 1 && this.navigationTask.status <= 3) || this.navigationTask.status === 5))
      this.navigationTask = {...this.navigationTask, status:grid ? 8 : 6, error:0, mode:0, flags:0};
    const factories = { imu: imuData, motor: motorData, lidar: lidarData,
      navigation:(sequence) => navigationData(sequence,timeUs64,this.navigationTask,grid),
      fusion:(sequence) => fusionData(sequence,timeUs,this.command,grid), health: healthData,
      ...(grid ? {local_plan:(sequence) => gridPlanData(sequence,timeUs,this.navigationTask),
        local_grid:(sequence) => localGridData(sequence,timeUs,this.navigationTask)} : {}) };
    for (const [stream, build] of Object.entries(factories)) {
      if (stream === 'local_grid' && this.lastGridAt !== undefined && performance.now()-this.lastGridAt < 190) continue;
      if (stream === 'local_grid') this.lastGridAt=performance.now();
      const sequence = this.streamSequences.get(stream);
      this.streamSequences.set(stream, (sequence + 1) >>> 0);
      const data = build(sequence, timeUs, this.command);
      const base = {
        v: 1, type: 'telemetry', bridge_id: this.bridgeId, stream, epoch: this.epoch,
        source_protocol: 'BX72', source_version: 1,
        valid: true, jetson_rx_mono_s: tx, jetson_tx_mono_s: tx, data,
      };
      const packet = jsonBuffer(base);
      if (packet.length <= 1400) {
        this.sendPacket(packet, this.peer, 'telemetry');
      } else {
        const parts = stream === 'local_grid' ? Array.from({length:8},(_,part) => ({...base,part,parts:8,
          data:{...data,packed_cells:data.packed_cells.slice(part*128,(part+1)*128)}})) : this.fragmentTelemetry(base);
        for (const part of parts) {
          const encoded = jsonBuffer(part);
          if (encoded.length > 1400) {
            this.emitEvent('telemetry_too_large', { stream, bytes: encoded.length });
            break;
          }
          this.sendPacket(encoded, this.peer, 'telemetry');
        }
      }
    }
  }

  navigationNowUs() {
    return Math.floor(Number(process.hrtime.bigint() - this.navigationStartedAt) / 1000);
  }

  fragmentTelemetry(base) {
    const coreKeys = new Set(['type', 'sequence', 'time_us']);
    const core = Object.fromEntries(Object.entries(base.data).filter(([key]) => coreKeys.has(key)));
    const fields = Object.entries(base.data).filter(([key]) => !coreKeys.has(key));
    const chunks = [];
    let current = { ...core };
    for (const [key, value] of fields) {
      const candidate = { ...current, [key]: value };
      const probe = jsonBuffer({ ...base, data: candidate, part: 31, parts: 32 });
      if (probe.length > 1400 && Object.keys(current).length > coreKeys.size) {
        chunks.push(current);
        current = { ...core, [key]: value };
      } else current = candidate;
    }
    chunks.push(current);
    return chunks.map((data, part) => ({ ...base, data, part, parts: chunks.length }));
  }

  sendVideo() {
    if (!this.peer || !this.videoEnabled) return;
    this.videoId = (this.videoId + 1) >>> 0;
    const count = Math.ceil(this.image.length / JPEG_CHUNK);
    if (count < 1 || count > Math.ceil(512 * 1024 / JPEG_CHUNK)) return;
    for (let index = 0; index < count; index += 1) {
      const start = index * JPEG_CHUNK;
      const fragment = this.image.subarray(start, start + JPEG_CHUNK);
      const header = Buffer.allocUnsafe(HEADER_BYTES);
      header.write('BXJV', 0, 4, 'ascii');
      header.writeUInt32BE(this.videoId, 4);
      header.writeUInt16BE(index, 8);
      header.writeUInt16BE(count, 10);
      this.sendPacket(Buffer.concat([header, fragment]), this.peer, 'video');
      this.videoPackets += 1;
    }
    this.videoFrames += 1;
    this.lastVideoAt = Date.now();
  }
}

export async function createSimulator(options = {}) {
  const simulator = new JetsonSimulator(options);
  await simulator.start();
  return simulator;
}

function readArgs(argv) {
  const result = {};
  for (let index = 0; index < argv.length; index += 1) {
    const [keyRaw, inline] = argv[index].slice(2).split('=', 2);
    const key = keyRaw.replaceAll('-', '_');
    const next = argv[index + 1];
    const value = inline ?? (next && !next.startsWith('--') ? argv[++index] : undefined);
    result[key] = value === undefined ? true : value;
  }
  return result;
}

if (process.argv[1] && path.resolve(process.argv[1]) === fileURLToPath(import.meta.url)) {
  const args = readArgs(process.argv.slice(2));
  const simulator = await createSimulator({
    bind: args.bind ?? '127.0.0.1',
    port: Number(args.port ?? 16200),
    videoFps: Number(args.video_fps ?? 10),
    dropEvery: Number(args.drop_every ?? 0),
    duplicateEvery: Number(args.duplicate_every ?? 0),
    reorderEvery: Number(args.reorder_every ?? 0),
    stopStreamAfterMs: Number(args.stop_stream_after_ms ?? 0),
    stopTelemetryAfterMs: Number(args.stop_telemetry_after_ms ?? 0),
    stopVideoAfterMs: Number(args.stop_video_after_ms ?? 0),
    restartAfterMs: Number(args.restart_after_ms ?? 0),
    navigationArchitecture: args.navigation_architecture ?? 'legacy',
  });
  process.stdout.write(`离线模拟 Jetson UDP ${args.bind ?? '127.0.0.1'}:${simulator.socket.address().port}; simulation=true\n`);
  const shutdown = async () => { await simulator.close(); process.exit(0); };
  process.once('SIGINT', shutdown);
  process.once('SIGTERM', shutdown);
}
