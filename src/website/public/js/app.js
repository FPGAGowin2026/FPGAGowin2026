/* BX72 独立网站交互脚本。
   网络 UDP 适配器、六类遥测、视频流、控制授权 / 停车证据、
   局部导航任务、建图与轨迹、轮速趋势与诊断均通过同一网关；
   单页按监看、任务、人工控制、传感器和诊断分区，导航只做锚点定位。
   经典脚本（非 module），依赖 BX72DashboardNetwork 与 BX72OccupancyMap。 */
(() => {
  'use strict';

  const labels = [
    ['x', 'm'], ['y', 'm'], ['航向 yaw', '°'], ['前后速度 vx', 'm/s'],
    ['左右速度 vy', 'm/s'], ['转向角速度 wz', 'rad/s'], ['陀螺零偏 bgz', 'rad/s']
  ];
  const diagnosticFields = [
    ['位姿更新数', 'fusion', 'updates'],
    ['CPU 故障码', 'fusion', 'fault_code'],
    ['最长控制间隔', 'fusion', 'max_control_gap_us', value => formatDiagnostic(value, 'µs')],
    ['CSM 接受数', 'fusion', 'csm_accepted'],
    ['CSM 拒绝数', 'fusion', 'csm_rejected'],
    ['启动算法自检', 'fusion', 'startup_algorithm_selftest_passed', value => value === true ? '通过' : value === false ? '未通过' : '不可用'],
    ['IMU 数据年龄', 'fusion', 'imu_age_us', value => formatDiagnostic(value, 'µs')],
    ['雷达数据年龄', 'fusion', 'radar_age_us', value => formatDiagnostic(value, 'µs')],
    ['位姿源时间', 'fusion', 'pose_stamp_us', value => formatDiagnostic(value, 'µs')],
    ['建图队列丢点', 'fusion', 'map_lidar_drops'],
    ['UDP 原始 flags', 'fusion', 'flags', value => formatDiagnostic(value, 8)],
    ['电机队列丢弃', 'motor', 'queue_dropped'],
    ['编码器无效掩码', 'motor', 'encoder_invalid_mask', value => formatDiagnostic(value, 2)],
    ['IMU 身份错误', 'imu', 'identity_errors'],
    ['健康队列丢弃', 'health', 'queue_dropped'],
    ['构图射线队列', 'fusion', 'map_ray_queue_depth', value => formatDiagnostic(value, '/32')],
    ['构图丢射线（32767饱和）', 'fusion', 'map_ray_drops'],
    ['构图溢出停车', 'fusion', 'map_ray_overflow', value => value === true ? '是' : value === false ? '否' : '不可用'],
    ['滚动地图等待', 'fusion', 'map_shift_pending', value => value === true ? '是' : value === false ? '否' : '不可用'],
    ['CPU 就绪', 'health', 'cpu_ready', value => value === true ? '是' : value === false ? '否' : '不可用'],
    ['CPU 硬件故障', 'health', 'cpu_fault', value => value === true ? '是' : value === false ? '否' : '不可用'],
    ['CPU 硬件故障码', 'health', 'cpu_fault_code'],
    ['构图方式', 'fusion', 'map_update_mode'],
    ['构图扫描行', 'fusion', 'map_raster_row', value => formatDiagnostic(value, '/64')],
    ['构图输入缺口（32767饱和）', 'fusion', 'map_input_gaps'],
    ['构图覆盖恢复等待', 'fusion', 'map_input_gap_latched', value => value === true ? '是' : value === false ? '否' : '不可用'],
    ['导航架构', 'fusion', 'architecture'],
    ['雷达原始样本', 'fusion', 'lidar_raw_samples'],
    ['5°聚合样本', 'fusion', 'lidar_reduced_samples'],
    ['极坐标扫描新鲜', 'fusion', 'polar_scan_fresh', value => value === true ? '是' : value === false ? '否' : '不可用'],
    ['极坐标完整覆盖', 'fusion', 'polar_complete_coverage', value => value === true ? '是' : value === false ? '否' : '不可用'],
    ['极坐标缺口锁', 'fusion', 'polar_gap_latched', value => value === true ? '是' : value === false ? '否' : '不可用'],
    ['边界绕障中', 'fusion', 'boundary_following', value => value === true ? '是' : value === false ? '否' : '不可用'],
    ['扫掠方向未知', 'fusion', 'unknown_in_swept_corridor', value => value === true ? '是' : value === false ? '否' : '不可用'],
    ['局部图新鲜', 'fusion', 'map_fresh', value => value === true ? '是' : value === false ? '否' : '不可用'],
    ['局部图完整圈就绪', 'fusion', 'map_ready', value => value === true ? '是' : value === false ? '否' : '不可用'],
    ['局部图输入缺口锁', 'fusion', 'map_gap_latched', value => value === true ? '是' : value === false ? '否' : '不可用'],
    ['最近地图缺口原因', 'fusion', 'map_gap_reason_name', value => gridGapReason(value)],
    ['启动区许可失效', 'fusion', 'start_confirmation_lost', value => value === true ? '是，请检查新起点与地图' : value === false ? '否' : '不可用'],
    ['候选评分中', 'fusion', 'candidate_planning', value => value === true ? '是' : value === false ? '否' : '不可用'],
    ['进度受阻', 'fusion', 'progress_blocked', value => value === true ? '是' : value === false ? '否' : '不可用'],
    ['候选路径遇未知', 'fusion', 'unknown_path', value => value === true ? '是' : value === false ? '否' : '不可用'],
    ['等待完整新扫描', 'fusion', 'map_recovery_required', value => value === true ? '是' : value === false ? '否' : '不可用'],
    ['相机校准状态', 'health', 'camera_calibration.state'],
    ['相机校准就绪', 'health', 'camera_calibration.ready', value => value === true ? '是' : value === false ? '否' : '不可用'],
    ['相机校准故障', 'health', 'camera_calibration.fault', value => value === true ? '是' : value === false ? '否' : '不可用'],
    ['FPGA视频包数', 'health', 'video_packets'],
    ['FPGA运动锁已解开', 'fusion', 'motion_unlocked', value => value === true ? '是' : value === false ? '否' : '不可用']
  ];
  function formatDiagnostic(value, format) {
    if (!Number.isFinite(value)) return '不可用';
    return typeof format === 'number' ? '0x' + value.toString(16).padStart(format, '0') : value + ' ' + format;
  }
  function gridGapReason(value) {
    return ({none:'未记录',fifo_overflow:'地图队列满',upstream_error:'雷达解析或跨域错误',
      sample_stale:'样本超时',pose_unavailable:'采样位姿不可用',
      angle_discontinuity:'角度不连续',invalid_sample:'样本无效',
      coordinate_reset:'地图坐标重置'})[value] || '不可用';
  }

  const networkApi = typeof BX72DashboardNetwork !== 'undefined' ? BX72DashboardNetwork : null;
  if (!networkApi) {
    const event = document.getElementById('networkEvent');
    if (event) event.textContent = '未加载 dashboard_network.js：页面脚本无法连接遥测链路。';
    return;
  }

  // ---------- DOM 访问助手：新页面里可选的元素缺失时安全跳过 ----------
  const $ = id => document.getElementById(id);
  function setText(id, value) { const element = $(id); if (element) element.textContent = value; }
  function setDisabled(id, value) { const element = $(id); if (element) element.disabled = !!value; }
  function bind(id, type, handler, options) {
    const element = $(id);
    if (element && typeof element.addEventListener === 'function') element.addEventListener(type, handler, options);
  }
  function setChip(id, text, state) {
    const chip = $(id);
    if (!chip) return null;
    chip.textContent = text;
    if (chip.dataset) chip.dataset.state = state || '';
    // 视频 HUD 的圆点在 id 文本节点之外：同步给 .hud-stream 里的 .hud-dot（CSS 已定义 data-state 配色）。
    if (typeof chip.closest === 'function') {
      const stream = chip.closest('.hud-stream');
      const dot = stream && typeof stream.querySelector === 'function' ? stream.querySelector('.hud-dot') : null;
      if (dot && dot.dataset) dot.dataset.state = state || 'unknown';
    }
    return chip;
  }
  const cards = $('stateCards');
  const diagBox = $('diagnostics');
  const occupancyView = BX72OccupancyMap.createView({document, window,
    onSelectGoal:selectMapGoal,
    onGlobalGoalInvalidated:invalidateGlobalGoals,
    onViewChange:() => updateLocalTaskUi(),
    onMapChange:() => updateLocalTaskUi()});
  const globalMapSource = typeof BX72GlobalMap !== 'undefined' ? BX72GlobalMap.createSource({window,
    onUpdate:update => occupancyView.setGlobalMap(update)}) : null;
  globalMapSource?.start();

  const pressedNetworkKeys = new Set();
  let networkSpeed = 100, networkTurn = 300, networkHeartbeat = 0;
  let networkTargetEdited = false, networkSimulation = false, networkTargetPending = null;
  let networkStateReceivedAt = 0, networkArmPendingUntil = 0;
  let observedBridgeStatus = null;
  let networkStopLocalRequested = null;
  let networkVideoSocket = null, networkVideoObjectUrl = null, networkVideoFrames = [], networkVideoLastAt = 0, networkVideoDrops = 0;
  let networkVideoDecodedAt = 0, networkVideoWidth = 0, networkVideoHeight = 0;
  let visionPendingUntil = 0;
  let visionDraft = null, visionOptionsKey = '';
  let mapGoals = [], mapGoalFrame = null, mapGoalDomain = null, mapGoalGeneration = null, mapGoalNotice = '';
  let navigationDraftSubmitted = null;
  const NETWORK_STREAM_MAX_AGE = {imu:1000,motor:1000,lidar:1000,fusion:1000,health:2000,navigation:500};
  let networkTelemetryCount = 0;
  let csvRows = [];
  let navigationPendingUntil = 0;
  let lastNavigationRejectionKey = '';
  const networkAdapter = new networkApi.DashboardNetworkAdapter({
    onConnection:onNetworkConnection,
    onHello:onNetworkHello,
    onState:onNetworkState,
    onTelemetry:onNetworkTelemetry,
    onEvent:onNetworkEvent,
    onMissionResult:onDashboardMissionResult,
    onReset:resetNetworkTelemetry
  });
  // 30秒默认趋势窗口与浅色主题；占用地图使用独立的低频快照。
  const workbench = networkApi.createWorkbenchPresentation({
    document, window, redraw:occupancyView.draw,
    windowMs:30000,
    theme:{ grid:'#ececee', axis:'#6e6e73', lines:['#295fe7','#0aa5c9','#1d1d1f','#8b6cf0'], stale:'#fa932c' }
  });

  // ---------- 总览指标卡与火花线（DESIGN 5.5 / 6.2） ----------
  const METRIC_KEYS = ['x', 'y', 'yaw', 'vx'];
  const METRIC_SUFFIX = {x:'X', y:'Y', yaw:'Yaw', vx:'Vx'};
  const METRIC_UNITS = {x:'m', y:'m', yaw:'°', vx:'m/s'};
  const METRIC_DIGITS = {x:3, y:3, yaw:2, vx:3};
  const SPARK_LIMIT = 20;
  const SPARK_WIDTH = 72;
  const SPARK_HEIGHT = 24;
  const sparkHistory = {x:[], y:[], yaw:[], vx:[]};
  let overviewSequence = null;

  function clearSparkHistory() {
    for (const key of METRIC_KEYS) sparkHistory[key].length = 0;
  }

  // 72×24 viewBox 内归一化：横向按样本序号，纵向按本次窗口 min/max，留 2px 边距。
  function sparkCoordinates(values) {
    if (!Array.isArray(values) || values.length < 2) return [];
    let minimum = Infinity, maximum = -Infinity;
    for (const value of values) {
      if (!Number.isFinite(value)) continue;
      if (value < minimum) minimum = value;
      if (value > maximum) maximum = value;
    }
    if (!Number.isFinite(minimum)) return [];
    const span = maximum - minimum;
    return values.map((value, index) => {
      const x = values.length === 1 ? SPARK_WIDTH / 2 : SPARK_WIDTH * index / (values.length - 1);
      const fraction = span > 1e-12 ? (value - minimum) / span : 0.5;
      const y = SPARK_HEIGHT - 2 - fraction * (SPARK_HEIGHT - 4);
      return {x, y};
    });
  }

  function drawSpark(key, values) {
    const line = $('spark' + METRIC_SUFFIX[key]);
    if (!line || typeof line.setAttribute !== 'function') return;
    const coordinates = sparkCoordinates(values);
    line.setAttribute('points', coordinates.map(point =>
      point.x.toFixed(1) + ',' + point.y.toFixed(1)).join(' '));
  }

  function drawOverviewValues(values, sigmas, uncertaintyLabel = 'σ') {
    METRIC_KEYS.forEach((key, index) => {
      const suffix = METRIC_SUFFIX[key];
      const value = values ? values[index] : null;
      const sigma = sigmas ? sigmas[index] : null;
      setText('metricValue' + suffix, Number.isFinite(value) ? fmt(value, METRIC_DIGITS[key]) : '—');
      setText('metricSigma' + suffix,
        Number.isFinite(sigma) ? `${uncertaintyLabel} ${fmt(sigma, METRIC_DIGITS[key])} ${METRIC_UNITS[key]}` : `${uncertaintyLabel} —`);
    });
  }

  // values / sigmas 顺序与 labels 前四项一致：x、y、yaw(度)、vx。
  function updateOverviewMetrics(values, sigmas, sequence, uncertaintyLabel = 'σ') {
    drawOverviewValues(values, sigmas, uncertaintyLabel);
    if (sequence === overviewSequence) return;
    overviewSequence = sequence;
    METRIC_KEYS.forEach((key, index) => {
      const value = values ? values[index] : null;
      if (!Number.isFinite(value)) { sparkHistory[key].length = 0; drawSpark(key, []); return; }
      const list = sparkHistory[key];
      list.push(value);
      if (list.length > SPARK_LIMIT) list.shift();
      drawSpark(key, list);
    });
  }

  function clearOverviewMetrics() {
    overviewSequence = null;
    clearSparkHistory();
    drawOverviewValues(null, null);
    for (const key of METRIC_KEYS) drawSpark(key, []);
  }

  // 视频：无帧显示占位，有帧隐藏占位并显示图像（DESIGN 5.3）。
  function setVideoFrameVisible(ready) {
    const image = $('networkVideo');
    if (image) image.hidden = !ready;
    const placeholder = $('videoPlaceholder');
    if (placeholder) placeholder.hidden = !!ready;
  }

  // 顶部摘要只使用当前查看链路的接收状态，不参与授权或控制定时器。
  function freshNetworkStatus() {
    const state = networkAdapter.state;
    const jetson = state && state.jetson;
    return state && networkStateReceivedAt && performance.now()-networkStateReceivedAt <= 500 &&
      jetson && jetson.online && Number.isFinite(jetson.age_ms) && jetson.age_ms <= 1000
      ? state.status : null;
  }
  function updateWorkbench() {
    const now = performance.now();
    const connected = networkAdapter.connected;
    const motor = freshNetworkItem('motor', now);
    const status = freshNetworkStatus();
    const mode = status && status.actual_mode;
    workbench.update({source:'network', transportText:'网络 UDP', connected,
      simulation:networkAdapter.hello ? networkSimulation : null,
      connectionText:!connected ? '网络未连接' : !status ? '服务已连接 · 网关状态不可用' : '网络已连接 · Jetson 在线',
      actualModeText:`${mode === 'auto' ? '自动' : mode === 'manual' ? '手动' : '不可用'}`,
      wheels:'ABCD'.split('').map(key => motor?.data.wheel_mm_s?.[key] ?? null),
      fresh:!!motor, sampleAt:motor?.receivedAt ?? null, nowMs:now});
  }

  labels.forEach(([label, unit], i) => {
    const el = document.createElement('article');
    // 传感器区 7 项复用总览指标卡样式（DESIGN 5.5 / 5.9）；id 契约见 DESIGN 6.2。
    el.className = 'metric-card';
    const tone = METRIC_KEYS[i];
    if (tone) el.dataset.axis = tone;
    const tag = tone ? `<span class="metric-tag is-${tone}" aria-hidden="true"></span>` : '';
    el.innerHTML = `<div class="metric-label">${tag}${label}</div>` +
      `<div class="metric-readout"><span id="v${i}" class="metric-value">—</span><span class="metric-unit">${unit}</span></div>` +
      `<div class="metric-sigma" id="sigma${i}">σ —</div>`;
    if (cards) cards.appendChild(el);
  });
  diagnosticFields.forEach(([name], i) => {
    const el = document.createElement('div');
    el.className = 'diag-item';
    el.innerHTML = `<span class="diag-label" id="dl${i}">${name}</span><span class="diag-value" id="d${i}">—</span>`;
    if (diagBox) diagBox.appendChild(el);
  });

  function fmt(n, digits=3) { return Number.isFinite(n) ? n.toFixed(digits) : '—'; }
  function hexByte(value) { return Number.isInteger(value) ? `0x${value.toString(16).padStart(2,'0')}` : '不可用'; }
  function updateStats() {
    setText('rxStats', `接收遥测样本 ${networkTelemetryCount}`);
    setDisabled('export', csvRows.length === 0);
  }
  function saveCsvRow(row) {
    csvRows.push(row);
    if (csvRows.length > 50000) csvRows.shift();
    updateStats();
  }
  function ageLabel(value) {
    return Number.isFinite(value) ? `${Math.round(value)} ms` : '年龄未知';
  }
  function freshNetworkItem(stream, now = performance.now()) {
    return networkAdapter.latest(stream, NETWORK_STREAM_MAX_AGE[stream], now);
  }
  function networkAge(item, now = performance.now()) {
    return item ? Math.max(0, now - item.receivedAt) : null;
  }
  function showNetworkStream(stream, label, now) {
    const item = freshNetworkItem(stream, now);
    const chip = `stream${stream[0].toUpperCase()}${stream.slice(1)}`;
    if (!item) {
      const had = networkAdapter.streams.has(stream);
      setChip(chip, `${label}：${had ? '过期' : '不可用'}`, had ? 'stale' : 'unknown');
      return null;
    }
    setChip(chip, `${label}：新鲜 ${(networkAge(item, now)/1000).toFixed(2)} s`, 'fresh');
    return item;
  }
  function resetNetworkTelemetry() {
    workbench.reset();
    networkStopLocalRequested = null;
    clearOverviewMetrics();
    updateNetworkSnapshot();
  }
  function onNetworkConnection(connected, message) {
    setChip('networkConnection', message, connected ? 'fresh' : 'stale');
    setDisabled('networkConnect', connected);
    setDisabled('networkDisconnect', !connected);
    setText('networkEvent', message);
    if (!connected) {
      clearManualInput();
      updateNetworkControls();
    }
    updateNetworkTargetControls();
    updateWorkbench();
  }
  function onNetworkHello(hello) {
    networkSimulation = hello.simulation === true;
    if (networkSimulation) { networkTargetEdited = false; networkTargetPending = null; }
    setDisabled('jetsonHost', networkSimulation);
    setDisabled('jetsonPort', networkSimulation);
    setChip('networkSimulation', `仿真：${hello.simulation ? '是' : '否'}`, hello.simulation ? 'stale' : '');
    updateNetworkTargetControls();
    updateNetworkControls();
    updateWorkbench();
  }
  function onNetworkState(state, receivedAt) {
    networkStateReceivedAt = receivedAt || 0;
    observeBridgeStatus(state, receivedAt);
    if (state && networkTargetPending) {
      const jetson = state.jetson || {};
      if (jetson.host === networkTargetPending.host && jetson.port === networkTargetPending.port) {
        networkTargetPending = null;
        networkTargetEdited = false;
        setText('networkEvent', '本机服务已确认新的 Jetson 目标。');
      } else if (performance.now() > networkTargetPending.deadline) {
        networkTargetPending = null;
        networkTargetEdited = true;
        setText('networkEvent', '本机服务尚未确认 Jetson 目标；请检查地址、端口和服务状态。');
      }
    }
    const hostInput = $('jetsonHost'), portInput = $('jetsonPort');
    if (state && !networkTargetEdited && !networkTargetPending) {
      const jetson = state.jetson || {};
      if (typeof jetson.host === 'string' && hostInput) hostInput.value = jetson.host;
      if (Number.isInteger(jetson.port) && portInput) portInput.value = String(jetson.port);
    }
    if (state && networkAdapter.localControlEnabled) {
      if (networkAdapter.isOwner()) networkArmPendingUntil=0;
      else if (state.armed && state.owner !== (networkAdapter.hello && networkAdapter.hello.client_id)) {
        clearManualInput();
        setText('networkEvent', '网络控制权由其他页面持有，本页未取得控制权。');
      } else if (!state.armed && networkArmPendingUntil && performance.now() > networkArmPendingUntil) {
        networkArmPendingUntil=0; releaseNetworkControl('网络服务未确认控制权');
      }
    }
    updateNetworkStateUi(state);
    updateNetworkControls();
    updateNetworkTargetControls();
    updateVisionUi(false);
  }
  function bridgeStatusAgeAtReceive(state, status) {
    const ackAge = Number.isFinite(status.fpga_ack_age_ms) ? status.fpga_ack_age_ms :
      Number.isFinite(status.fpga_ack_age) ? status.fpga_ack_age * 1000 : null;
    // 服务的 ack_age_ms 已累加该 status 在服务端的停留时间。
    return Number.isFinite(ackAge) && ackAge >= 0 && Number.isFinite(state.fpga?.ack_age_ms)
      ? Math.max(0, state.fpga.ack_age_ms - ackAge) : 0;
  }
  // state 还会因其他遥测而更新，只有网桥 status 序号前进才能证明状态自身在更新。
  function observeBridgeStatus(state, receivedAt) {
    const status = state?.status;
    const sequence = status?.seq;
    if (!state?.jetson?.online || !Number.isFinite(receivedAt) ||
        typeof status?.bridge_id !== 'string' || !status.bridge_id ||
        !Number.isInteger(sequence) || sequence < 0 || sequence > 0xffffffff) {
      observedBridgeStatus = null;
      return;
    }
    const previous = observedBridgeStatus;
    const sameBridge = previous && previous.bridgeId === status.bridge_id && previous.session === status.session;
    const delta = sameBridge ? (sequence - previous.sequence) >>> 0 : 0;
    if (!sameBridge || (delta >= 0x80000000)) {
      observedBridgeStatus = {bridgeId:status.bridge_id, session:status.session, sequence,
        receivedAt, ageMs:bridgeStatusAgeAtReceive(state,status), confirmed:false, status};
    } else if (delta > 0) {
      observedBridgeStatus = {bridgeId:status.bridge_id, session:status.session, sequence,
        receivedAt, ageMs:bridgeStatusAgeAtReceive(state,status), confirmed:true, status};
    } else {
      previous.ageMs = Math.max(previous.ageMs + Math.max(0,receivedAt - previous.receivedAt),
        bridgeStatusAgeAtReceive(state,previous.status));
      previous.receivedAt = receivedAt;
    }
  }
  function freshBridgeStatus() {
    const now = performance.now();
    const state = networkAdapter.state;
    return networkAdapter.connected && state?.jetson?.online === true && networkStateReceivedAt &&
      now - networkStateReceivedAt <= 500 && observedBridgeStatus?.confirmed &&
      observedBridgeStatus.ageMs + Math.max(0,now - observedBridgeStatus.receivedAt) <= 500
      ? observedBridgeStatus.status : null;
  }
  function bridgeEmergencyStop() {
    const status = freshBridgeStatus();
    if (!status) return null;
    if (status.emergency_stop === true || status.control_source === 'emergency_stop') return true;
    if (status.emergency_stop === false) return false;
    // 仅模拟器将控制源与急停绑定；真实网桥的控制源不能反推急停未激活。
    if (networkAdapter.hello?.simulation === true && ['none','pc','ros'].includes(status.control_source)) return false;
    return null;
  }
  function motionStartAllowed() {
    return bridgeEmergencyStop() === false;
  }
  function motionStartBlockReason() {
    const status = freshBridgeStatus();
    if (!status) return '网桥状态未知或已过期，不能启动。';
    if (status.emergency_stop === true) return '软件急停已激活，不能启动。';
    if (status.control_source === 'emergency_stop')
      return '网桥急停反馈不一致，状态未知，不能启动。';
    if (bridgeEmergencyStop() === null) return '网桥急停状态未知，不能启动。';
    return null;
  }
  function currentVision() {
    const state = networkAdapter.state;
    const elapsed = networkStateReceivedAt ? performance.now() - networkStateReceivedAt : Infinity;
    if (!state?.vision || elapsed > 500 || !state.jetson?.online) return null;
    const age_ms = Number(state.vision.age_ms) + Math.max(0, elapsed);
    return {...state.vision, age_ms, stale:state.vision.stale || !Number.isFinite(age_ms) || age_ms > 500};
  }
  function visionCanStart(vision) {
    return !!(vision && !vision.stale && ['IDLE','STOPPED'].includes(vision.mission_state));
  }
  function visionFrameReady(vision) {
    const image = $('networkVideo');
    const now = performance.now();
    return !!(vision && !vision.stale && image && !image.hidden &&
      networkVideoWidth === vision.width && networkVideoHeight === vision.height &&
      networkVideoSocket && networkVideoLastAt && now-networkVideoLastAt <= 1000 &&
      networkVideoDecodedAt && now-networkVideoDecodedAt <= 1000);
  }
  function visionDraftPresent(vision) {
    return !!(visionFrameReady(vision) && visionDraft &&
      (vision.detections || []).some(box => box.id === visionDraft.id && box.class_id === visionDraft.class_id));
  }
  function visionFrameProblem(vision) {
    if (!vision || vision.stale) return null;
    const image = $('networkVideo');
    if (!image || image.hidden || !networkVideoSocket || !networkVideoLastAt || !networkVideoDecodedAt)
      return '视频画面缺失';
    const now = performance.now();
    if (now-networkVideoLastAt > 1000 || now-networkVideoDecodedAt > 1000)
      return '视频画面已过期';
    if (networkVideoWidth !== vision.width || networkVideoHeight !== vision.height)
      return '视频与视觉尺寸不一致';
    return null;
  }
  function updateVisionUi(draw = true) {
    const vision = currentVision();
    const pending = performance.now() < visionPendingUntil;
    const states = {UNKNOWN:'未知', IDLE:'空闲', TRACKING:'跟踪中', SEARCHING:'搜索中', FOUND:'已发现（等待交接）', FOLLOWING:'跟随中', APPROACHING:'接近中', ARRIVED:'已到达', BLOCKED:'前方间距不足', LOST:'目标丢失', STOPPED:'已停止'};
    const stop = bridgeEmergencyStop();
    const stopFeedbackConflict = stop === true && freshBridgeStatus()?.emergency_stop !== true;
    const emergency = stopFeedbackConflict ? '软件急停：未知（网桥反馈不一致，启动受限）' :
      stop === null ? '软件急停：未知（等待新鲜网桥状态）' :
      `软件急停：${stop ? '已激活' : '未激活'}`;
    const mission = !vision ? (networkAdapter.state?.vision ? '视觉状态已过期' : '视觉任务接口未连接') : vision.stale ? '视觉观测已过期' :
      `任务：${states[vision.mission_state] || '未知'} · 控制源：${vision.control_source} · ROS ${vision.ros_authorized?'已授权':'未授权'}`;
    const taskStartNotice = vision && !vision.stale && !visionCanStart(vision)
      ? ' · 启动受限：请先确认任务为空闲或已停止' : '';
    setText('visionStatus', `${mission} · ${emergency}${taskStartNotice}`);
    const target = vision && !vision.stale ? vision.selected_target : null;
    const frameProblem = visionFrameProblem(vision);
    setText('visionTarget', `检测叠加（非逐帧同步）· ${vision ? `观测年龄 ${vision.age_ms.toFixed(0)} ms${vision.stale?'（已过期）':''}`:'无观测'} · ${target ? `锁定 ${target.id}`:'无有效锁定目标'}${vision?.truncated?' · 目标列表已截断':''}${frameProblem ? ` · ${frameProblem}，不能选点` : ''}`);
    const detections = vision && !vision.stale ? vision.detections || [] : [];
    const present = visionDraftPresent(vision);
    setText('visionDraft', visionDraft
      ? `已选择 ${visionDraft.id}${present ? '' : '（当前检测中不可用）'}`
      : '尚未选择目标');
    const selection = $('visionDraftSelect');
    const optionsKey = JSON.stringify(detections.map(box => [box.id,box.class_id]));
    if (selection && optionsKey !== visionOptionsKey && typeof selection.replaceChildren === 'function') {
      const placeholder = document.createElement('option');
      placeholder.value = ''; placeholder.textContent = '选择检测目标';
      const options = detections.map((box, index) => {
        const option = document.createElement('option');
        option.value = String(index + 1); option.textContent = `${box.id} · 类别 ${box.class_id}`;
        return option;
      });
      selection.replaceChildren(placeholder, ...options); visionOptionsKey = optionsKey;
    }
    if (selection) {
      const index = visionDraft ? detections.findIndex(box => box.id === visionDraft.id && box.class_id === visionDraft.class_id) : -1;
      selection.value = index < 0 ? '' : String(index + 1);
      selection.disabled = !visionFrameReady(vision) || !detections.length || ['TRACKING','FOLLOWING','APPROACHING','SEARCHING','FOUND','BLOCKED','LOST'].includes(vision?.mission_state);
    }
    setDisabled('visionEnable', !vision || !motionStartAllowed() || vision.ros_authorized || pending);
    setDisabled('visionDisable', !vision || !vision.ros_authorized);
    setDisabled('visionStart', !visionCanStart(vision) || !motionStartAllowed() || pending ||
      (!isSearchBehavior($('visionBehavior').value) && !present));
    setDisabled('visionStop', !networkAdapter.connected);
    setDisabled('visionEstop', !networkAdapter.connected || networkAdapter.state?.jetson?.online !== true ||
      !networkStateReceivedAt || performance.now() - networkStateReceivedAt > 500);
    setDisabled('visionRelease', stop !== true || pending);
    if (draw) drawVisionOverlay(vision);
  }
  function onVisionResult(result) {
    visionPendingUntil = 0;
    setText('visionResult', `${result.unknown?'结果未知':result.success?'请求已响应':'请求未成功'}：${result.message}；实际任务状态以反馈为准。`);
    updateVisionUi();
  }
  function onDashboardMissionResult(result) {
    if (typeof result.action === 'string' && result.action.startsWith('navigation_')) {
      navigationPendingUntil = 0;
      if (result.action === 'navigation_start') {
        if (result.success && !result.unknown && navigationDraftSubmitted === waypointDraftKey()) clearWaypoints();
        navigationDraftSubmitted = null;
      }
      setText('localTaskResult', `${result.unknown ? '结果未知' : result.success ? '请求已响应' : '请求未成功'}：${result.message}；执行状态以任务反馈为准。`);
      updateLocalTaskUi();
      return;
    }
    if (typeof result.action === 'string' && result.action.startsWith('local_')) {
      const transaction = Number.isInteger(result.transaction_task_id) ? ` · 事务 task ${result.transaction_task_id}` : '';
      const activeTask = Number.isInteger(result.active_task_id) ? ` · 当前 task ${result.active_task_id}` : '';
      const cancelledTask = Number.isInteger(result.cancelled_task_id) && result.cancelled_task_id !== 0
        ? ` · 等待原任务 ${result.cancelled_task_id} 的取消状态` : '';
      setText('localTaskResult', `${result.unknown ? '结果未知' : result.success ? '请求已响应' : '请求未成功'}：${result.message}${transaction}${activeTask}${cancelledTask}；分片 ACK 只代表接收，不代表执行完成。`);
      updateLocalTaskUi();
      return;
    }
    onVisionResult(result);
  }

  function currentNavigation() {
    const state = networkAdapter.state;
    const navigation = state && state.navigation;
    if (!navigation || !networkStateReceivedAt || performance.now() - networkStateReceivedAt > 500 ||
        navigation.stale === true || !Number.isFinite(navigation.age_ms)) return null;
    const age_ms = navigation.age_ms + Math.max(0, performance.now() - networkStateReceivedAt);
    return {...navigation, age_ms, stale:age_ms > 500 || state.jetson?.online !== true};
  }
  function navigationMatchesMap(nav, info) {
    return !!(nav && !nav.stale && info.fresh && info.domain === `${nav.boot_id}:${nav.odom_domain}`);
  }
  function selectMapGoal(point) {
    if (!mapGoals.length) {
      mapGoalFrame = point.map ? 'map' : 'odom';
      mapGoalDomain = point.domain;
      mapGoalGeneration = point.map?.generation ?? null;
    }
    const position = point[mapGoalFrame];
    if (!position || point.domain !== mapGoalDomain || (mapGoalFrame === 'map' && position.generation !== mapGoalGeneration)) {
      mapGoalNotice = '等待同一坐标域和地图代次的坐标变换后再选点。';
      updateLocalTaskUi();
      return;
    }
    mapGoalNotice = '';
    const goal = {x:Number(position.x.toFixed(3)), y:Number(position.y.toFixed(3))};
    const index = mapGoals.findIndex(item => Math.hypot(item.x-goal.x, item.y-goal.y) < 0.15);
    if (index >= 0) mapGoals.splice(index,1); else mapGoals.push(goal);
    if (!mapGoals.length) clearWaypoints();
    updateLocalTaskUi();
  }
  function waypointDraftKey() {
    return JSON.stringify({frame:mapGoalFrame,domain:mapGoalDomain,generation:mapGoalGeneration,goals:mapGoals});
  }
  function clearWaypoints() {
    mapGoals = [];
    mapGoalFrame = mapGoalDomain = mapGoalGeneration = null;
    mapGoalNotice = '';
  }
  function invalidateGlobalGoals() {
    if (!mapGoals.length || mapGoalFrame !== 'map') return;
    clearWaypoints();
    mapGoalNotice = '全局地图坐标域或代次变化，请重新选点。';
    updateLocalTaskUi();
  }
  function updateLocalTaskUi() {
    const nav = currentNavigation();
    const state = networkAdapter.state || {};
    const task = state.navigation_task;
    const localMap = occupancyView.getInfo();
    const display = mapGoals.length
      ? {frame:mapGoalFrame,domain:mapGoalDomain,generation:mapGoalGeneration,goals:mapGoals,route:[]}
      : task?.id ? task : null;
    occupancyView.setNavigation(display);
    const waypointList = $('waypointList');
    if (waypointList) {
      const goals = display?.goals || [];
      waypointList.hidden = !goals.length;
      waypointList.replaceChildren(...goals.map((goal,index) => {
        const item = document.createElement('li');
        item.textContent = `${display.frame} x ${goal.x.toFixed(3)} m · y ${goal.y.toFixed(3)} m${display === task && task?.active && index === task.goal_index ? ' · 当前' : ''}`;
        return item;
      }));
    }
    const domainNotice = mapGoals.length && nav && !nav.stale && `${nav.boot_id}:${nav.odom_domain}` !== mapGoalDomain
      ? ' · 当前导航坐标域不同，由 Jetson 验证' : '';
    setText('mapGoalHint', mapGoals.length
      ? `已选 ${mapGoals.length} 个航点 · ${mapGoalFrame} x ${mapGoals[0].x.toFixed(3)} m · y ${mapGoals[0].y.toFixed(3)} m · 域 ${mapGoalDomain}${domainNotice}${mapGoalNotice ? ` · ${mapGoalNotice}` : ''}`
      : task?.active ? `已提交 ${task.goal_count} 个航点` : mapGoalNotice);
    const active = !!(nav && (typeof nav.active_task === 'boolean' ? nav.active_task :
      ((nav.status >= 1 && nav.status <= 3) || nav.status === 5) && [2,3].includes(nav.mode)));
    const taskWatermarkKnown = nav?.task_id_watermark_known === true && Number.isInteger(nav.last_task_id);
    const polarNavigation = nav?.protocol === 'nav_polar_v2';
    const jetsonNavigation = nav?.protocol === 'jetson_nav_v1';
    const gridNavigation = nav?.protocol === 'nav_grid_v1' || jetsonNavigation;
    const fullNavigation = nav?.protocol === 'nav_full_v1' || polarNavigation || gridNavigation;
    const statusNames = fullNavigation
      ? ['空闲','接收任务中','规划中','执行中','已到达','受阻（等待重试）','任务被拒绝','已取消','观测或任务失效','等待执行']
      : ['空闲','已接收','规划中','执行中','已到达','受阻（等待重试）','已过期','状态过期','数值无效','已取消','里程域已重置','任务容量不足','链路丢失'];
    const modeNames = fullNavigation ? ['待机','手动','协同','自主'] : ['手动','视觉','协同','独立'];
    const baseStatus = polarNavigation && nav.status === 2
      ? (Math.abs(nav.wz_mrad_s || 0) > 0 ? '转向观测中' : '规划中 / 等待新扫描')
      : nav ? (statusNames[nav.status] || `未知状态 ${nav.status}`) : '等待新鲜导航状态';
    const fusion = freshNetworkItem('fusion')?.data;
    const gridWait = nav?.protocol === 'nav_grid_v1' && nav.status === 2 && fusion?.architecture === 'local_grid_v1' &&
      Math.abs(((nav.time_us - fusion.time_us) | 0)) <= 500000
      ? fusion.start_confirmation_lost ? '启动区许可失效，等待新任务与地图恢复'
        : fusion.map_fresh === false ? `等待新鲜局部地图${fusion.map_gap_latched ? '（' + gridGapReason(fusion.map_gap_reason_name) + '）' : ''}`
          : ''
      : '';
    const status = nav?.rx_rejection ? '收到新任务接收拒绝' : gridWait || baseStatus;
    const guardReasons = ['CPU','位姿','IMU','轮速','雷达','输出过期','输出范围',
      '命令序号','任务FIFO溢出','输入缺口','滚动屏障','命令链路','软件停车','近障'];
    const errorText = nav?.rx_rejection ? '' : nav?.guard_error
      ? ` · 保护原因 ${guardReasons[nav.guard_error - 1] || '未知'}（${nav.guard_error}）`
      : nav?.planner_error ? ` · 规划器错误原因码 ${nav.planner_error}` : '';
    const rawError = Number.isInteger(nav?.error) && nav.error !== 0
      ? ` · 原始错误 ${nav.error}（0x${nav.error.toString(16)}）` : '';
    const idleStop = nav?.guard_error === 13 && !active && nav.mode === 1 &&
      freshBridgeStatus()?.control_source === 'none'
      ? ' · 当前无活动导航任务或控制源' : '';
    const remainingUs = nav ? (fullNavigation
      ? Math.max(0, (Number(nav.expiry_us) - Number(nav.now_us)) | 0)
      : Math.max(0, nav.expiry_us - nav.now_us)) : 0;
    const deadlineKnown = nav?.deadline_known ?? !!nav?.expiry_us;
    const deadline = deadlineKnown ? `剩余 ${(remainingUs / 1e6).toFixed(2)} s` : '无活动任务期限';
    const details = nav ? `状态 ${status} · 模式 ${modeNames[nav.mode] || nav.mode} · task ${nav.task_id} · boot ${nav.boot_id} · odom ${nav.odom_domain} · ${deadline}${errorText}${rawError}${idleStop}` : status;
    const names = {planning:'规划中',following:'执行中',blocked:'等待处理',
      arrived:'全部到达',cancelled:'已取消',failed:'任务失败'};
    const progress = task?.goal_count ? ` · 航点 ${Math.min(task.goal_index + 1,task.goal_count)}/${task.goal_count}` : '';
    const summary = task ? `${names[task.status] || task.status}${progress}${task.message ? ` · ${task.message}` : ''} · ${details}` : details;
    setText('localTaskState', summary + (nav?.stale ? ' · 已过期' : ''));
    setText('overviewTaskState', summary + (nav?.stale ? ' · 已过期' : ''));
    const searchCount = gridNavigation ? `已评分候选 ${nav.candidates_evaluated ?? '—'} · 地图版本 ${nav.map_version ?? '—'}`
      : polarNavigation ? `极坐标候选 ${nav.polar_candidates_evaluated ?? '—'} · 扫描代次 ${nav.polar_generation ?? '—'}`
      : fullNavigation ? `A*扩展 ${nav?.planner_expansions ?? '—'}` : `候选 ${nav?.candidate_count ?? '—'}`;
    const observationAge = gridNavigation && nav.map_age_us === 0xffffffff ? '尚无完整圈'
      : polarNavigation ? `扫描龄 ${nav.polar_age_us ?? '—'} µs` : `地图龄 ${nav?.map_age_us ?? '—'} µs`;
    const mapDetail = jetsonNavigation
      ? (navigationMatchesMap(nav,localMap) ? 'Jetson 局部栅格就绪' : '等待同坐标域的 Jetson 局部栅格')
      : `${searchCount} · ${observationAge}`;
    setText('localTaskPose', nav ? `局部目标 (${(nav.goal_x_q16 / 65536).toFixed(3)}, ${(nav.goal_y_q16 / 65536).toFixed(3)}) m · ${mapDetail} · 位姿龄 ${nav.pose_age_us ?? '—'} µs` : '');
    const pending = performance.now() < navigationPendingUntil;
    const connected = !!(networkAdapter.connected && networkAdapter.hello);
    setDisabled('localTaskStart', !connected || pending || !mapGoals.length);
    // 旧遥测可能落后于任务开始回执，停止和取消始终可请求。
    setDisabled('localTaskStop', !connected);
    setDisabled('localTaskCancel', !connected);
    setText('localTaskFreshness', nav && !nav.stale
      ? taskWatermarkKnown
        ? `导航时钟与事务水位新鲜 · ${nav.age_ms.toFixed(0)} ms · last task ${nav.last_task_id}`
        : '导航状态缺少任务水位，等待 Jetson 确认执行器状态'
      : '导航状态不可用或已过期');
    if (nav?.rx_rejection === true || nav?.rejected === true) {
      const rejectedId = Number.isInteger(nav.rejected_task_id) ? nav.rejected_task_id : nav.task_id;
      const reason = Number.isInteger(nav.rejection_reason) ? nav.rejection_reason : Number.isInteger(nav.rx_rejection_error) ? nav.rx_rejection_error & 0x7f : 0;
      const key = `${nav.sequence}:${rejectedId}:${reason}`;
      if (key !== lastNavigationRejectionKey) {
        lastNavigationRejectionKey = key;
        setText('localTaskResult', `导航拒绝接收任务 ${rejectedId}，原因码 ${reason}；当前任务 ${nav.task_id}：${statusNames[nav.status] || status}。`);
      }
    }
    const freshness = $('localTaskFreshness');
    if (freshness?.dataset) freshness.dataset.state = nav && !nav.stale ? 'fresh' : 'stale';
  }
  function requestNavigationTask(action) {
    if (action === 'start' && !mapGoals.length) {
      setText('localTaskResult','请先在地图上选点。');
      return;
    }
    const number = id => {
      const value = $(id)?.value;
      return typeof value === 'string' && value.trim() ? Number(value) : NaN;
    };
    const fields = action === 'start' ? {frame:mapGoalFrame,domain:mapGoalDomain,generation:mapGoalGeneration,
      goals:mapGoals.map(({x,y}) => ({x,y})),
      max_speed_mm_s:number('localMaxSpeed'),max_accel_mm_s2:number('localMaxAccel'),
      tolerance_mm:number('localTolerance'),yaw_tolerance_mrad:number('localYawTolerance'),
      duration_s:number('localDuration')} : {};
    if (action === 'start') clearManualInput();
    if (!networkAdapter.navigationTask(action,fields)) {
      setText('localTaskResult','请求未发送：本机网络服务未连接。');
      return;
    }
    if (action === 'start') navigationDraftSubmitted = waypointDraftKey();
    navigationPendingUntil = performance.now() + 3000;
    setText('localTaskResult',action === 'start' ? '已发送航点队列，等待服务确认。' : '已请求结束任务，等待状态回报。');
    updateLocalTaskUi();
  }
  function requestVisionAction(action) {
    if (action === 'release_emergency_stop' && bridgeEmergencyStop() !== true) {
      setText('visionResult','网桥急停状态未确认激活，未发送解除请求。');
      return;
    }
    if ((action === 'start' || action === 'enable_ros') && !motionStartAllowed()) {
      setText('visionResult', motionStartBlockReason());
      return;
    }
    const vision = currentVision();
    if (action === 'start' && (!vision || vision.stale)) {
      setText('visionResult', vision ? '视觉观测已过期，不能启动。' : '视觉状态不可用，不能启动。');
      return;
    }
    if (action === 'start' && !visionCanStart(vision)) {
      setText('visionResult', '视觉任务尚未确认空闲或停止，请先核实状态并显式停止原任务。');
      return;
    }
    if (action === 'start' && performance.now() < visionPendingUntil) {
      setText('visionResult', '上一项视觉请求仍在等待结果，未再次发送启动请求。');
      return;
    }
    const behavior = $('visionBehavior').value;
    const search = isSearchBehavior(behavior);
    if (action === 'start') {
      if (!search && !visionDraftPresent(vision)) {
        const frameProblem = visionFrameProblem(vision);
        setText('visionResult', frameProblem ? `${frameProblem}，请等待画面后选择目标。` :
          visionDraft ? '所选目标不在当前检测中，请重新选择。' : '请先选择检测目标。');
        return;
      }
    }
    if ((action === 'enable_ros' || action === 'start') &&
        networkAdapter.localControlEnabled) {
      releaseNetworkControl('切换到 ROS 视觉转向');
    }
    if (action === 'enable_ros' || action === 'start') clearWaypoints();
    const fields = action === 'start' ? {target_id:search ? '' : visionDraft.id,
      class_id:search ? $('visionClass').value : visionDraft.class_id, behavior,color:$('visionColor').value} : {};
    if (!networkAdapter.missionCommand(action, fields)) {
      setText('visionResult', '请求未发送：本机网络服务未连接');
      return;
    }
    visionPendingUntil = performance.now() + 3000;
    setText('visionResult', '请求已发出，等待网桥或任务服务响应；不代表机械动作完成。');
    updateVisionUi();
  }
  function isSearchBehavior(behavior) {
    return behavior === 'search' || behavior === 'search_only';
  }
  function drawVisionOverlay(vision = currentVision()) {
    const canvas = $('visionOverlay'), image = $('networkVideo');
    if (!canvas || !image) return;
    const bounds = canvas.getBoundingClientRect();
    const dpr = window.devicePixelRatio || 1;
    const width = Math.round(bounds.width*dpr), height = Math.round(bounds.height*dpr);
    if (canvas.width !== width || canvas.height !== height) { canvas.width=width; canvas.height=height; }
    const ctx = canvas.getContext('2d');
    if (!ctx) return;
    ctx.setTransform(dpr,0,0,dpr,0,0);
    ctx.clearRect(0,0,bounds.width,bounds.height);
    if (!visionFrameReady(vision)) return;
    const boxes = [...(vision.detections || [])];
    if (vision.selected_target && !boxes.some(box => box.id === vision.selected_target.id)) boxes.push(vision.selected_target);
    ctx.font='12px sans-serif'; ctx.lineWidth=2;
    for (const box of boxes) {
      const rect = networkApi.visionRect(bounds.width,bounds.height,vision.width,vision.height,box);
      if (!rect) continue;
      ctx.strokeStyle=box.id === vision.selected_target?.id ? '#34d399' :
        box.id === visionDraft?.id && box.class_id === visionDraft?.class_id ? '#fbbf24' : '#67e8f9';
      ctx.fillStyle=ctx.strokeStyle;
      ctx.strokeRect(rect.x,rect.y,rect.width,rect.height);
      ctx.fillText(`${box.id.slice(-24)} ${(box.score*100).toFixed(0)}%`,Math.max(0,rect.x),Math.max(14,rect.y-4));
    }
  }
  function selectVisionDraft(box) {
    visionDraft = box ? {id:box.id, class_id:box.class_id} : null;
    updateVisionUi();
  }
  function selectVisionAt(event) {
    const vision = currentVision(), canvas = $('visionOverlay'), image = $('networkVideo');
    if (!visionFrameReady(vision) || ['TRACKING','FOLLOWING','APPROACHING','SEARCHING','FOUND','BLOCKED','LOST'].includes(vision.mission_state) || !canvas || !image) return;
    const bounds=canvas.getBoundingClientRect(), x=event.clientX-bounds.left, y=event.clientY-bounds.top;
    const matches=(vision.detections || []).map(box => ({box,rect:networkApi.visionRect(bounds.width,bounds.height,vision.width,vision.height,box)}))
      .filter(item => item.rect && x >= item.rect.x && y >= item.rect.y &&
        x <= item.rect.x+item.rect.width && y <= item.rect.y+item.rect.height)
      .sort((a,b) => a.rect.width*a.rect.height-b.rect.width*b.rect.height);
    if (matches.length) selectVisionDraft(matches[0].box);
  }
  function onNetworkEvent(message) {
    const eventLabels = {
      websocket_error:'WebSocket 链路错误', state_timeout:'网络状态超时',
      udp_error:'UDP 套接字错误', udp_send_error:'UDP 发送失败',
      target_changed:'Jetson 目标已更改', configured:'Jetson 目标配置完成',
      configure_rejected:'Jetson 目标配置被拒绝', arm_rejected:'控制授权被拒绝',
      armed:'本页已取得控制权', drive_command:'手动指令已接受',
      auto_requested:'已停用的无目标自动请求', auto_rejected:'无目标自动模式已停用',
      auto_disabled:'旧自动模式已关闭', legacy_auto_rejected:'已停用的无目标自动请求',
      stop_requested:'停车请求并解除控制',
      control_sent:'控制报文状态已发送', zero_handshake:'零速握手已发送',
      bridge_changed:'Jetson 网关进程已更换', bridge_epoch_changed:'遥测 epoch 已更换',
      message_rejected:'网络消息被拒绝', video_stale:'视频流已中断',
      jetson_offline:'Jetson UDP 链路离线'
    };
    const reasonLabels = {
      'release-control-before-changing-target':'更换目标前须先停车并解除控制',
      'target-or-owner-changed-during-configuration':'配置期间目标或控制者发生变化',
      'simulation-target-locked-to-startup-endpoint':'仿真目标锁定为服务启动地址',
      'target-unchanged':'目标未变化', 'target-changed':'目标已更改',
      'another-page-owns-control':'其他页面持有控制权',
      'page-consumption-unconfirmed':'页面尚未确认接收状态，操作未执行',
      'jetson-or-fpga-not-ready':'Jetson 或 FPGA 尚未就绪',
      'user-requested-control':'用户主动允许控制', 'accepted-drive':'已接受驾驶指令',
      enabled:'已请求开启', disabled:'已请求关闭',
      'nonzero-drive-during-auto':'自动请求期间收到非零手动指令',
      'auto-needs-fresh-zero-drive':'自动模式需要新鲜零速心跳',
      'legacy-auto-disabled-zero':'旧自动模式已停用并发送零速',
      'page-command-expired':'页面控制心跳超时',
      'controller-disconnected':'控制页面已断开',
      'jetson-udp-timeout':'Jetson UDP 状态超时', 'udp-timeout':'Jetson UDP 状态超时',
      'fpga-ack-stale':'FPGA 回执过期', 'server-closed':'本机服务已关闭',
      'control-state-changed':'控制状态发生变化',
      'initial-zero-handshake':'启动时发送零速握手',
      'target-change-new-zero':'新目标已发送零速握手',
      'session-rotation-zero':'会话轮换时发送零速握手',
      'video-silent-for-one-second':'超过 1 秒未收到完整视频帧',
      'new-bridge-id':'检测到新的网关进程 ID',
      'sequence-and-time-reset-after-silence':'静默后序号与时间同时回退',
      'jetson-epoch-changed':'Jetson 遥测 epoch 已更新',
      'browser-binary-message':'控制通道不接受二进制消息',
      'browser-json-invalid':'控制消息不是有效 JSON',
      'browser-message-invalid':'控制消息字段无效',
      'browser-message-unknown':'控制消息类型未知',
      'drive-without-control':'当前页面没有控制权',
      'drive-fields-invalid':'驾驶指令字段无效',
      'drive-over-hard-limit':'驾驶指令超出硬件限制',
      'auto-without-control':'当前页面没有控制权',
      'auto-value-invalid':'自动模式参数无效',
      'udp-source-or-size-invalid':'UDP 来源或报文长度无效',
      'udp-json-invalid':'UDP 报文不是有效 JSON',
      'udp-envelope-invalid':'UDP envelope 字段无效',
      'telemetry-envelope-invalid':'遥测 envelope 字段无效',
      'telemetry-part-invalid':'遥测分片字段无效',
      'telemetry-part-count-conflict':'遥测分片数量冲突',
      'telemetry-part-conflict':'遥测分片内容冲突'
    };
    const event = typeof message.event === 'string' ? message.event : '';
    const reason = typeof message.reason === 'string' ? message.reason : '';
    const eventText = eventLabels[event] || (event ? `事件码：${event}` : '网络事件');
    const reasonText = reasonLabels[reason] || (reason ? (/[\u3400-\u9fff]/.test(reason) ? reason : `原因码：${reason}`) : '');
    const detail = [eventText, reasonText].filter(Boolean).join(' · ');
    if (detail) setText('networkEvent', detail);
    if (message.event === 'arm_rejected') {
      clearManualInput();
      updateNetworkControls();
    }
    if (message.event === 'state_timeout') {
      networkStopLocalRequested = {sent:true,at:performance.now(),reason:message.reason || '状态超时'};
      clearManualInput();
      updateNetworkControls();
    }
  }
  function onNetworkTelemetry(stream, item) {
    if (stream === 'navigation' || stream === 'fusion') updateLocalTaskUi();
    saveCsvRow({source:'udp',stream,envelope:item.envelope,receivedAt:new Date().toISOString()});
    if (stream === 'health') updateVideoMetrics();
  }
  function updateNetworkStateUi(state) {
    if (!state) {
      setChip('networkTarget','Jetson：不可用','unknown');
      setChip('networkOwner','控制者：未知','unknown');
      setChip('networkReady','FPGA 零速握手：未知','unknown');
      setChip('networkActualMode','实际模式：不可用','unknown');
      setChip('wbModeRequest','无目标type5自动模式：已停用','stale');
      setChip('wbModeAck','局部导航状态：等待遥测','unknown');
      updateNetworkStopEvidence(null);
      return;
    }
    const stateAge = networkStateReceivedAt ? Math.max(0, performance.now() - networkStateReceivedAt) : Infinity;
    const stale = stateAge > 500;
    const jetson = state.jetson || {};
    const fpga = state.fpga || {};
    const status = freshNetworkStatus() || {};
    setChip('networkTarget',`Jetson：${jetson.host || '—'}:${jetson.port || '—'} · ${jetson.online ? '在线' : '离线'} · ${ageLabel(jetson.age_ms)}`,stale || !jetson.online ? 'stale' : 'fresh');
    setChip('networkOwner',`网页控制者：${state.owner || '无'}${state.armed ? ' · 已授权' : ''}`,stale ? 'stale' : state.armed ? 'fresh' : '');
    setChip('networkReady',`FPGA 零速握手：${fpga.ready ? '已回执' : '未就绪'} · ${ageLabel(fpga.ack_age_ms)}`,stale ? 'stale' : fpga.ready ? 'fresh' : 'pending');
    const mode = status.actual_mode === 'auto' ? '自动位' : status.actual_mode === 'manual' ? '手动位' : '不可用';
    setChip('networkActualMode','FPGA融合状态位：' + mode,stale || !status.actual_mode ? 'stale' : 'fresh');
    setChip('wbModeRequest','无目标type5自动模式：已停用','stale');
    const nav = currentNavigation();
    const navNames = ['空闲','已接收','规划中','执行中','已到达','受阻等待重试','已过期','状态过期','数值无效','已取消','里程域已重置','任务容量不足','链路丢失'];
    const navFresh = !!(nav && nav.stale !== true);
    setChip('wbModeAck',navFresh ? '局部导航状态：' + (navNames[nav.status] || ('未知 ' + nav.status)) : '局部导航状态：等待遥测',
      navFresh ? (nav.status === 5 ? 'pending' : 'fresh') : 'stale');
    setChip('networkSimulation',`仿真：${state.simulation ? '是' : '否'}`,state.simulation ? 'stale' : '');
    updateNetworkStopEvidence(state);
  }
  function updateNetworkStopEvidence(state) {
    const age = state && networkStateReceivedAt ? Math.max(0,performance.now()-networkStateReceivedAt) : Infinity;
    const fresh = age <= 500;
    const stop = state && state.stop || {};
    const serviceRequested = fresh && typeof stop.requested === 'boolean' ? stop.requested : null;
    const local = networkStopLocalRequested && networkStopLocalRequested.sent
      ? '本页已发送' : '本页未发送';
    const service = serviceRequested === true ? '服务报告已请求' :
      serviceRequested === false ? '服务报告无待处理请求' : '服务报告不可用或过期';
    setChip('networkStopRequest',`停车请求：${local} · ${service}`,fresh ? (serviceRequested ? 'fresh' : '') : 'stale');

    const acknowledged = fresh && typeof stop.fpga_acknowledged === 'boolean' ? stop.fpga_acknowledged : null;
    const sequence = Number.isInteger(stop.pc_seq) ? ` · PC 序号 ${stop.pc_seq}` : '';
    const fullSession = typeof stop.session === 'string' ? stop.session : '';
    const shortSession = fullSession ? ` · 会话 ${fullSession.slice(0,8)}${fullSession.length > 8 ? '…' : ''}` : '';
    const ackText = acknowledged === true ? 'FPGA 已回执零速入队' :
      acknowledged === false ? 'FPGA 零速回执待确认' : '未提供或已过期';
    const ackChip = setChip('networkStopAck',`FPGA 零速回执：${ackText}${sequence}${shortSession}`,acknowledged === true ? 'fresh' : 'stale');
    if (ackChip) ackChip.title = fullSession ? `完整会话 ID：${fullSession}` : '';

    const motorAge = Number.isFinite(stop.motor_age_ms) ? stop.motor_age_ms : null;
    const threshold = Number.isFinite(stop.threshold_mm_s) ? stop.threshold_mm_s : null;
    const wheelKnown = fresh && motorAge !== null && motorAge <= 1000 && typeof stop.wheels_low === 'boolean';
    const wheelsLow = wheelKnown ? stop.wheels_low : null;
    const wheelText = wheelsLow === true ? '四轮反馈低于阈值' :
      wheelsLow === false ? '反馈仍高于阈值' : '未提供或已过期';
    const thresholdText = threshold === null ? '阈值不可用' : `阈值 ${threshold} mm/s`;
    const motorAgeText = motorAge === null ? '电机反馈年龄不可用' : `电机反馈 ${ageLabel(motorAge)}`;
    setChip('networkWheelsLow',`轮速反馈：${wheelText} · ${thresholdText} · ${motorAgeText}`,wheelsLow === true ? 'fresh' : 'stale');
  }
  function driveStateIsFresh() {
    const state = networkAdapter.state;
    if (!state || !networkStateReceivedAt || performance.now() - networkStateReceivedAt > 500) return false;
    const jetson = state.jetson || {}, fpga = state.fpga || {};
    return jetson.online === true && Number.isFinite(jetson.age_ms) && jetson.age_ms <= 1000 &&
      fpga.ready === true && Number.isFinite(fpga.ack_age_ms) && fpga.ack_age_ms <= 1000;
  }
  function networkCanArm() {
    return !!(networkAdapter.connected && networkAdapter.hello &&
      driveStateIsFresh() && !networkAdapter.state.armed);
  }
  function updateNetworkControls() {
    const ready = networkAdapter.isOwner();
    setDisabled('networkArm', !networkCanArm() || networkArmPendingUntil > performance.now());
    setText('networkArm', ready ? '网络控制已允许' : '允许网络控制');
    setDisabled('networkSpeed', !ready);
    setDisabled('networkTurn', !ready);
    document.querySelectorAll('[data-network-key]').forEach(button => {
      button.disabled = !ready;
      button.classList.toggle('active', pressedNetworkKeys.has(button.dataset.networkKey));
    });
    setText('networkDriveStatus', ready
      ? '网络控制已允许 · 手动控制'
      : '网络仅监控 · 未允许控制');
    if (currentVision()?.ros_authorized) {
      setText('networkDriveStatus', 'ROS 视觉转向已授权 · 手动控制未授权');
    }
    const driveStatus = $('networkDriveStatus');
    if (driveStatus) driveStatus.classList.toggle('ready',ready);
    setText('networkSpeedValue', `${networkSpeed} mm/s`);
    setText('networkTurnValue', `${networkTurn} mrad/s`);
    updateLocalTaskUi();
  }
  function updateNetworkTargetControls() {
    if (networkTargetPending && performance.now() > networkTargetPending.deadline) {
      networkTargetPending = null;
      networkTargetEdited = true;
      setText('networkEvent', '本机服务尚未确认 Jetson 目标；请检查地址、端口和服务状态。');
    }
    const state = networkAdapter.state;
    const connected = !!(networkAdapter.connected && state);
    setDisabled('jetsonHost', networkSimulation);
    setDisabled('jetsonPort', networkSimulation);
    setDisabled('networkApplyTarget', networkSimulation || !connected || !!(state && state.armed) ||
      !networkTargetEdited || !!networkTargetPending);
  }
  function applyNetworkTarget() {
    const state = networkAdapter.state;
    if (networkSimulation) {
      setText('networkEvent', '仿真服务锁定其启动目标；请修改服务配置后重启。');
      return;
    }
    if (!state || !networkAdapter.connected) {
      setText('networkEvent', '请先连接本机监控服务，再应用 Jetson 目标。');
      return;
    }
    if (state.armed) {
      setText('networkEvent', '请先停车并解除网络控制，再应用 Jetson 目标。');
      return;
    }
    const hostInput = $('jetsonHost'), portInput = $('jetsonPort');
    if (!hostInput || !portInput) return;
    const host = hostInput.value.trim();
    const port = Number(portInput.value);
    if (!host || !Number.isInteger(port) || port < 1 || port > 65535) {
      setText('networkEvent', '请输入有效的 Jetson 地址和 UDP 端口。');
      return;
    }
    if (networkAdapter.configure(host, port)) {
      networkTargetPending = {host,port,deadline:performance.now()+2000};
      setText('networkEvent', '已发送 Jetson 目标配置，等待本机服务状态确认。');
      updateNetworkTargetControls();
    }
  }
  function updateNetworkSnapshot() {
    const now = performance.now();
    const imu = showNetworkStream('imu','IMU',now);
    const motor = showNetworkStream('motor','电机',now);
    const lidar = showNetworkStream('lidar','雷达',now);
    const fusion = showNetworkStream('fusion','融合',now);
    const health = showNetworkStream('health','健康',now);
    const fusionData = fusion && fusion.data;
    const state = fusionData && fusionData.state;
    const stateKeys = ['x_m','y_m','yaw_rad','vx_m_s','vy_m_s','wz_rad_s','gyro_bias_rad_s'];
    const values = state && stateKeys.map(key => Number.isFinite(state[key]) ? state[key] : null);
    const uncertainty = networkApi.poseUncertainty(fusionData);
    const fusionOutputValid = !!(fusionData && Number.isInteger(fusionData.flags) && (fusionData.flags & 1));
    const hasState = !!(fusionOutputValid && values && values.every(value => value !== null));
    if (hasState) {
      const shown = [values[0],values[1],values[2]*180/Math.PI,values[3],values[4],values[5],values[6]];
      const sigmas = uncertainty.values;
      const sigmaShown = [sigmas[0],sigmas[1],sigmas[2] == null ? null : sigmas[2]*180/Math.PI,sigmas[3],sigmas[4],sigmas[5],sigmas[6]];
      shown.forEach((value,index) => {
        setText('v' + index, fmt(value,index===2?2:4));
        setText('sigma' + index, `${uncertainty.label} ${fmt(sigmaShown[index],index===2?2:4)} ${labels[index][1]}`);
      });
      updateOverviewMetrics(shown, sigmaShown, fusionData.sequence, uncertainty.label);
      const roll = state?.roll_rad, pitch = state?.pitch_rad;
      const attitude = Number.isFinite(roll) && Number.isFinite(pitch)
        ? ` · 横滚 ${fmt(roll*180/Math.PI,2)}° · 俯仰 ${fmt(pitch*180/Math.PI,2)}°` : '';
      setText('poseHeading', `${fmt(shown[2],2)}°${attitude} · ${fusionData.architecture === 'local_grid_v1' ? '位姿融合 / 局部栅格' : uncertainty.polar ? '位姿融合 / 点线ICP v2' : 'UDP 融合'}`);
      setText('lastSeen', `UDP fusion 序号 ${fusionData.sequence} · 接收 ${(networkAge(fusion,now)).toFixed(0)} ms 前`);
    } else {
      labels.forEach((_,index) => {
        setText('v' + index, '不可用');
        setText('sigma' + index, 'σ 不可用');
      });
      clearOverviewMetrics();
      setText('poseHeading', '不可用或已过期');
      setText('lastSeen', !fusion ? 'UDP fusion 不可用或已过期' : !fusionOutputValid
        ? `UDP fusion 输出无效（flags bit0=0）· 序号 ${fusion.data.sequence}`
        : `UDP fusion 状态字段无效 · 序号 ${fusion.data.sequence}`);
    }
    const diagnosticData = {fusion:fusionData, motor:motor?.data, imu:imu?.data, health:health?.data};
    diagnosticFields.forEach(([, stream, key, format], index) => {
      const polarMatch = uncertainty.polar && ['csm_accepted','csm_rejected'].includes(key);
      const displayKey = polarMatch ? key.replace('csm_', 'scan_match_') : key;
      setText('dl' + index, polarMatch ? key === 'csm_accepted' ? '点线ICP 接受数' : '点线ICP 拒绝数' : diagnosticFields[index][0]);
      const value = fusionData ? displayKey.split('.').reduce((data, field) => data?.[field], diagnosticData[stream]) : null;
      setText('d' + index, format ? format(value) : String(value ?? '不可用'));
    });
    setText('udpRawFlags', Number.isFinite(fusionData?.flags)
      ? 'UDP fusion.flags 原始值 0x' + fusionData.flags.toString(16).padStart(8, '0')
      : 'UDP fusion.flags 不可用');
    const wheelNames=['A · 左上','B · 右上','C · 左下','D · 右下'];
    const wheelMap = motor && motor.data.wheel_mm_s;
    const mapBytes = motor && motor.data;
    const mapValue = mapBytes && Number.isInteger(mapBytes.motor_channel_map) ? mapBytes.motor_channel_map : null;
    const encoderMap = mapBytes && Number.isInteger(mapBytes.encoder_channel_map) ? mapBytes.encoder_channel_map : null;
    const motorInvert = mapBytes && Number.isInteger(mapBytes.motor_invert) ? mapBytes.motor_invert : null;
    const encoderInvert = mapBytes && Number.isInteger(mapBytes.encoder_invert) ? mapBytes.encoder_invert : null;
    for (let index=0;index<4;index++) {
      const letter='ABCD'[index];
      const speed=wheelMap && Number.isFinite(wheelMap[letter]) ? wheelMap[letter] : null;
      const physical=mapValue===null ? '物理通道不可用' : `M${((mapValue >>> (index*2)) & 3)+1}`;
      setText(`wheelLabel${index}`, `逻辑轮 ${wheelNames[index]} → ${physical}`);
      setText(`wheelSpeed${index}`, speed===null?'不可用':fmt(speed,2));
    }
    const matches = mapValue===0xd2 && encoderMap===0xd2 && motorInvert===0x0b && encoderInvert===0x0a;
    setText('networkWheelMapping', !motor ? 'UDP 电机映射不可用或已过期。' :
      `motor_channel_map=${hexByte(mapValue)} · encoder_channel_map=${hexByte(encoderMap)}\n`+
      `motor_invert=${hexByte(motorInvert)} · encoder_invert=${hexByte(encoderInvert)}\n`+
      `与当前 FPGA 配置字节 ${matches?'匹配':'不匹配'}（期望 D2 / D2 / 0B / 0A）。当前配置轮径 98 mm、轴距 160 mm、轮距 196 mm、CPR 122880；均待整圈或落地标定。`);
    setText('imuRaw', !imu ? 'UDP IMU 不可用或已过期' :
      `WHO_AM_I=0x${Number(imu.data.who_am_i).toString(16).padStart(2,'0')} · flags=0x${Number(imu.data.flags).toString(16).padStart(2,'0')} · 样本序号 ${imu.data.sample_sequence}\n`+
      `gyro raw x/y/z: ${imu.data.gyro_raw?.x ?? '不可用'} / ${imu.data.gyro_raw?.y ?? '不可用'} / ${imu.data.gyro_raw?.z ?? '不可用'}\n`+
      `accel raw x/y/z: ${imu.data.accel_raw?.x ?? '不可用'} / ${imu.data.accel_raw?.y ?? '不可用'} / ${imu.data.accel_raw?.z ?? '不可用'}\n`+
      `gyro nominal x/y/z: ${fmt(imu.data.gyro_rad_s_nominal?.x,4)} / ${fmt(imu.data.gyro_rad_s_nominal?.y,4)} / ${fmt(imu.data.gyro_rad_s_nominal?.z,4)} rad/s`);

    const navSnapshot = freshNetworkItem('navigation', now)?.data;
    setText('plannerState', navSnapshot ? `导航状态 ${navSnapshot.status}` : '等待新鲜导航状态');
    setText('plannerCommand', navSnapshot ? `vx ${navSnapshot.vx_mm_s} · vy ${navSnapshot.vy_mm_s} mm/s · wz ${navSnapshot.wz_mrad_s} mrad/s` : '规划器命令不可用');
    const plannerCard = $('plannerStateCard');
    if (plannerCard && plannerCard.dataset) plannerCard.dataset.state=navSnapshot ? 'active' : 'waiting';
    occupancyView.draw();
    updateWorkbench();
  }
  function networkCurrentVector() {
    let vx=(pressedNetworkKeys.has('w')?1:0)-(pressedNetworkKeys.has('s')?1:0);
    let vy=(pressedNetworkKeys.has('a')?1:0)-(pressedNetworkKeys.has('d')?1:0);
    const magnitude=Math.hypot(vx,vy);
    if(magnitude>1){vx/=magnitude;vy/=magnitude;}
    const wz=(pressedNetworkKeys.has('q')?1:0)-(pressedNetworkKeys.has('e')?1:0);
    return {vx:Math.round(vx*networkSpeed),vy:Math.round(vy*networkSpeed),wz:Math.round(wz*networkTurn)};
  }
  function sendNetworkDrive() {
    if (!networkAdapter.isOwner()) return false;
    const {vx, vy, wz} = networkCurrentVector();
    const values = [vx, vy, wz];
    if (!values.every(Number.isFinite) || Math.abs(vx) > 400 || Math.abs(vy) > 400 || Math.abs(wz) > 800) return false;
    if (values.some(value => value !== 0) && (!driveStateIsFresh() || !pageCanControl())) return false;
    return networkAdapter.drive(vx, vy, wz);
  }
  function startNetworkHeartbeat() {
    if(networkHeartbeat)window.clearInterval(networkHeartbeat);
    networkHeartbeat=window.setInterval(()=>{
      networkAdapter.freshnessTick();
      sendNetworkDrive();
    },50);
  }
  function clearManualInput() {
    networkAdapter.localControlEnabled = false;
    pressedNetworkKeys.clear();
    if(networkHeartbeat)window.clearInterval(networkHeartbeat);
    networkHeartbeat=0;
    networkArmPendingUntil=0;
  }
  function releaseNetworkControl(reason) {
    const stopSent = networkAdapter.stop(reason);
    networkStopLocalRequested = {sent:stopSent,at:performance.now(),reason};
    clearManualInput();
    updateNetworkStopEvidence(networkAdapter.state);
    updateNetworkControls();
  }
  function connectNetwork() {
    resetNetworkTelemetry();
    const protocol=location.protocol==='https:'?'wss:':'ws:';
    const url=`${protocol}//${location.host}/ws`;
    try {
      occupancyView.connect(`${protocol}//${location.host}/map`);
      if(networkAdapter.connected){
        setText('networkEvent', '监控连接已存在；修改 Jetson 目标后请点击“应用目标”。');
      } else networkAdapter.connect(url);
    } catch(error) { setText('networkEvent', `网络连接失败：${error.message||error}`); }
    updateNetworkSnapshot(); updateNetworkControls(); updateNetworkTargetControls();
  }
  async function disconnectNetwork() {
    occupancyView.disconnect();
    if(networkAdapter.localControlEnabled)releaseNetworkControl('断开网络前停车并解除控制');
    await new Promise(resolve=>window.setTimeout(resolve,80));
    networkAdapter.closeSocket();
    networkAdapter.reset('断开网络监控');
    networkStopLocalRequested = null;
    onNetworkConnection(false,'本机网络监控已断开');
    resetNetworkTelemetry();
  }
  function allowNetworkControl() {
    if(!networkCanArm())return;
    if(networkAdapter.arm()){
      clearWaypoints();
      networkArmPendingUntil=performance.now()+1500;
      pressedNetworkKeys.clear();
      startNetworkHeartbeat();
      setText('networkEvent', '已请求网络控制权，等待本机服务确认 owner。');
      updateNetworkControls();
    }
  }
  function connectVideo() {
    if(networkVideoSocket&&networkVideoSocket.readyState<2)return;
    const protocol=location.protocol==='https:'?'wss:':'ws:';
    const socket=new WebSocket(`${protocol}//${location.host}/video`);
    networkVideoSocket=socket; socket.binaryType='arraybuffer';
    socket.onopen=()=>{
      setDisabled('videoConnect', true); setDisabled('videoDisconnect', false);
      setChip('videoStatus','视频：已连接','fresh'); setText('videoMetrics','等待完整 JPEG 帧');
    };
    socket.onmessage=event=>{
      if(!(event.data instanceof ArrayBuffer))return;
      const bytes=new Uint8Array(event.data);
      if(bytes.length<4||bytes.length>512*1024||bytes[0]!==0xff||bytes[1]!==0xd8||bytes[bytes.length-2]!==0xff||bytes[bytes.length-1]!==0xd9){networkVideoDrops++;return;}
      const now=performance.now(); networkVideoFrames.push(now); networkVideoFrames=networkVideoFrames.filter(value=>now-value<=1000);
      networkVideoLastAt=now;
      if(networkVideoObjectUrl)URL.revokeObjectURL(networkVideoObjectUrl);
      networkVideoObjectUrl=URL.createObjectURL(new Blob([event.data],{type:'image/jpeg'}));
      const image=$('networkVideo');
      if (image) {
        image.onload=()=>{
          networkVideoWidth=image.naturalWidth; networkVideoHeight=image.naturalHeight;
          networkVideoDecodedAt=performance.now(); updateVisionUi();
        };
        image.src=networkVideoObjectUrl;
      }
      setVideoFrameVisible(true);
      updateVideoMetrics();
    };
    socket.onerror=()=>setChip('videoStatus','视频：WebSocket 错误','fault');
    socket.onclose=()=>{
      if(networkVideoSocket!==socket)return;
      networkVideoSocket=null; setDisabled('videoConnect', false); setDisabled('videoDisconnect', true);
      networkVideoDecodedAt=0; networkVideoWidth=0; networkVideoHeight=0;
      if(networkVideoObjectUrl)URL.revokeObjectURL(networkVideoObjectUrl); networkVideoObjectUrl=null;
      const image=$('networkVideo');
      if (image) image.removeAttribute('src');
      setVideoFrameVisible(false);
      drawVisionOverlay();
      setChip('videoStatus','视频：已断开','stale'); setText('videoMetrics','视频已断开');
    };
  }
  function disconnectVideo() { if(networkVideoSocket)networkVideoSocket.close(); }
  function updateVideoMetrics() {
    updateVisionUi();
    const now=performance.now(); networkVideoFrames=networkVideoFrames.filter(value=>now-value<=1000);
    const age=networkVideoLastAt?Math.max(0,now-networkVideoLastAt):null;
    const stale=age===null||age>1000;
    const health = networkAdapter.connected ? freshNetworkItem('health',now)?.data : null;
    const calibration = health?.camera_calibration;
    let camera = '';
    if (calibration?.fault === true) {
      const reason = health.camera_autocal_diag_valid === true &&
        health.camera_autocal_diag?.reason_name === 'bad_csi_burst' ? ' · CSI 突发异常' : '';
      const retries = health.camera_autocal_diag_valid === true &&
        Number.isInteger(health.camera_autocal_diag?.retry_count)
        ? ` · 重试 ${health.camera_autocal_diag.retry_count} 次` : '';
      const state = Number.isInteger(calibration.state) ? ` · 状态 ${calibration.state}` : '';
      const packets = Number.isInteger(health.video_packets) && health.video_packets >= 0
        ? ` · FPGA视频包累计 ${health.video_packets}` : '';
      camera = ` · 相机采集校准故障${state}${reason}${retries}${packets}`;
    }
    if(!networkVideoSocket){
      setChip('videoStatus','视频：未连接','stale');
      setText('videoMetrics',`视频未连接${camera}`);
      return;
    }
    setChip('videoStatus',stale?'视频：断流或帧已过期':`视频：新鲜 ${(age/1000).toFixed(2)} s`,stale?'stale':'fresh');
    const serverVideo=networkAdapter.state?.video||{};
    setText('videoMetrics', `浏览器 fps ${networkVideoFrames.length} · 帧龄 ${age===null?'不可用':`${age.toFixed(0)} ms`} · 服务 fps ${Number.isFinite(serverVideo.fps)?serverVideo.fps:'不可用'} · 丢帧 ${Number.isFinite(serverVideo.dropped)?serverVideo.dropped:networkVideoDrops}${camera}`);
  }
  function openConnectionSettings() {
    stopAllControl('打开连接配置');
    const dialog = $('wbConnectionDialog');
    if (dialog && !dialog.open && typeof dialog.showModal === 'function') {
      try { dialog.showModal(); } catch (_) { dialog.open = true; }
    }
    const closeButton = $('wbConnectionClose');
    if (closeButton && typeof closeButton.focus === 'function') closeButton.focus();
  }
  function stopAllControl(reason) {
    clearWaypoints();
    if (networkAdapter.connected)
      releaseNetworkControl(reason);
  }
  function stopOnFocusLoss(reason, stopVision = false) {
    if (networkAdapter.localControlEnabled || networkArmPendingUntil ||
        (stopVision && currentVision()?.ros_authorized))
      stopAllControl(reason);
  }
  function pageCanControl() {
    return !$('wbConnectionDialog')?.open &&
      !document.hidden && (typeof document.hasFocus !== 'function' || document.hasFocus());
  }
  function changeNetworkKey(key, pressed) {
    if (!networkAdapter.isOwner()) return;
    if (pressed) { if (pressedNetworkKeys.has(key)) return; pressedNetworkKeys.add(key); }
    else { if (!pressedNetworkKeys.has(key)) return; pressedNetworkKeys.delete(key); }
    updateNetworkControls(); sendNetworkDrive();
  }
  const keyboardNetworkMap = {KeyW:'w',KeyA:'a',KeyS:'s',KeyD:'d',KeyQ:'q',KeyE:'e'};
  function isEditableTarget(target) {
    return !!target && (target.isContentEditable || ['INPUT','TEXTAREA','SELECT'].includes(target.tagName));
  }
  document.addEventListener('keydown', event => {
    if (isEditableTarget(event.target)) return;
    if ((event.code === 'Space' || event.code === 'Escape') && networkAdapter.isOwner()) {
      event.preventDefault(); stopAllControl('键盘停车并解除控制'); return;
    }
    if (!networkAdapter.localControlEnabled) return;
    const key = keyboardNetworkMap[event.code];
    if (key) { event.preventDefault(); changeNetworkKey(key,true); }
  });
  document.addEventListener('keyup', event => {
    const key = keyboardNetworkMap[event.code];
    if (networkAdapter.localControlEnabled && key && pressedNetworkKeys.has(key)) {
      event.preventDefault(); changeNetworkKey(key,false);
    }
  });
  window.setInterval(() => {
    networkAdapter.freshnessTick();
    updateNetworkSnapshot(); updateNetworkControls(); updateNetworkTargetControls();
    updateNetworkStateUi(networkAdapter.state); updateVideoMetrics(); updateVisionUi();
  }, 250);
  window.addEventListener('blur', () => stopOnFocusLoss('窗口失焦，停车并清除授权'));
  document.addEventListener('visibilitychange', () => {
    if (document.hidden) stopOnFocusLoss('页面隐藏，停车并清除授权');
  });
  window.addEventListener('pagehide', () => stopOnFocusLoss('页面关闭，停车并清除授权', true));
  document.querySelectorAll('[data-network-key]').forEach(button=>{
    const key=button.dataset.networkKey;
    button.addEventListener('pointerdown',event=>{
      if(!networkAdapter.isOwner())return;
      event.preventDefault(); if(button.setPointerCapture)button.setPointerCapture(event.pointerId); changeNetworkKey(key,true);
    });
    const release=event=>{event.preventDefault();changeNetworkKey(key,false);};
    button.addEventListener('pointerup',release); button.addEventListener('pointercancel',release); button.addEventListener('lostpointercapture',release);
  });
  bind('networkConnect', 'click', connectNetwork);
  bind('networkApplyTarget', 'click', applyNetworkTarget);
  ['jetsonHost','jetsonPort'].forEach(id=>bind(id, 'input', ()=>{
    networkTargetEdited=true; networkTargetPending=null; updateNetworkTargetControls();
  }));
  bind('networkDisconnect', 'click', ()=>disconnectNetwork());
  bind('networkArm', 'click', allowNetworkControl);
  bind('networkStop', 'click', ()=>{ stopAllControl('用户请求停车并解除控制'); setEvidenceOpen(); });
  bind('wbStop', 'click', ()=>{ stopAllControl('用户从顶部请求停车并解除控制'); setEvidenceOpen(); });
  function setEvidenceOpen() {
    const evidence = $('wbStopEvidence');
    if (evidence) evidence.open = true;
  }
  for (const id of ['networkSpeed', 'networkTurn']) bind(id, 'input', () => {
    const value = Number($(id)?.value);
    if (!Number.isFinite(value)) return;
    if (id === 'networkSpeed') networkSpeed = value;
    else networkTurn = value;
    updateNetworkControls();
    sendNetworkDrive();
  });
  bind('videoConnect', 'click', ()=>connectVideo());
  bind('videoDisconnect', 'click', ()=>disconnectVideo());
  for (const [id, action] of Object.entries({visionEnable:'enable_ros', visionDisable:'disable_ros',
    visionStart:'start', visionStop:'stop', visionEstop:'emergency_stop', visionRelease:'release_emergency_stop'})) {
    bind(id, 'click', ()=>requestVisionAction(action));
  }
  bind('visionOverlay', 'click', selectVisionAt);
  bind('visionBehavior', 'change', updateVisionUi);
  bind('visionDraftSelect', 'change', () => {
    const vision=currentVision(), index=Number($('visionDraftSelect')?.value)-1;
    selectVisionDraft(visionFrameReady(vision) && index >= 0 ? vision.detections?.[index] : null);
  });
  bind('localTaskStart', 'click', () => requestNavigationTask('start'));
  bind('localTaskStop', 'click', () => requestNavigationTask('stop'));
  bind('localTaskCancel', 'click', () => requestNavigationTask('cancel'));
  function exportCsv() {
    const header = ['source','stream','received_at','protocol_version','sequence','timestamp_us','bridge_id','epoch','pc_rx_wall_ms','pc_rx_mono_ms','jetson_rx_mono_s','jetson_tx_mono_s',
      'x_m','y_m','yaw_rad','vx_m_s','vy_m_s','wz_rad_s','gyro_bias_rad_s','covariance_diag_json','flags','updates','ekf_rejected','saturated','input_dropped',
      'lidar_rejected','scans','gyro_age_us','lidar_age_us','lidar_score','queue_dropped','wheel_0_mm_s','wheel_1_mm_s','wheel_2_mm_s','wheel_3_mm_s',
      'motor_channel_map','encoder_channel_map','motor_invert','encoder_invert','active_physical_mask','encoder_invalid_mask','wheel_targets_json',
      'gyro_raw_json','accel_raw_json','lidar_points_json','navigation_architecture','error_estimate_mm_mm_mrad_json',
      'lidar_raw_samples','lidar_reduced_samples','scan_match_accepted','scan_match_rejected','raw_data_json'];
    const lines = [header.join(',')];
    for (const row of csvRows) {
      if (row.source !== 'udp') continue;
      const e=row.envelope||{},d=e.data||{},state=d.state||{},wheels=d.wheel_mm_s||{},targets=d.target_mm_s||{};
      const stateKeys=['x_m','y_m','yaw_rad','vx_m_s','vy_m_s','wz_rad_s','gyro_bias_rad_s'];
      const fusedState=stateKeys.map(key=>state[key] ?? '');
      const values=['udp',row.stream,row.receivedAt,e.source_version ?? '',d.sequence,d.time_us,e.bridge_id,e.epoch,e.pc_rx_wall_ms,e.pc_rx_mono_ms,e.jetson_rx_mono_s,e.jetson_tx_mono_s,
        ...fusedState,JSON.stringify(d.covariance_diag||[]),Number.isFinite(d.flags)?'0x'+d.flags.toString(16):'',d.updates,d.ekf_rejected,d.saturated,d.input_dropped,
        d.lidar_rejected,d.scans,d.gyro_age_us,d.lidar_age_us,d.lidar_score,d.queue_dropped,
        wheels.A ?? '',wheels.B ?? '',wheels.C ?? '',wheels.D ?? '',d.motor_channel_map,d.encoder_channel_map,d.motor_invert,d.encoder_invert,d.active_physical_mask,d.encoder_invalid_mask,
        JSON.stringify(targets),JSON.stringify(d.gyro_raw||{}),JSON.stringify(d.accel_raw||{}),JSON.stringify(d.points||[]),
        d.architecture ?? d.protocol ?? '',JSON.stringify(d.error_estimate||[]),
        d.lidar_raw_samples,d.lidar_reduced_samples,d.scan_match_accepted,d.scan_match_rejected,JSON.stringify(e)];
      lines.push(values.map(csvCell).join(','));
    }
    const blob = new Blob(['\ufeff',lines.join('\r\n')],{type:'text/csv;charset=utf-8'});
    const url = URL.createObjectURL(blob); const a = document.createElement('a');
    a.href=url; a.download='bx72_telemetry_'+new Date().toISOString().replaceAll(':','-')+'.csv'; a.click(); URL.revokeObjectURL(url);
  }
  function csvCell(value) {
    const text = value === null || value === undefined ? '' : String(value);
    return /[",\r\n]/.test(text) ? `"${text.replaceAll('"','""')}"` : text;
  }

  // ---------- 单页导航：锚点定位与当前工作区高亮 ----------
  const sectionTargets = {overview:'overview', tasks:'tasks', remote:'remotePage', sensors:'sensors', diagnostics:'diagnosticsPage'};
  const sectionTitles = {overview:'运行总览', tasks:'任务中心', remote:'手动遥控', sensors:'传感器', diagnostics:'诊断与连接'};
  const navNodes = Array.from(document.querySelectorAll('a[data-nav]'));
  const reducedMotion = typeof window.matchMedia === 'function' && window.matchMedia('(prefers-reduced-motion: reduce)').matches;
  function navTargetId(node) {
    return node?.dataset?.nav || node?.getAttribute('data-nav');
  }
  function setActiveNav(id) {
    navNodes.forEach(node => {
      const active = navTargetId(node) === id;
      node.classList.toggle('is-active', active);
      if (active) node.setAttribute('aria-current', 'location');
      else node.removeAttribute('aria-current');
    });
    setText('pageBreadcrumb', sectionTitles[id] || sectionTitles.overview);
  }
  function navigateToSection(id) {
    if (id === 'connectionSettings') { openConnectionSettings(); return; }
    const target = id === 'overview' ? $('workspaceContent') : $(sectionTargets[id]);
    if (!target) return;
    if (id !== 'remote' && (networkAdapter.localControlEnabled || networkArmPendingUntil > performance.now()))
      releaseNetworkControl('切换工作区导航，停车并解除控制');
    if (typeof target.scrollIntoView === 'function')
      target.scrollIntoView({behavior:reducedMotion ? 'auto' : 'smooth', block:'start'});
    setActiveNav(id);
  }
  navNodes.forEach(node => node.addEventListener('click', event => {
    const id = navTargetId(node);
    if (!id) return;
    event.preventDefault();
    if (id !== 'connectionSettings' && window.history?.pushState && location.hash !== '#' + id)
      window.history.pushState(null, '', '#' + id);
    navigateToSection(id);
  }));
  function routeFromLocation() { navigateToSection((location.hash || '#overview').slice(1)); }
  window.addEventListener('hashchange', routeFromLocation);
  window.addEventListener('popstate', routeFromLocation);
  let navHighlightQueued = false;
  function updateActiveSection() {
    navHighlightQueued = false;
    let active = 'overview';
    for (const [id, targetId] of Object.entries(sectionTargets)) {
      const rect = $(targetId)?.getBoundingClientRect();
      if (rect && rect.top <= 140 && (id !== 'tasks' || rect.bottom > 140)) active = id;
    }
    setActiveNav(active);
  }
  window.addEventListener('scroll', () => {
    if (navHighlightQueued) return;
    navHighlightQueued = true;
    if (typeof window.requestAnimationFrame === 'function') window.requestAnimationFrame(updateActiveSection);
    else updateActiveSection();
  }, {passive:true});
  if (location.hash) routeFromLocation();
  else setActiveNav('overview');

  // ---------- 轮速趋势时间窗：30 s / 2 min / 10 min（DESIGN 5.7 / 8） ----------
  const TREND_WINDOW_MS = {30:30000, 120:120000, 600:600000};
  function selectTrendRange(windowMs) {
    if (typeof workbench.setWindow === 'function') workbench.setWindow(windowMs);
    const box = $('trendRanges');
    if (!box || typeof box.querySelectorAll !== 'function') return;
    box.querySelectorAll('.range-btn').forEach(button => {
      const range = Number(button.dataset && button.dataset.range);
      const active = TREND_WINDOW_MS[range] === windowMs;
      button.classList.toggle('is-active', active);
      button.classList.toggle('active', active);
      if (typeof button.setAttribute === 'function')
        button.setAttribute('aria-pressed', active ? 'true' : 'false');
    });
  }
  const trendRangeBox = $('trendRanges');
  if (trendRangeBox) trendRangeBox.addEventListener('click', event => {
    const node = event.target;
    const button = node && typeof node.closest === 'function' ? node.closest('.range-btn') : null;
    if (!button) return;
    const range = Number(button.dataset && button.dataset.range);
    if (TREND_WINDOW_MS[range]) selectTrendRange(TREND_WINDOW_MS[range]);
  });
  selectTrendRange(30000);

  bind('wbConfigure', 'click', () => openConnectionSettings());
  bind('wbConnectionClose', 'click', () => {
    const dialog = $('wbConnectionDialog');
    if (dialog && typeof dialog.close === 'function') dialog.close();
  });
  bind('export', 'click', exportCsv);
  bind('clear', 'click', () => {
    csvRows=[]; workbench.reset(); clearOverviewMetrics(); updateStats();
  });
  window.addEventListener('resize', () => { occupancyView.draw(); workbench.resize(); drawVisionOverlay(); });
  if (window.ResizeObserver && $('visionOverlay')) {
    new window.ResizeObserver(()=>drawVisionOverlay()).observe($('visionOverlay').parentElement);
  }
  setVideoFrameVisible(false);
  updateNetworkControls(); updateLocalTaskUi(); updateStats(); occupancyView.draw(); updateVideoMetrics(); updateWorkbench();
  // 由本机服务打开时自动连接 /ws 监控与 /video 视频流（file:// 打开或服务未启动时保持手动连接）。
  if (location.protocol === 'http:' || location.protocol === 'https:') {
    connectNetwork();
    try { connectVideo(); } catch (_) { /* 视频不可用时保持占位画面 */ }
  }
})();
