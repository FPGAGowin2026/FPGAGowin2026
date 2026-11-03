/* BX72 前端网络适配器：UDP 遥测、控制与展示。 */
(function (root, factory) {
  const api = factory();
  if (typeof module === 'object' && module.exports) module.exports = api;
  if (root) root.BX72DashboardNetwork = api;
})(typeof globalThis === 'undefined' ? this : globalThis, function () {
  'use strict';

  const STREAMS = new Set(['imu', 'motor', 'lidar', 'fusion', 'health', 'navigation']);
  const OPEN = 1;
  const PRESENTATION_DEFAULT_WINDOW_MS = 20000;
  const PRESENTATION_STALE_MS = 1000;
  const PRESENTATION_MAX_SAMPLES = 2000;
  const PRESENTATION_COLORS = ['#67e8f9', '#60a5fa', '#34d399', '#fbbf24'];
  // 默认主题保持旧页面观感；新页面通过 options.theme 传入浅色配色。
  const PRESENTATION_THEME = Object.freeze({
    grid:'#263449', axis:'#8795a9', lines:PRESENTATION_COLORS, stale:'#fa932c'
  });

  function uint32IsNewer(candidate, previous) {
    const delta = (candidate - previous) >>> 0;
    return delta !== 0 && delta < 0x80000000;
  }

  // 极坐标与局部栅格布局均报告mm/mm/mrad误差估计；旧布局使用SI协方差。
  // 计数字段不能被误画为协方差。
  function poseUncertainty(data) {
    const polar = Number.isInteger(data?.flags) && (data.flags & 0x4000) !== 0;
    if (polar) {
      const e = Array.isArray(data.error_estimate) ? data.error_estimate : [];
      return {polar:true, label:'估计误差', values:Array.from({length:7}, (_, i) =>
        i < 3 && Number.isFinite(e[i]) && e[i] >= 0 ? e[i] / 1000 : null)};
    }
    const p = Array.isArray(data?.covariance_diag) ? data.covariance_diag : [];
    return {polar:false, label:'σ', values:Array.from({length:7}, (_, i) =>
      Number.isFinite(p[i]) && p[i] >= 0 ? Math.sqrt(p[i]) : null)};
  }

  class DashboardNetworkAdapter {
    constructor(options = {}) {
      this.WebSocketImpl = options.WebSocketImpl || (typeof WebSocket !== 'undefined' ? WebSocket : null);
      this.now = options.now || (() => (typeof performance !== 'undefined' ? performance.now() : Date.now()));
      this.onConnection = options.onConnection || (() => {});
      this.onHello = options.onHello || (() => {});
      this.onState = options.onState || (() => {});
      this.onTelemetry = options.onTelemetry || (() => {});
      this.onEvent = options.onEvent || (() => {});
      this.onMissionResult = options.onMissionResult || this.onEvent;
      this.onReset = options.onReset || (() => {});
      this.socket = null;
      this.hello = null;
      this.state = null;
      this.stateReceivedAt = 0;
      this.streams = new Map();
      this.lastSequence = new Map();
      this.bridgeId = null;
      this.epoch = null;
      this.nextSequence = 0;
      this.localControlEnabled = false;
      this.staleReported = false;
      this.deliveryFault = false;
    }

    connect(url) {
      if (!this.WebSocketImpl) throw new Error('浏览器不支持 WebSocket');
      this.closeSocket();
      this.reset('连接网络服务');
      const socket = new this.WebSocketImpl(url);
      this.socket = socket;
      socket.onopen = () => this.onConnection(true, '本机网络服务已连接');
      socket.onmessage = event => {
        if (this.socket !== socket) return;
        try { this.receive(event.data); }
        catch (error) {
          // 页面处理失败时不得确认消费，服务端会按失联路径撤权。
          this.deliveryFault = true;
          throw error;
        }
      };
      socket.onerror = () => this.onEvent({event:'websocket_error', reason:'WebSocket 网络错误'});
      socket.onclose = event => {
        if (this.socket !== socket) return;
        this.socket = null;
        this.localControlEnabled = false;
        this.state = null;
        this.stateReceivedAt = 0;
        this.onState(null, null);
        const detail = event?.code === 1013 ?
          (event.reason === 'client consumption stalled' ? '：页面消费确认超时（1013）' :
            event.reason === 'client transport stalled' ? '：网络发送停滞（1013）' : '：页面接收积压（1013）') :
          event?.code && event.code !== 1000 ? `（关闭码 ${event.code}）` : '';
        this.onConnection(false, `本机网络服务已断开${detail}`);
      };
      return socket;
    }

    closeSocket() {
      const socket = this.socket;
      if (socket && socket.readyState < 2) socket.close();
      else if (!socket) return;
      else this.socket = null;
    }

    reset(reason) {
      this.hello = null;
      this.state = null;
      this.stateReceivedAt = 0;
      this.streams.clear();
      this.lastSequence.clear();
      this.bridgeId = null;
      this.epoch = null;
      this.localControlEnabled = false;
      this.staleReported = false;
      this.deliveryFault = false;
      this.onReset(reason);
      this.onState(null, null);
    }

    get connected() { return this.socket?.readyState === OPEN; }

    send(message) {
      if (!this.connected) return false;
      this.socket.send(JSON.stringify({v:1, ...message}));
      return true;
    }

    configure(jetson, port) {
      if (this.state && this.state.armed) return false;
      return this.send({type:'configure', jetson, port});
    }

    arm() {
      if (!this.send({type:'arm'})) return false;
      this.localControlEnabled = true;
      return true;
    }

    drive(vx, vy, wz) {
      this.nextSequence = (this.nextSequence + 1) >>> 0;
      return this.send({type:'drive', seq:this.nextSequence, vx:Math.round(vx), vy:Math.round(vy), wz:Math.round(wz)});
    }

    auto(enabled) { return this.send({type:'auto', enabled:!!enabled}); }

    missionCommand(action, fields = {}) {
      if (!['enable_ros','disable_ros','start','stop','emergency_stop','release_emergency_stop'].includes(action)) return false;
      return this.send({type:'mission_command', action, ...fields});
    }

    navigationTask(action, fields = {}) {
      if (!['start','stop','cancel'].includes(action)) return false;
      return this.send({type:'navigation_task', action, ...fields});
    }

    stop(reason = '用户停车/解除控制') {
      this.localControlEnabled = false;
      return this.send({type:'stop', reason:String(reason)});
    }

    isOwner() {
      return !!(this.localControlEnabled && this.hello && this.hello.client_id && this.state &&
        this.state.armed && this.state.owner === this.hello.client_id);
    }

    latest(stream, maxAgeMs = stream === 'health' ? 2000 : stream === 'navigation' ? 500 : 1000, at = this.now()) {
      const item = this.streams.get(stream);
      return item && at - item.receivedAt <= maxAgeMs ? item : null;
    }

    freshnessTick(at = this.now()) {
      if (this.isOwner() && this.stateReceivedAt && at - this.stateReceivedAt > 500 && !this.staleReported) {
        this.staleReported = true;
        this.stop('状态心跳超时');
        this.onEvent({event:'state_timeout', reason:'网络状态超过 500 ms 未更新，已清除本页控制授权'});
      }
    }

    receive(raw) {
      if (typeof raw !== 'string') return;
      let message;
      try { message = JSON.parse(raw); } catch (_) { return; }
      if (!message || message.v !== 1 || typeof message.type !== 'string') return;
      const at = this.now();
      if (message.type === 'hello') {
        this.hello = message;
        if (message.delivery_ack === 1) this.send({type:'delivery_ready'});
        this.onHello(message);
      } else if (message.type === 'delivery_probe') {
        // 该探针排在一个批次之后；只有前面的状态和遥测回调都完成才确认。
        if (!this.deliveryFault && Number.isInteger(message.id) && message.id >= 0 && message.id <= 0xffffffff)
          this.send({type:'delivery_ack',id:message.id});
      } else if (message.type === 'state') {
        this.state = message;
        this.stateReceivedAt = at;
        this.staleReported = false;
        const reportedBridge = message.status && message.status.bridge_id;
        if (reportedBridge && reportedBridge !== this.bridgeId) this.changeBridge(reportedBridge, '网关进程已更换');
        this.onState(message, at);
      } else if (message.type === 'telemetry') {
        this.receiveTelemetry(message, at);
      } else if (message.type === 'event') {
        this.onEvent(message);
      } else if (message.type === 'mission_result' && typeof message.success === 'boolean' &&
                 typeof message.message === 'string') {
        this.onMissionResult(message);
      }
    }

    changeBridge(bridgeId, reason) {
      this.bridgeId = bridgeId;
      this.epoch = null;
      this.streams.clear();
      this.lastSequence.clear();
      this.onReset(reason);
    }

    receiveTelemetry(message, at) {
      if (!STREAMS.has(message.stream) || message.valid !== true || !message.data || typeof message.data !== 'object') return;
      if (!Number.isInteger(message.epoch) || !Number.isInteger(message.data.sequence)) return;
      const expectedBridge = this.state && this.state.status && this.state.status.bridge_id;
      if (expectedBridge && message.bridge_id !== expectedBridge) return;
      if (this.bridgeId && message.bridge_id !== this.bridgeId) return;
      if (!this.bridgeId) this.bridgeId = message.bridge_id;
      if (this.epoch !== null && message.epoch !== this.epoch) {
        if (!uint32IsNewer(message.epoch >>> 0, this.epoch >>> 0)) return;
        this.epoch = message.epoch;
        this.lastSequence.clear();
        this.streams.clear();
        this.onReset('Jetson 遥测 epoch 已变化');
      } else if (this.epoch === null) this.epoch = message.epoch;

      const previous = this.lastSequence.get(message.stream);
      const sequence = message.data.sequence >>> 0;
      if (previous !== undefined && !uint32IsNewer(sequence, previous)) return;
      this.lastSequence.set(message.stream, sequence);
      const item = {envelope:message, data:message.data, receivedAt:at};
      this.streams.set(message.stream, item);
      this.onTelemetry(message.stream, item);
    }
  }

  // 界面辅助展示：只接收调用方归一化后的轮速和状态，不解析或改变任何协议。
  // options.windowMs / options.theme 是新增的可选增量，缺省值与旧页面完全一致。
  function createWorkbenchPresentation(options = {}) {
    const doc = options.document || (typeof globalThis !== 'undefined' ? globalThis.document : null);
    const win = options.window || (typeof globalThis !== 'undefined' ? globalThis.window : null);
    const redraw = typeof options.redraw === 'function' ? options.redraw : null;
    const canvas = doc && typeof doc.getElementById === 'function' ? doc.getElementById('telemetryTrend') : null;
    const trendStatus = doc && typeof doc.getElementById === 'function' ? doc.getElementById('telemetryTrendStatus') : null;
    const headers = {
      transport:doc && doc.getElementById('wbTransport'),
      connection:doc && doc.getElementById('wbConnection'),
      simulation:doc && doc.getElementById('wbSimulation'),
      actualMode:doc && doc.getElementById('wbActualMode')
    };
    let context = null;
    try { if (canvas && typeof canvas.getContext === 'function') context = canvas.getContext('2d'); } catch (_) {}

    const requestedWindow = Number(options.windowMs);
    let windowMs = Number.isFinite(requestedWindow) && requestedWindow >= 1000
      ? Math.round(requestedWindow) : PRESENTATION_DEFAULT_WINDOW_MS;
    const requestedTheme = options.theme && typeof options.theme === 'object' ? options.theme : {};
    const theme = {
      grid:typeof requestedTheme.grid === 'string' ? requestedTheme.grid : PRESENTATION_THEME.grid,
      axis:typeof requestedTheme.axis === 'string' ? requestedTheme.axis : PRESENTATION_THEME.axis,
      lines:Array.isArray(requestedTheme.lines) && requestedTheme.lines.length >= 4
        ? Array.from({length:4}, (_, index) =>
          typeof requestedTheme.lines[index] === 'string' ? requestedTheme.lines[index] : PRESENTATION_COLORS[index])
        : PRESENTATION_COLORS,
      stale:typeof requestedTheme.stale === 'string' ? requestedTheme.stale : PRESENTATION_THEME.stale
    };

    let samples = [];
    let lastSampleAt = null;
    let lastSource = null;
    let sourceInitialized = false;
    let inGap = true;
    let lastStatus = `轮速趋势 · 等待新鲜轮速 · mm/s · 最近 ${windowWords(windowMs)}`;
    let lastNowMs = 0;

    function windowWords(ms) {
      if (ms >= 60000) return `${Number((ms / 60000).toFixed(1))} 分钟`;
      return `${Number((ms / 1000).toFixed(1))} 秒`;
    }

    function axisWords(seconds) {
      if (seconds >= 60) return `${Number((seconds / 60).toFixed(1))} min`;
      return `${Number(seconds.toFixed(1))} s`;
    }

    function setText(element, value) {
      if (element) element.textContent = value;
    }

    function clearTrend(preserveSampleAt = false) {
      samples = [];
      if (!preserveSampleAt) lastSampleAt = null;
      inGap = true;
    }

    function prune(nowMs) {
      if (Number.isFinite(nowMs)) {
        const cutoff = nowMs - windowMs;
        while (samples.length && samples[0].at < cutoff) samples.shift();
      }
      if (samples.length > PRESENTATION_MAX_SAMPLES)
        samples.splice(0, samples.length - PRESENTATION_MAX_SAMPLES);
    }

    function markGap(at) {
      if (!inGap) {
        samples.push({at:Number.isFinite(at) ? at : lastNowMs, wheels:null});
        inGap = true;
      }
    }

    function wheelValues(value) {
      if (!Array.isArray(value) || value.length < 4) return null;
      return Array.from({length:4}, (_, index) => Number.isFinite(value[index]) ? value[index] : null);
    }

    function formatValue(value) {
      if (!Number.isFinite(value)) return '—';
      return Number(value.toFixed(1)).toString();
    }

    function sizeOf(target, axis, fallback) {
      if (!target) return fallback;
      const client = axis === 'width' ? target.clientWidth : target.clientHeight;
      if (Number.isFinite(client) && client > 0) return client;
      if (typeof target.getBoundingClientRect === 'function') {
        const rect = target.getBoundingClientRect();
        const measured = rect && rect[axis];
        if (Number.isFinite(measured) && measured > 0) return measured;
      }
      const intrinsic = target[axis];
      return Number.isFinite(intrinsic) && intrinsic > 0 ? intrinsic : fallback;
    }

    function drawTrend() {
      if (!canvas || !context) return;
      const width = sizeOf(canvas, 'width', 640);
      const height = sizeOf(canvas, 'height', 190);
      const pixelRatio = win && Number.isFinite(win.devicePixelRatio) && win.devicePixelRatio > 0
        ? win.devicePixelRatio : 1;
      const pixelWidth = Math.max(1, Math.round(width * pixelRatio));
      const pixelHeight = Math.max(1, Math.round(height * pixelRatio));
      if (canvas.width !== pixelWidth) canvas.width = pixelWidth;
      if (canvas.height !== pixelHeight) canvas.height = pixelHeight;
      if (typeof context.setTransform === 'function') context.setTransform(pixelRatio, 0, 0, pixelRatio, 0, 0);
      else if (typeof context.resetTransform === 'function') context.resetTransform();
      context.clearRect(0, 0, width, height);

      const left = 42, right = 12, top = 25, bottom = 24;
      const plotWidth = Math.max(1, width - left - right);
      const plotHeight = Math.max(1, height - top - bottom);
      const visible = samples.filter(sample => sample.at >= lastNowMs - windowMs && sample.at <= lastNowMs);
      let maximum = 0;
      for (const sample of visible) {
        if (!sample.wheels) continue;
        for (const value of sample.wheels) if (Number.isFinite(value)) maximum = Math.max(maximum, Math.abs(value));
      }
      const scale = Math.max(50, Math.ceil(maximum / 50) * 50);
      const yFor = value => top + (scale - value) / (scale * 2) * plotHeight;

      context.font = '10px sans-serif';
      context.textBaseline = 'middle';
      context.textAlign = 'left';
      context.strokeStyle = theme.grid;
      context.lineWidth = 1;
      for (let index = 0; index <= 4; index++) {
        const fraction = index / 4;
        const y = top + fraction * plotHeight;
        const value = scale - fraction * scale * 2;
        context.beginPath();
        context.moveTo(left, y);
        context.lineTo(width - right, y);
        context.stroke();
        context.fillStyle = theme.axis;
        context.fillText(formatValue(value), 3, y);
      }
      for (let index = 0; index <= 4; index++) {
        const x = left + index / 4 * plotWidth;
        context.beginPath();
        context.moveTo(x, top);
        context.lineTo(x, height - bottom);
        context.stroke();
        context.fillStyle = theme.axis;
        context.textAlign = index === 0 ? 'left' : index === 4 ? 'right' : 'center';
        context.fillText(index === 4 ? '现在' : axisWords(windowMs / 1000 * (1 - index / 4)), x, height - 10);
      }
      context.textAlign = 'left';
      for (let wheel = 0; wheel < 4; wheel++) {
        context.fillStyle = theme.lines[wheel];
        const names = ['A', 'B', 'C', 'D'];
        context.fillText(names[wheel], left + wheel * 48, 12);
      }

      for (let wheel = 0; wheel < 4; wheel++) {
        let drawing = false;
        context.beginPath();
        for (const sample of visible) {
          const value = sample.wheels && sample.wheels[wheel];
          if (!Number.isFinite(value)) { drawing = false; continue; }
          const x = left + Math.max(0, Math.min(1, (sample.at - (lastNowMs - windowMs)) / windowMs)) * plotWidth;
          const y = yFor(value);
          if (drawing) context.lineTo(x, y);
          else context.moveTo(x, y);
          drawing = true;
        }
        context.strokeStyle = theme.lines[wheel];
        context.lineWidth = 1.7;
        context.stroke();
      }
    }

    function resize() {
      drawTrend();
    }

    function updateHeader(snapshot) {
      const source = snapshot.source;
      const transportText = typeof snapshot.transportText === 'string' && snapshot.transportText.trim()
        ? snapshot.transportText : source === 'network' ? '网络 UDP' : '未知';
      setText(headers.transport, transportText);
      const connectionText = typeof snapshot.connectionText === 'string' && snapshot.connectionText.trim()
        ? snapshot.connectionText : snapshot.connected === true ? '已连接' : snapshot.connected === false ? '已断开' : '未知';
      setText(headers.connection, connectionText);
      setText(headers.simulation, snapshot.simulation === true ? '软件模拟' : snapshot.simulation === false ? '非模拟' : '模拟状态未知');
      if (headers.simulation && headers.simulation.dataset)
        headers.simulation.dataset.state = snapshot.simulation === true ? 'simulation' : snapshot.simulation === false ? 'live' : 'unknown';
      if (headers.connection && headers.connection.dataset)
        headers.connection.dataset.state = snapshot.connected === true ? 'fresh' : 'stale';
      setText(headers.actualMode, typeof snapshot.actualModeText === 'string' && snapshot.actualModeText.trim()
        ? snapshot.actualModeText : '未知');
    }

    function update(snapshot = {}) {
      const state = snapshot && typeof snapshot === 'object' ? snapshot : {};
      updateHeader(state);
      const nowMs = Number.isFinite(state.nowMs) ? state.nowMs : lastNowMs;
      lastNowMs = nowMs;
      const source = state.source === 'network' ? 'network' : null;
      if (sourceInitialized && source !== lastSource) clearTrend(false);
      sourceInitialized = true;
      lastSource = source;

      const sampleAt = Number.isFinite(state.sampleAt) ? state.sampleAt :
        state.sampleAt == null && Number.isFinite(state.nowMs) ? state.nowMs : null;
      const wheels = wheelValues(state.wheels);
      const age = Number.isFinite(sampleAt) && Number.isFinite(nowMs) ? nowMs - sampleAt : Infinity;
      const hasWheel = !!(wheels && wheels.some(Number.isFinite));
      const eligible = source !== null && state.connected === true && state.fresh === true && hasWheel &&
        Number.isFinite(sampleAt) && age >= 0 && age <= PRESENTATION_STALE_MS;

      if (!eligible) {
        markGap(nowMs);
        lastStatus = state.connected === false || state.fresh === false || age > PRESENTATION_STALE_MS
          ? `轮速趋势 · 连接或轮速数据已过期 · mm/s · 最近 ${windowWords(windowMs)}`
          : `轮速趋势 · 等待新鲜轮速 · mm/s · 最近 ${windowWords(windowMs)}`;
      } else if (sampleAt !== lastSampleAt) {
        const ordered = lastSampleAt === null || sampleAt >= lastSampleAt;
        if (ordered) {
          if (lastSampleAt !== null && sampleAt - lastSampleAt > PRESENTATION_STALE_MS) markGap(sampleAt);
          samples.push({at:sampleAt, wheels});
          lastSampleAt = sampleAt;
          inGap = !wheels.some(Number.isFinite);
          const names = ['A', 'B', 'C', 'D'];
          lastStatus = `轮速趋势 · mm/s · 最近 ${windowWords(windowMs)} · ${names.map((name, index) => `${name} ${formatValue(wheels[index])}`).join(' / ')}`;
        } else {
          markGap(nowMs);
          lastStatus = `轮速趋势 · 接收时间回退，等待新鲜轮速 · mm/s · 最近 ${windowWords(windowMs)}`;
        }
      }

      prune(nowMs);
      setText(trendStatus, lastStatus);
      resize();
    }

    function reset() {
      clearTrend(true);
      lastStatus = `轮速趋势 · 等待新鲜轮速 · mm/s · 最近 ${windowWords(windowMs)}`;
      setText(trendStatus, lastStatus);
      resize();
    }

    // 新页面时间窗按钮：只改变展示窗口，不改任何采样/协议语义。
    function setWindow(nextMs) {
      const value = Number(nextMs);
      if (!Number.isFinite(value) || value < 1000) return false;
      windowMs = Math.round(value);
      prune(lastNowMs);
      lastStatus = lastStatus.replace(/最近 [^·]+/, `最近 ${windowWords(windowMs)}`);
      setText(trendStatus, lastStatus);
      resize();
      return true;
    }

    const ResizeObserverImpl = win && win.ResizeObserver ||
      (typeof ResizeObserver === 'function' ? ResizeObserver : null);
    let observer = null;
    if (typeof ResizeObserverImpl === 'function') {
      const observedSizes = new WeakMap();
      try {
        observer = new ResizeObserverImpl(entries => {
          let changed = false;
          for (const entry of entries || []) {
            const rect = entry.contentRect || {};
            const width = Number.isFinite(rect.width) ? rect.width : 0;
            const height = Number.isFinite(rect.height) ? rect.height : 0;
            const previous = observedSizes.get(entry.target);
            if (!previous || previous.width !== width || previous.height !== height) {
              observedSizes.set(entry.target, {width, height});
              changed = true;
            }
          }
          if (!changed) return;
          resize();
          if (redraw) redraw();
        });
        const targets = new Set();
        for (const element of [
          doc && doc.getElementById('radarCloud'),
          canvas
        ]) {
          const target = element && (element.parentElement || element);
          if (target) targets.add(target);
        }
        for (const target of targets) observer.observe(target);
      } catch (_) {
        observer = null;
      }
    }

    resize();
    return Object.freeze({update, reset, resize, setWindow});
  }

  // Match object-fit: contain, including letterboxing in the browser viewer.
  function visionRect(viewWidth, viewHeight, imageWidth, imageHeight, box) {
    if (![viewWidth,viewHeight,imageWidth,imageHeight].every(v => Number.isFinite(v) && v > 0) ||
        !box || !['cx','cy','width','height'].every(k => Number.isFinite(box[k])) ||
        box.width <= 0 || box.height <= 0) return null;
    const scale = Math.min(viewWidth/imageWidth, viewHeight/imageHeight);
    return {x:(viewWidth-imageWidth*scale)/2+(box.cx-box.width/2)*scale,
      y:(viewHeight-imageHeight*scale)/2+(box.cy-box.height/2)*scale,
      width:box.width*scale, height:box.height*scale};
  }

  return Object.freeze({
    uint32IsNewer,
    poseUncertainty,
    DashboardNetworkAdapter,
    visionRect,
    createWorkbenchPresentation
  });
});
