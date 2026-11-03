import crypto from 'node:crypto';
import dgram from 'node:dgram';
import dns from 'node:dns/promises';
import fs from 'node:fs';
import http from 'node:http';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { performance } from 'node:perf_hooks';
import { WebSocketServer, WebSocket } from 'ws';
import { FpgaGridMap } from './fpga_grid_map.mjs';
import { parseSemanticCommand } from './semantic_command.mjs';

const HERE = path.dirname(fileURLToPath(import.meta.url));
const PUBLIC_DIR = path.join(HERE, 'public');
const ASSETS_DIR = path.join(HERE, 'assets');
const HTML_FILE = path.join(PUBLIC_DIR, 'index.html');
const SHARED_GRID_CODEC = path.join(HERE, '../ugv_ws/src/car/car_navigation/scripts/global_planner/public/js/grid_codec.js');
const STREAMS = new Set(['imu', 'motor', 'lidar', 'fusion', 'health', 'navigation', 'local_plan', 'local_grid', 'navigation_task']);
const MISSION_ACTIONS = new Set(['enable_ros', 'disable_ros', 'start', 'stop', 'emergency_stop', 'release_emergency_stop',
  'local_start', 'local_stop', 'local_cancel']);
// 静态路由白名单：/、/css/*.css、/js/*.js、/assets/*；其余一律 404。
const STATIC_TYPES = {
  '.html': 'text/html; charset=utf-8',
  '.css': 'text/css; charset=utf-8',
  '.js': 'text/javascript; charset=utf-8',
  '.jpg': 'image/jpeg',
  '.jpeg': 'image/jpeg',
  '.png': 'image/png',
  '.svg': 'image/svg+xml',
  '.gif': 'image/gif',
  '.webp': 'image/webp',
  '.ico': 'image/x-icon',
};
const STATIC_NAME = /^[A-Za-z0-9_][A-Za-z0-9._-]*$/;

function staticFile(pathname) {
  if (pathname === '/') return { file: HTML_FILE, type: STATIC_TYPES['.html'] };
  const match = /^\/(css|js|assets)\/([^/]+)$/.exec(pathname);
  if (!match) return null;
  const [, directory, name] = match;
  if (!STATIC_NAME.test(name)) return null;
  const extension = path.extname(name).toLowerCase();
  if (directory === 'css' && extension !== '.css') return null;
  if (directory === 'js' && extension !== '.js') return null;
  const type = STATIC_TYPES[extension];
  if (!type) return null;
  if (directory === 'js' && name === 'grid_codec.js') return { file: SHARED_GRID_CODEC, type };
  const root = directory === 'assets' ? ASSETS_DIR : path.join(PUBLIC_DIR, directory);
  return { file: path.join(root, name), type };
}
const U32 = 0x1_0000_0000;
const MAX_JPEG = 512 * 1024;
const JPEG_CHUNK = 1200;
const MAX_JPEG_PARTS = Math.ceil(MAX_JPEG / JPEG_CHUNK);
const MAX_DATAGRAM = 1400;
const CLIENT_BATCH_BYTES = 128 * 1024;
const CLIENT_BATCH_INTERVAL_MS = 100;
const CLIENT_DELIVERY_TIMEOUT_MS = 2000;

function monoMs() { return performance.now(); }
function u32(value) { return Number.isInteger(value) && value >= 0 && value < U32; }
function newer(candidate, previous) {
  const delta = (candidate - previous + U32) % U32;
  return delta > 0 && delta < U32 / 2;
}
function clamp(value, low, high) { return Math.min(high, Math.max(low, value)); }
function isRecord(value) { return value !== null && typeof value === 'object' && !Array.isArray(value); }
function sessionId() { return crypto.randomBytes(16).toString('hex'); }
function validVisionBox(box, width, height) {
  return isRecord(box) && typeof box.id === 'string' && box.id.length > 0 && box.id.length <= 128 &&
    typeof box.class_id === 'string' && box.class_id.length <= 32 &&
    ['score', 'cx', 'cy', 'width', 'height'].every(key => Number.isFinite(box[key])) &&
    box.score >= 0 && box.score <= 1 && box.cx >= 0 && box.cx <= width && box.cy >= 0 && box.cy <= height &&
    box.width > 0 && box.width <= width && box.height > 0 && box.height <= height;
}
function makeEvent(event, reason, extra = {}) {
  return { v: 1, type: 'event', event, reason, ...extra };
}

function mergeObjects(target, source) {
  for (const [key, value] of Object.entries(source)) {
    if (Array.isArray(value) && Array.isArray(target[key])) target[key] = [...target[key], ...value];
    else if (isRecord(value) && isRecord(target[key])) mergeObjects(target[key], value);
    else target[key] = value;
  }
  return target;
}

// FPGA grid fragments may split fields across packets, but any repeated scalar
// metadata must agree. Only arrays are concatenated in fragment order.
function mergeGridFragment(target, source) {
  for (const [key, value] of Object.entries(source)) {
    if (!(key in target)) {
      if (Array.isArray(value)) target[key] = value.slice();
      else if (isRecord(value)) {
        target[key] = {};
        if (!mergeGridFragment(target[key], value)) return false;
      } else target[key] = value;
    } else if (Array.isArray(value) && Array.isArray(target[key])) {
      target[key].push(...value);
    } else if (isRecord(value) && isRecord(target[key])) {
      if (!mergeGridFragment(target[key], value)) return false;
    } else if (target[key] !== value) return false;
  }
  return true;
}

class RollingJsonlLog {
  constructor(directory, { maxBytes = 4 * 1024 * 1024, backups = 3, maxQueue = 512 } = {}) {
    this.directory = directory;
    this.maxBytes = maxBytes;
    this.backups = backups;
    this.maxQueue = maxQueue;
    this.file = path.join(directory, 'dashboard.jsonl');
    this.size = 0;
    this.queue = [];
    this.dropped = 0;
    this.writing = false;
    this.ready = fs.promises.mkdir(directory, { recursive: true })
      .then(() => fs.promises.stat(this.file).then(info => { this.size = info.size; }, error => { if (error.code !== 'ENOENT') throw error; }))
      .catch(error => { process.stderr.write(`dashboard log initialize failed: ${error.message}\n`); });
  }

  write(record) {
    if (this.queue.length >= this.maxQueue) {
      this.dropped += 1;
      return;
    }
    const value = { wall_ms: Date.now(), mono_ms: monoMs(), ...record };
    if (this.dropped) {
      value.log_queue_dropped = this.dropped;
      this.dropped = 0;
    }
    this.queue.push(`${JSON.stringify(value)}\n`);
    void this.drain();
  }

  async rotate() {
    for (let index = this.backups - 1; index >= 1; index -= 1) {
      try { await fs.promises.rename(`${this.file}.${index}`, `${this.file}.${index + 1}`); }
      catch (error) { if (error.code !== 'ENOENT') throw error; }
    }
    try { await fs.promises.rename(this.file, `${this.file}.1`); }
    catch (error) { if (error.code !== 'ENOENT') throw error; }
    this.size = 0;
  }

  async drain() {
    if (this.writing) return;
    this.writing = true;
    try {
      await this.ready;
      while (this.queue.length) {
        const line = this.queue.shift();
        const bytes = Buffer.byteLength(line);
        if (this.size + bytes > this.maxBytes) await this.rotate();
        await fs.promises.appendFile(this.file, line, 'utf8');
        this.size += bytes;
      }
    } catch (error) {
      process.stderr.write(`dashboard log write failed: ${error.message}\n`);
      this.dropped += this.queue.length;
      this.queue.length = 0;
    } finally {
      this.writing = false;
      if (this.queue.length) void this.drain();
    }
  }

  async flush() {
    while (this.writing || this.queue.length) {
      await this.drain();
      if (this.writing || this.queue.length) await new Promise(resolve => setTimeout(resolve, 5));
    }
  }
}

export class JpegAssembler {
  constructor({ maxFrames = 3, timeoutMs = 250 } = {}) {
    this.maxFrames = maxFrames;
    this.timeoutMs = timeoutMs;
    this.frames = new Map();
    this.lastDelivered = null;
    this.dropped = 0;
    this.duplicates = 0;
    this.completed = 0;
  }

  prune(now = monoMs()) {
    for (const [id, frame] of this.frames) {
      if (now - frame.firstAt > this.timeoutMs) {
        this.frames.delete(id);
        this.dropped += 1;
      }
    }
  }

  feed(packet, now = monoMs()) {
    this.prune(now);
    if (!Buffer.isBuffer(packet) || packet.length <= 12 || packet.length > 12 + JPEG_CHUNK ||
        packet.toString('ascii', 0, 4) !== 'BXJV') return null;
    const id = packet.readUInt32BE(4);
    const index = packet.readUInt16BE(8);
    const count = packet.readUInt16BE(10);
    const payload = packet.subarray(12);
    if (count < 1 || count > MAX_JPEG_PARTS || index >= count || payload.length > JPEG_CHUNK) return null;
    if (this.lastDelivered !== null && id === this.lastDelivered) {
      this.duplicates += 1;
      return null;
    }
    if (this.lastDelivered !== null && !newer(id, this.lastDelivered)) return null;

    let frame = this.frames.get(id);
    if (!frame) {
      if (this.frames.size >= this.maxFrames) {
        const oldest = this.frames.keys().next().value;
        this.frames.delete(oldest);
        this.dropped += 1;
      }
      frame = { id, count, firstAt: now, parts: new Map(), bytes: 0 };
      this.frames.set(id, frame);
    }
    if (frame.count !== count) {
      this.frames.delete(id);
      this.dropped += 1;
      return null;
    }
    const previous = frame.parts.get(index);
    if (previous) {
      if (previous.equals(payload)) this.duplicates += 1;
      else { this.frames.delete(id); this.dropped += 1; }
      return null;
    }
    frame.parts.set(index, Buffer.from(payload));
    frame.bytes += payload.length;
    if (frame.bytes > MAX_JPEG) {
      this.frames.delete(id);
      this.dropped += 1;
      return null;
    }
    if (frame.parts.size !== count) return null;

    const jpeg = Buffer.concat(Array.from({ length: count }, (_, part) => frame.parts.get(part)));
    this.frames.delete(id);
    if (!jpeg.length || jpeg.length > MAX_JPEG || jpeg[0] !== 0xff || jpeg[1] !== 0xd8 ||
        jpeg.at(-2) !== 0xff || jpeg.at(-1) !== 0xd9) {
      this.dropped += 1;
      return null;
    }
    if (this.lastDelivered !== null && !newer(id, this.lastDelivered)) {
      this.dropped += 1;
      return null;
    }
    this.lastDelivered = id;
    this.completed += 1;
    for (const [pendingId] of this.frames) {
      if (pendingId !== id && !newer(pendingId, id)) this.frames.delete(pendingId);
    }
    return { id, jpeg };
  }
}

function validHost(host) {
  return typeof host === 'string' && host.length > 0 && host.length <= 253 &&
    !/[\s\u0000-\u001f/\\]/.test(host);
}

function validOrigin(value, host) {
  if (typeof value !== 'string') return false;
  try {
    const url = new URL(value);
    return (url.protocol === 'http:' || url.protocol === 'https:') &&
      (['localhost', '127.0.0.1', '[::1]', '::1'].includes(url.hostname) || url.host === host);
  } catch { return false; }
}

export class DashboardBridge {
  constructor(options = {}) {
    this.options = {
      bindHost: '127.0.0.1',
      httpPort: 8765,
      udpBindHost: '0.0.0.0',
      jetsonHost: '127.0.0.1',
      jetsonPort: 6200,
      globalMapPort: 8766,
      simulation: false,
      speed: 400,
      turn: 800,
      linkTimeoutMs: 500,
      maxClientQueueBytes: 1024 * 1024,
      maxVideoQueueBytes: 512 * 1024,
      logDirectory: path.join(HERE, 'logs'),
      ...options,
    };
    this.options.speed = clamp(Number(this.options.speed), 10, 400);
    this.options.turn = clamp(Number(this.options.turn), 0, 800);
    if (!Number.isInteger(this.options.globalMapPort) || this.options.globalMapPort < 1 || this.options.globalMapPort > 65535)
      throw new Error('global map port is invalid');
    this.log = options.log === false ? null : new RollingJsonlLog(this.options.logDirectory, options.logOptions);
    this.udp = dgram.createSocket('udp4');
    this.udp.on('message', (message, remote) => this.onUdpMessage(message, remote));
    this.udp.on('error', error => this.emitEvent('udp_error', error.message));
    this.http = http.createServer((request, response) => this.onHttp(request, response));
    this.webSockets = new WebSocketServer({ noServer: true, maxPayload: 16 * 1024, perMessageDeflate: false });
    this.webSockets.on('connection', (socket, request, streamName) => this.onWebSocket(socket, request, streamName));
    this.http.on('upgrade', (request, socket, head) => this.onUpgrade(request, socket, head));
    this.clients = new Map();
    this.events = [];
    this.nextEventId = 1;
    this.target = null;
    this.simulationTarget = null;
    this.session = sessionId();
    this.seq = 0;
    this.lastSendAt = 0;
    this.owner = null;
    this.pendingArm = null;
    this.stopRequest = null;
    this.lastDriveAt = null;
    this.lastDriveSeq = null;
    this.commandExpiryTimer = null;
    this.velocity = [0, 0, 0];
    this.autoRequested = false;
    this.handshakeSent = false;
    this.startedAt = null;
    this.lastJetsonRxAt = null;
    this.lastStatusAt = null;
    this.status = null;
    this.statusSequence = null;
    this.bridgeId = null;
    this.retiredBridgeIds = new Set();
    this.configureGeneration = 0;
    this.authorizationGeneration = 0;
    this.bridgeEpoch = null;
    this.telemetrySequences = new Map();
    this.telemetryLatest = new Map();
    this.partials = new Map();
    this.jpeg = new JpegAssembler();
    this.videoAt = null;
    this.videoTimes = [];
    this.videoDropped = 0;
    this.videoWasStale = true;
    this.videoFrameLatest = null;
    this.online = false;
    this.timers = [];
    this.stopping = false;
    this.rejections = 0;
    this.lastLoggedControlKey = null;
    this.vision = null;
    this.visionAt = null;
    this.visionSequence = null;
    this.pendingMission = new Map();
    this.missionOwner = null;
    this.lastNavigationLogKey = null;
    this.mapping = new FpgaGridMap();
    // 全局路线和任务调度属于 Jetson；这里只转发请求和展示实际状态。
    this.navigationTask = null;
    this.navigationTaskAt = null;
    this.pendingNavigation = new Map();
  }

  async resolveTarget(host, port) {
    if (!validHost(host) || !Number.isInteger(port) || port < 1 || port > 65535) {
      throw new Error('jetson host/port is invalid');
    }
    const resolved = await dns.lookup(host, { family: 4 });
    return { host, address: resolved.address, port };
  }

  async start() {
    this.target = await this.resolveTarget(this.options.jetsonHost, this.options.jetsonPort);
    if (this.options.simulation) this.simulationTarget = { address: this.target.address, port: this.target.port };
    await new Promise((resolve, reject) => {
      const onError = error => { this.udp.off('listening', onListening); reject(error); };
      const onListening = () => { this.udp.off('error', onError); resolve(); };
      this.udp.once('error', onError);
      this.udp.once('listening', onListening);
      this.udp.bind(0, this.options.udpBindHost);
    });
    await new Promise((resolve, reject) => {
      this.http.once('error', reject);
      this.http.listen(this.options.httpPort, this.options.bindHost, () => {
        this.http.off('error', reject);
        resolve();
      });
    });
    this.startedAt = monoMs();
    this.sendControl([0, 0, 0], false, 'initial-zero-handshake');
    this.handshakeSent = true;
    this.timers.push(setInterval(() => this.tick(), 25));
    this.timers.push(setInterval(() => this.publishState(), 100));
    this.timers.push(setInterval(() => this.flushClients(), 25));
    this.timers.push(setInterval(() => this.jpeg.prune(), 50));
    // 规划段在500ms失效，地图推送须留出传输和绘制时间。
    this.timers.push(setInterval(() => this.publishMap(), 200));
    return this.http.address();
  }

  async close() {
    if (this.stopping) return;
    if (this.missionOwner) this.sendMissionCommand(null, 'disable_ros');
    this.stopping = true;
    for (const timer of this.timers) clearInterval(timer);
    this.timers = [];
    this.disarm('server-closed', { log: true });
    for (let attempt = 0; attempt < 3; attempt += 1) this.sendControl([0, 0, 0], false, 'server-close-zero');
    for (const socket of this.clients.keys()) socket.close(1001, 'server stopping');
    await this.log?.flush();
    await new Promise(resolve => this.http.close(() => resolve()));
    await new Promise(resolve => this.webSockets.close(() => resolve()));
    if (this.udp.address()) await new Promise(resolve => this.udp.close(() => resolve()));
  }

  onHttp(request, response) {
    const pathname = new URL(request.url, 'http://127.0.0.1').pathname;
    if (request.method !== 'GET') {
      response.writeHead(405, { Allow: 'GET', 'Content-Type': 'text/plain; charset=utf-8' }).end('Method Not Allowed');
      return;
    }
    if (pathname === '/map.json') {
      response.writeHead(200, {'Content-Type':'application/json; charset=utf-8',
        'Cache-Control':'no-store', 'Content-Disposition':'attachment; filename="bx72-map.json"'})
        .end(JSON.stringify(this.mapping.snapshot(monoMs())));
      return;
    }
    if (pathname === '/global-map.json' || pathname === '/global-map-meta.json') {
      this.proxyGlobalMap(response, pathname);
      return;
    }
    const asset = staticFile(pathname);
    if (!asset) { response.writeHead(404, { 'Content-Type': 'text/plain; charset=utf-8' }).end('Not Found'); return; }
    fs.readFile(asset.file, (error, data) => {
      if (error) { response.writeHead(404, { 'Content-Type': 'text/plain; charset=utf-8' }).end('Not Found'); return; }
      response.writeHead(200, {
        'Content-Type': asset.type,
        'Cache-Control': 'no-store',
        'X-Content-Type-Options': 'nosniff',
      }).end(data);
    });
  }

  readGlobalMap(pathname) {
    const target = this.target;
    return new Promise((resolve, reject) => {
      if (!target) { reject(new Error('尚未连接全局地图服务')); return; }
      const upstream = http.get({hostname:target.address, port:this.options.globalMapPort,
        path:pathname, timeout:3000}, incoming => {
        if (incoming.statusCode !== 200) {
          incoming.resume(); reject(new Error('Jetson 全局地图服务不可用')); return;
        }
        const chunks = []; let size = 0;
        incoming.on('data', chunk => {
          size += chunk.length;
          if (size > (pathname === '/global-map-meta.json' ? 16 * 1024 : 8 * 1024 * 1024))
            incoming.destroy(new Error('全局地图超过接口容量'));
          else chunks.push(chunk);
        });
        incoming.on('end', () => {
          if (this.target !== target) reject(new Error('地图来源已改变'));
          else resolve(Buffer.concat(chunks));
        });
        incoming.on('error', reject);
        incoming.on('aborted', () => reject(new Error('全局地图传输中断')));
      });
      upstream.on('timeout', () => upstream.destroy(new Error('全局地图读取超时')));
      upstream.on('error', reject);
    });
  }

  async proxyGlobalMap(response, pathname = '/global-map.json') {
    try {
      const bytes = await this.readGlobalMap(pathname);
      if (!response.destroyed)
        response.writeHead(200, {'Content-Type':'application/json; charset=utf-8', 'Cache-Control':'no-store',
          'X-Content-Type-Options':'nosniff'}).end(bytes);
    } catch (_) {
      if (!response.headersSent && !response.destroyed)
        response.writeHead(502, {'Content-Type':'application/json; charset=utf-8', 'Cache-Control':'no-store'})
          .end(JSON.stringify({error:'Jetson 全局地图服务不可用'}));
    }
  }

  sendNavigationTask(socket, message, {replyAction = `navigation_${message.action}`, fallbackOnFailure = true} = {}) {
    const action = message.action;
    if (!['start', 'stop', 'cancel'].includes(action)) return this.reject('navigation-task-action-invalid');
    const fail = text => {
      this.deliverMissionResult({v:1,type:'mission_result',action:replyAction,
        success:false,unknown:false,message:text}, socket);
      if (fallbackOnFailure && action !== 'start')
        this.sendMissionCommand(null,'stop',{}, {skipNavigation:true});
    };
    if (!this.target) return fail('尚未设置 Jetson 网桥地址');
    try { this.udp.address(); } catch { return fail('本机 UDP 网络服务尚未就绪'); }
    if (action === 'start' && this.owner?.socket === socket) this.disarm('switch-to-navigation');
    const request_id = sessionId();
    this.seq = (this.seq + 1) >>> 0;
    const request = {...message, v:1, type:'navigation_task', session:this.session, seq:this.seq, request_id};
    delete request.claim_nonce;
    if (this.status?.session === this.session && typeof this.status.claim_nonce === 'string')
      request.claim_nonce = this.status.claim_nonce;
    const bytes = Buffer.from(JSON.stringify(request));
    // 网桥的目标入口限 16 KiB；超出时明确失败，不丢弃后续航点。
    if (bytes.length > 16 * 1024) return fail('航点请求超出网桥单包容量，请减少本次航点数量');
    this.pendingNavigation.set(request_id,{socket,action,replyAction,fallbackOnFailure,session:this.session,at:monoMs()});
    this.lastSendAt = monoMs();
    this.udp.send(bytes,this.target.port,this.target.address,error => {
      if (!error) return;
      this.pendingNavigation.delete(request_id);
      fail(`导航请求发送失败：${error.message}`);
    });
    return request_id;
  }

  acceptNavigationTaskResult(result) {
    const pending = this.pendingNavigation.get(result.request_id);
    if (!pending || result.session !== this.session || result.session !== pending.session ||
        typeof result.bridge_id !== 'string' || !result.bridge_id || result.bridge_id.length > 128 ||
        this.retiredBridgeIds.has(result.bridge_id) || (this.bridgeId && result.bridge_id !== this.bridgeId) ||
        result.action !== pending.action || typeof result.success !== 'boolean' ||
        typeof result.message !== 'string' ||
        (result.unknown !== undefined && typeof result.unknown !== 'boolean')) return;
    // 来源已在 UDP 入口校验；首次匹配的实际结果也可以确认网桥身份。
    if (!this.bridgeId) this.bridgeId = result.bridge_id;
    this.pendingNavigation.delete(result.request_id);
    if (pending.fallbackOnFailure && pending.action !== 'start' && (!result.success || result.unknown === true))
      this.sendMissionCommand(null,'stop',{}, {skipNavigation:true});
    this.deliverMissionResult({v:1,type:'mission_result',request_id:result.request_id,
      action:pending.replyAction ?? `navigation_${pending.action}`,
      success:result.success,unknown:result.unknown === true,
      message:result.message},pending.socket);
  }

  onUpgrade(request, socket, head) {
    const pathname = new URL(request.url, 'http://127.0.0.1').pathname;
    if (!['/ws', '/video', '/map'].includes(pathname) || !validOrigin(request.headers.origin, request.headers.host)) {
      socket.write('HTTP/1.1 403 Forbidden\r\nConnection: close\r\n\r\n');
      socket.destroy();
      return;
    }
    this.webSockets.handleUpgrade(request, socket, head, ws => {
      this.webSockets.emit('connection', ws, request, pathname === '/ws' ? 'json' : pathname.slice(1));
    });
  }

  onWebSocket(socket, request, streamName) {
    if (streamName === 'map') {
      this.clients.set(socket, {id:crypto.randomUUID(), kind:'map',
        pendingMap:JSON.stringify(this.mapping.snapshot(monoMs()))});
      socket.on('close', () => this.onClientClose(socket));
      socket.on('error', () => this.onClientClose(socket));
      return;
    }
    if (streamName === 'video') {
      this.clients.set(socket, { id: crypto.randomUUID(), kind: 'video', events: [], pendingState: null, pendingTelemetry: new Map(), pendingVideo: null });
      socket.on('close', () => this.onClientClose(socket));
      socket.on('error', () => this.onClientClose(socket));
      return;
    }
    const id = crypto.randomUUID();
    const client = { id, kind: 'json', events: [], pendingState: null, pendingTelemetry: new Map(), pendingVideo: null, lastDriveSeq: null,
      deliveryAckEnabled:false, deliveryPending:null, deliveryAckAt:null, deliverySeq:0, lastBatchAt:null,
      backpressureSince:null };
    this.clients.set(socket, client);
    this.safeSend(socket, { v: 1, type: 'hello', client_id: id, simulation: Boolean(this.options.simulation),
      delivery_ack:1, limits: { speed: this.options.speed, turn: this.options.turn } });
    socket.on('message', (data, isBinary) => this.onClientMessage(socket, data, isBinary));
    socket.on('close', () => this.onClientClose(socket));
    socket.on('error', () => this.onClientClose(socket));
  }

  onClientClose(socket) {
    const client = this.clients.get(socket);
    if (!client) return;
    this.clients.delete(socket);
    if (this.missionOwner === socket) {
      this.missionOwner = null;
      if (!this.stopping) this.sendMissionCommand(null, 'disable_ros');
    }
    if (this.owner?.socket === socket) this.disarm('controller-disconnected', { log: true });
  }

  onClientMessage(socket, data, isBinary) {
    if (isBinary) return this.reject('browser-binary-message');
    let message;
    try { message = JSON.parse(data.toString('utf8')); } catch { return this.reject('browser-json-invalid'); }
    if (!isRecord(message) || message.v !== 1 || typeof message.type !== 'string') return this.reject('browser-message-invalid');
    const client = this.clients.get(socket);
    if (!client || client.kind !== 'json') return;
    switch (message.type) {
      case 'ping': return;
      case 'delivery_ready':
        client.deliveryAckEnabled = true;
        return;
      case 'delivery_ack': {
        const pending = client.deliveryPending;
        if (!pending || message.id !== pending.id) return;
        const now = monoMs();
        if (now - pending.at > CLIENT_DELIVERY_TIMEOUT_MS) {
          const blocked = socket.bufferedAmount >= Math.min(CLIENT_BATCH_BYTES, this.options.maxClientQueueBytes / 2);
          this.closeSlowClient(socket, blocked ? 'client transport stalled' : 'client consumption stalled');
          return;
        }
        client.deliveryPending = null;
        client.deliveryAckAt = now;
        return;
      }
      case 'semantic_command': {
        if (typeof message.text !== 'string') {
          this.safeSend(socket, {v:1,type:'semantic_result',accepted:false,error:'命令文本无效'});
          return;
        }
        let parsed;
        try { parsed = parseSemanticCommand(message.text); }
        catch (error) { this.safeSend(socket, {v:1,type:'semantic_result',accepted:false,error:error.message}); return; }
        if (parsed.type === 'stop') {
          this.onClientMessage(socket, Buffer.from(JSON.stringify({v:1,...parsed})), false);
          this.safeSend(socket, {v:1,type:'semantic_result',accepted:true});
          return;
        }
        if (parsed.type === 'local_task') {
          const domain = this.navigationDomain();
          if (!domain) {
            this.safeSend(socket, {v:1,type:'semantic_result',accepted:false,error:'缺少导航坐标域，无法构造目标'});
            return;
          }
          const request = this.sendNavigationTask(socket,{action:'start',frame:'odom',
            domain,generation:null,
            goals:[{x:parsed.goal_x_m,y:parsed.goal_y_m}],
            max_speed_mm_s:400,max_accel_mm_s2:200,tolerance_mm:100,yaw_tolerance_mrad:100,
            duration_s:120});
          this.safeSend(socket, {v:1,type:'semantic_result',accepted:!!request,...(request ? {request_id:request} : {error:'导航目标未发送'})});
          return;
        }
        const request = this.sendMissionCommand(socket,parsed.action,parsed);
        this.safeSend(socket, {v:1,type:'semantic_result',accepted:!!request,...(request ? {request_id:request} : {error:'视觉任务未发送'})});
        return;
      }
      case 'map_reset':
        this.sendEvent(socket, 'map_reset_rejected', 'fpga-map-reset-unavailable');
        return;
      case 'configure':
        if (!this.clientConsumptionReady(socket)) {
          this.sendEvent(socket, 'configure_rejected', 'page-consumption-unconfirmed');
          return;
        }
        void this.configureTarget(socket, message);
        return;
      case 'arm': return this.arm(socket);
      case 'drive': return this.drive(socket, message);
      case 'auto': return this.setAuto(socket, message);
      case 'mission_command': return this.sendMissionCommand(socket, message.action, message);
      case 'navigation_task': return void this.sendNavigationTask(socket, message);
      case 'local_task': {
        const action = message.action === 'start' ? 'local_start' :
          message.action === 'stop' ? 'local_stop' :
          message.action === 'cancel' ? 'local_cancel' : null;
        if (!action) return this.reject('local-task-action-invalid');
        return this.sendMissionCommand(socket, action, message);
      }
      case 'stop': {
        this.disarm(typeof message.reason === 'string' ? message.reason.slice(0, 120) : 'browser-stop', { log: true });
        // 通用停车覆盖视觉、人工及非网页发起的设备任务；Jetson 全局协调器另接收取消。
        this.sendMissionCommand(socket, 'stop');
        return;
      }
      default: return this.reject(`browser-message-unknown:${message.type.slice(0, 40)}`);
    }
  }

  async configureTarget(socket, message) {
    if (this.owner) {
      this.sendEvent(socket, 'configure_rejected', 'release-control-before-changing-target');
      return;
    }
    const generation = ++this.configureGeneration;
    const authorizationGeneration = this.authorizationGeneration;
    let next;
    try { next = await this.resolveTarget(message.jetson, message.port); }
    catch (error) { this.sendEvent(socket, 'configure_rejected', error.message); return; }
    if (generation !== this.configureGeneration || authorizationGeneration !== this.authorizationGeneration || this.owner) {
      this.sendEvent(socket, 'configure_rejected', 'target-or-owner-changed-during-configuration');
      return;
    }
    if (this.options.simulation && (next.address !== this.simulationTarget?.address || next.port !== this.simulationTarget?.port)) {
      this.sendEvent(socket, 'configure_rejected', 'simulation-target-locked-to-startup-endpoint');
      return;
    }
    if (this.target?.address === next.address && this.target?.port === next.port) {
      this.sendEvent(socket, 'configured', 'target-unchanged', { jetson: { host: next.host, port: next.port } });
      return;
    }
    const previous = this.target;
    this.sendControl([0, 0, 0], false, 'target-change-old-zero', previous);
    this.rotateSession('target-changed', { sendOldZero: false, sendZero: false });
    this.target = next;
    this.resetRemoteCaches();
    this.sendControl([0, 0, 0], false, 'target-change-new-zero');
    this.handshakeSent = true;
    this.emitEvent('target_changed', 'target-changed', { host: next.host, port: next.port, previous_host: previous?.host ?? null, previous_port: previous?.port ?? null });
    this.sendEvent(socket, 'configured', 'target-changed', { jetson: { host: next.host, port: next.port } });
  }

  arm(socket) {
    if (this.navigationTask?.active) {
      this.sendEvent(socket, 'arm_rejected', '请先停止全局导航');
      return;
    }
    if (this.owner) {
      this.sendEvent(socket, 'arm_rejected', 'another-page-owns-control');
      return;
    }
    if (!this.fpgaReady() || !this.jetsonOnline() || !this.handshakeSent) {
      this.sendEvent(socket, 'arm_rejected', 'jetson-or-fpga-not-ready');
      return;
    }
    const client = this.clients.get(socket);
    if (!client || client.kind !== 'json') return;
    if (!this.clientConsumptionReady(socket)) {
      this.sendEvent(socket, 'arm_rejected', 'page-consumption-unconfirmed');
      return;
    }
    this.owner = { id: client.id, socket };
    this.missionOwner = null;
    this.stopRequest = null;
    this.authorizationGeneration += 1;
    this.lastDriveSeq = null;
    this.lastDriveAt = monoMs();
    this.scheduleCommandExpiry(socket);
    this.velocity = [0, 0, 0];
    this.autoRequested = false;
    const seq = this.sendControl([0, 0, 0], false, 'manual-zero-handshake');
    if (seq === null) return this.disarm('handshake-send-failed', { log: true });
    this.pendingArm = { session: this.session, seq, at: monoMs() };
  }

  drive(socket, message) {
    if (this.owner?.socket !== socket) return this.reject('drive-without-control');
    if (!u32(message.seq) || !['vx', 'vy', 'wz'].every(key => Number.isSafeInteger(message[key]))) return this.reject('drive-fields-invalid');
    if (Math.abs(message.vx) > 400 || Math.abs(message.vy) > 400 || Math.abs(message.wz) > 800) return this.reject('drive-over-hard-limit');
    if (this.pendingArm && (message.vx !== 0 || message.vy !== 0 || message.wz !== 0)) return this.reject('drive-handshake-pending');
    if (this.lastDriveSeq !== null && !newer(message.seq, this.lastDriveSeq)) return;
    this.lastDriveSeq = message.seq;
    this.lastDriveAt = monoMs();
    this.scheduleCommandExpiry(socket);
    if (this.autoRequested && (message.vx !== 0 || message.vy !== 0 || message.wz !== 0)) {
      return this.disarm('nonzero-drive-during-auto', { log: true });
    }
    const previousVelocity = this.velocity;
    const x = message.vx;
    const y = message.vy;
    const magnitude = Math.hypot(x, y);
    const scale = magnitude > this.options.speed ? this.options.speed / magnitude : 1;
    this.velocity = [Math.round(x * scale), Math.round(y * scale), clamp(message.wz, -this.options.turn, this.options.turn)];
    if (this.velocity.some((value, index) => value !== previousVelocity[index])) {
      this.emitEvent('drive_command', 'accepted-drive', { owner: this.owner.id, seq: message.seq, velocity: this.velocity, auto: this.autoRequested });
    }
  }

  setAuto(socket, message) {
    if (this.owner?.socket !== socket) return this.reject('auto-without-control');
    if (typeof message.enabled !== 'boolean') return this.reject('auto-value-invalid');
    if (message.enabled) {
      this.sendEvent(socket, 'auto_rejected', '无目标type5自动模式已停用；请在FPGA局部导航中提交odom目标');
      return;
    }
    this.velocity = [0, 0, 0];
    this.autoRequested = false;
    this.sendControl([0, 0, 0], false, 'legacy-auto-disabled-zero');
    this.emitEvent('auto_disabled', 'legacy-auto-path-disabled', { owner: this.owner.id });
  }

  disarm(reason, { log = false } = {}) {
    const hadOwner = Boolean(this.owner);
    const owner = this.owner?.id ?? null;
    this.clearAuthorization();
    const seq = this.sendControl([0, 0, 0], false, reason);
    this.stopRequest = seq === null ? null : { requested: true, seq, session: this.session };
    if (hadOwner || log) this.emitEvent('stop_requested', reason, { owner, session: this.session, seq });
  }

  clearAuthorization() {
    this.authorizationGeneration += 1;
    if (this.commandExpiryTimer) clearTimeout(this.commandExpiryTimer);
    this.commandExpiryTimer = null;
    this.owner = null;
    this.pendingArm = null;
    this.lastDriveAt = null;
    this.lastDriveSeq = null;
    this.velocity = [0, 0, 0];
    this.autoRequested = false;
  }

  scheduleCommandExpiry(socket) {
    if (this.commandExpiryTimer) clearTimeout(this.commandExpiryTimer);
    const expectedAt = this.lastDriveAt;
    const expire = () => {
      if (this.owner?.socket !== socket || this.lastDriveAt !== expectedAt) return;
      const remaining = 200 - (monoMs() - this.lastDriveAt);
      if (remaining > 0) {
        this.commandExpiryTimer = setTimeout(expire, remaining);
        return;
      }
      this.disarm('page-command-expired', { log: true });
    };
    this.commandExpiryTimer = setTimeout(expire, Math.max(0, 200 - (monoMs() - expectedAt)));
  }

  deliverMissionResult(result, socket) {
    if (result.success === false || result.unknown === true || result.action?.startsWith('local_')) {
      // 只记录诊断字段，不记录请求参数、会话令牌，也不广播私有结果。
      this.log?.write({type:'mission_result', action:result.action,
        request_id:typeof result.request_id === 'string' ? result.request_id : null,
        success:result.success, unknown:result.unknown === true, message:result.message,
        ...(typeof result.invalid_field === 'string' ? {invalid_field:result.invalid_field} : {}),
        ...Object.fromEntries(['task_id','transaction_task_id','active_task_id','cancelled_task_id']
          .filter(key => u32(result[key])).map(key => [key,result[key]]))});
    }
    if (socket && this.clients.has(socket)) this.safeSend(socket, result);
  }

  clientConsumptionReady(socket, now = monoMs()) {
    const client = this.clients.get(socket);
    return Boolean(client?.kind === 'json' && client.deliveryAckEnabled &&
      client.deliveryAckAt !== null && now - client.deliveryAckAt <= CLIENT_DELIVERY_TIMEOUT_MS &&
      (!client.deliveryPending || now - client.deliveryPending.at <= CLIENT_DELIVERY_TIMEOUT_MS));
  }

  bridgeEmergencyStop(now = monoMs()) {
    if (!this.status || this.lastStatusAt === null || now < this.lastStatusAt ||
        now - this.lastStatusAt > Math.min(500, this.options.linkTimeoutMs)) return null;
    if (this.status.emergency_stop === true || this.status.control_source === 'emergency_stop') return true;
    if (this.status.emergency_stop === false) return false;
    // The simulator alone uses its control source as its emergency state.
    if (this.options.simulation && ['none','pc','ros'].includes(this.status.control_source)) return false;
    return null;
  }

  navigationDomain() {
    const nav = this.telemetryLatest.get('navigation')?.data;
    return u32(nav?.boot_id) && nav.boot_id !== 0 && u32(nav?.odom_domain) && nav.odom_domain !== 0
      ? `${nav.boot_id}:${nav.odom_domain}` : null;
  }

  navigationMayBeRunning() {
    return this.navigationTask?.active === true ||
      [...this.pendingNavigation.values()].some(item => item.action === 'start');
  }

  sendLegacyLocalTask(socket, action, payload = {}) {
    if (action === 'local_start') {
      // 旧接口的协同断链、固定航向和终点朝向无法在全局接口无损表达。
      if (payload.mode !== 'independent' ||
          (payload.allow_disconnect !== undefined && payload.allow_disconnect !== true) ||
          payload.position_only !== true || payload.hold_yaw_rad !== undefined ||
          payload.final_yaw_rad !== undefined || (payload.frame !== undefined && payload.frame !== 'odom')) {
        this.deliverMissionResult({v:1,type:'mission_result',action,success:false,
          message:'旧局部任务包含不受支持的模式或航向；请使用 navigation_task 提交航点'}, socket);
        return null;
      }
      const domain = this.navigationDomain();
      if (!domain) {
        this.deliverMissionResult({v:1,type:'mission_result',action,success:false,
          message:'缺少导航坐标域，无法构造目标'}, socket);
        return null;
      }
      return this.sendNavigationTask(socket, {action:'start', frame:'odom',
        domain, generation:null,
        goals:[{x:payload.goal_x_m,y:payload.goal_y_m}],
        max_speed_mm_s:payload.max_speed_mm_s,max_accel_mm_s2:payload.max_accel_mm_s2,
        tolerance_mm:payload.tolerance_mm,yaw_tolerance_mrad:payload.yaw_tolerance_mrad,
        duration_s:payload.duration_s,allow_disconnect:true}, {replyAction:action});
    }
    const navigationAction = action === 'local_stop' ? 'stop' : 'cancel';
    if (this.navigationMayBeRunning()) {
      const request = this.sendNavigationTask(socket,{action:navigationAction},{replyAction:action});
      if (request && (this.navigationTaskAt === null || monoMs() - this.navigationTaskAt > 1500))
        this.sendMissionCommand(null,'stop',{}, {skipNavigation:true});
      return request;
    }
    // 状态未知或为其他来源任务时，也发送通用停车；不由网页构造局部事务。
    this.sendNavigationTask(null,{action:navigationAction},{fallbackOnFailure:false});
    return this.sendMissionCommand(socket,'stop',{}, {replyAction:action,skipNavigation:true});
  }

  sendMissionCommand(socket, action, payload = {}, {replyAction = action, skipNavigation = false} = {}) {
    if (!MISSION_ACTIONS.has(action)) return null;
    if (action.startsWith('local_')) return this.sendLegacyLocalTask(socket,action,payload);
    if (socket && ['start','enable_ros'].includes(action) && this.navigationTask?.active) {
      this.deliverMissionResult({v:1,type:'mission_result',action,success:false,
        message:'全局导航正在执行，请先停止当前任务'}, socket);
      return null;
    }
    // 即使全局状态尚未到达或启动结果未知，也要让协调器撤销正在排队的目标。
    if (!skipNavigation && socket && ['stop','emergency_stop'].includes(action))
      this.sendNavigationTask(null,{action:'stop'},{fallbackOnFailure:false});
    if (!this.target) {
      this.deliverMissionResult({v:1,type:'mission_result',action:replyAction,success:false,
        message:'尚未设置 Jetson 网桥地址，未发送请求'}, socket);
      return null;
    }
    try { this.udp.address(); } catch {
      this.deliverMissionResult({v:1,type:'mission_result',action:replyAction,success:false,
        message:'本机 UDP 网络服务尚未就绪，未发送请求'}, socket);
      return null;
    }
    const emergencyAction = action === 'emergency_stop' || action === 'release_emergency_stop';
    // A stale receive status must not prevent a best-effort stop packet. Its
    // result is still unknown unless the gateway replies.
    if (socket && !['stop','emergency_stop'].includes(action) && !this.jetsonOnline()) {
      this.deliverMissionResult({v:1,type:'mission_result',action,success:false,
        message:'Jetson 网络网桥当前不在线'}, socket);
      return null;
    }
    if (socket && !emergencyAction && action !== 'stop' && !this.vision) {
      this.deliverMissionResult({v:1,type:'mission_result',action,success:false,
        message:'网桥未提供视觉任务接口'}, socket);
      return null;
    }
    if (socket && action === 'start') {
      const emergency = this.bridgeEmergencyStop();
      if (emergency !== false) {
        this.deliverMissionResult({v:1,type:'mission_result',action,success:false,
          message:emergency ? '软件急停已激活，不能启动' : '网桥急停状态未知或已过期，不能启动'}, socket);
        return null;
      }
    }
    if (action === 'start') {
      const vision = this.visionState();
      if (vision?.stale) {
        this.deliverMissionResult({v:1,type:'mission_result',action,success:false,
          message:'视觉观测已过期，不能启动'}, socket);
        return null;
      }
      if (!['follow','approach','search','search_only'].includes(payload.behavior) ||
          typeof payload.class_id !== 'string' || !payload.class_id || payload.class_id.length > 32 ||
          !(payload.target_id === null || typeof payload.target_id === 'string' && payload.target_id.length <= 128) ||
          !['','red'].includes(payload.color)) {
        this.deliverMissionResult({v:1,type:'mission_result',action,success:false,message:'视觉目标或行为无效'}, socket);
        return null;
      }
      if (socket && !this.clientConsumptionReady(socket)) {
        this.deliverMissionResult({v:1,type:'mission_result',action,success:false,
          message:'页面尚未确认接收状态，不能启动任务'}, socket);
        return null;
      }
      if (this.owner) this.disarm('switch-to-ros');
    }
    if (socket && ['enable_ros','release_emergency_stop'].includes(action) && !this.clientConsumptionReady(socket)) {
      this.deliverMissionResult({v:1,type:'mission_result',action,success:false,
        message:'页面尚未确认接收状态，不能执行该请求'}, socket);
      return null;
    }
    if (action === 'enable_ros' && this.owner) this.disarm('switch-to-ros');
    if (this.pendingMission.size >= 8) {
      if (action !== 'emergency_stop' && action !== 'disable_ros' && action !== 'stop') {
        this.deliverMissionResult({v:1,type:'mission_result',action,success:false,
          message:'等待中的任务请求已满，未发送新请求'}, socket);
        return null;
      }
      this.clearPendingMission('安全操作取代等待中的请求，原结果未知');
    }
    const request_id = sessionId();
    this.seq = (this.seq + 1) >>> 0;
    const command = {v:1, type:'mission_command', session:this.session, seq:this.seq, request_id, action};
    if (this.status?.session === this.session && typeof this.status.claim_nonce === 'string')
      command.claim_nonce = this.status.claim_nonce;
    if (action === 'start') Object.assign(command, {target_id:payload.target_id,class_id:payload.class_id,behavior:payload.behavior,color:payload.color});
    this.pendingMission.set(request_id, {action, replyAction, session:this.session, at:monoMs(), socket});
    if (socket && (action === 'enable_ros' || action === 'start')) this.missionOwner = socket;
    this.lastSendAt = monoMs();
    this.udp.send(Buffer.from(JSON.stringify(command)), this.target.port, this.target.address, error => {
      if (error) this.emitEvent('udp_send_error', error.message, {action, request_id});
    });
    return request_id;
  }

  clearPendingMission(message) {
    const pendingRequests = [...this.pendingMission];
    this.pendingMission.clear();
    for (const [request_id, pending] of pendingRequests) {
      const result = {v:1, type:'mission_result', request_id, action:pending.action,
        success:false, unknown:true, message};
      this.deliverPendingMissionResult(result, pending);
    }
  }

  deliverPendingMissionResult(result, pending) {
    this.deliverMissionResult(pending.replyAction ? {...result, action:pending.replyAction} : result, pending.socket);
  }

  acceptMissionResult(result) {
    const pending = this.pendingMission.get(result.request_id);
    if (!pending || result.session !== this.session || result.session !== pending.session ||
        result.action !== pending.action || typeof result.success !== 'boolean' ||
        typeof result.message !== 'string' || result.message.length > 512 ||
        (result.unknown !== undefined && typeof result.unknown !== 'boolean')) return;
    this.pendingMission.delete(result.request_id);
    if (result.success && ['disable_ros', 'stop', 'emergency_stop'].includes(result.action)) this.missionOwner = null;
    this.deliverPendingMissionResult(result, pending);
  }

  acceptVision(message, rx) {
    if (message.session !== this.session || !this.bridgeId || message.bridge_id !== this.bridgeId ||
        !u32(message.seq) || (this.visionSequence !== null && !newer(message.seq, this.visionSequence)) ||
        !['UNKNOWN','IDLE','TRACKING','SEARCHING','FOUND','FOLLOWING','APPROACHING','ARRIVED','BLOCKED','LOST','STOPPED'].includes(message.mission_state) ||
        !['none','pc','ros','navigation','emergency_stop'].includes(message.control_source) ||
        typeof message.ros_authorized !== 'boolean' || typeof message.emergency_stop !== 'boolean' ||
        typeof message.stale !== 'boolean' || !Number.isFinite(message.age_ms) || message.age_ms < 0 ||
        !((message.width === 640 && message.height === 360) ||
          (message.width === 800 && message.height === 480)) || typeof message.frame_id !== 'string' ||
        message.frame_id.length > 128 || !isRecord(message.stamp) ||
        !Number.isInteger(message.stamp.sec) || message.stamp.sec < 0 ||
        !Number.isInteger(message.stamp.nanosec) || message.stamp.nanosec < 0 || message.stamp.nanosec >= 1e9 ||
        !Array.isArray(message.detections) || message.detections.length > 8 ||
        !message.detections.every(box => validVisionBox(box, message.width, message.height)) ||
        (message.selected_target !== null && !validVisionBox(message.selected_target, message.width, message.height))) return;
    this.vision = message;
    this.visionSequence = message.seq;
    this.visionAt = rx;
    this.queueLatest('state');
  }

  visionState(now = monoMs()) {
    if (!this.vision) return null;
    const age_ms = this.vision.age_ms + Math.max(0, now - this.visionAt);
    const stale = this.vision.stale || age_ms > 500 || now - this.visionAt > 500 || !this.jetsonOnline(now);
    return {...this.vision, age_ms, stale,
      detections:stale ? [] : this.vision.detections,
      selected_target:stale ? null : this.vision.selected_target};
  }

  navigationState(now = monoMs()) {
    const item = this.telemetryLatest.get('navigation');
    if (!item) return null;
    const age_ms = Math.max(0, now - item.pc_rx_mono_ms);
    const watermarkMatches = u32(item.data.boot_id) && item.data.boot_id !== 0 &&
      u32(item.data.odom_domain) && item.data.odom_domain !== 0 && u32(item.data.last_task_id);
    return {...item.data, task_id_watermark_known:watermarkMatches,
      age_ms, stale:age_ms > 500 || !this.jetsonOnline(now)};
  }

  sendControl(velocity, auto, cause, target = this.target) {
    if (!target) return null;
    try { this.udp.address(); } catch { return null; }
    this.seq = (this.seq + 1) >>> 0;
    const message = Buffer.from(JSON.stringify({ v: 1, type: 'control', session: this.session,
      seq: this.seq, vx: velocity[0], vy: velocity[1], wz: velocity[2], auto: Boolean(auto),
      take_control:Boolean(this.owner),
      ...(this.owner && cause === 'manual-zero-handshake' ? {claim:true} : {}),
      ...(this.status?.session === this.session && typeof this.status.claim_nonce === 'string' ?
        {claim_nonce:this.status.claim_nonce} : {}) }));
    const sentSeq = this.seq;
    this.lastSendAt = monoMs();
    this.udp.send(message, target.port, target.address, error => {
      if (error && !this.stopping) this.emitEvent('udp_send_error', error.message, { cause, seq: sentSeq });
    });
    const controlKey = JSON.stringify([this.session, velocity, Boolean(auto), Boolean(this.owner)]);
    if (controlKey !== this.lastLoggedControlKey) {
      this.lastLoggedControlKey = controlKey;
      this.emitEvent('control_sent', 'control-state-changed', { session: this.session, seq: sentSeq, velocity,
        auto: Boolean(auto), take_control:Boolean(this.owner), cause });
    }
    if (cause === 'initial-zero-handshake' || cause === 'target-change-new-zero' || cause === 'session-rotation-zero') {
      this.emitEvent('zero_handshake', cause, { session: this.session, seq: sentSeq });
    }
    return sentSeq;
  }

  rotateSession(reason, { sendOldZero = true, sendZero = true } = {}) {
    if (sendOldZero) this.sendControl([0, 0, 0], false, `${reason}-old-zero`);
    this.session = sessionId();
    this.seq = 0;
    this.handshakeSent = false;
    this.stopRequest = null;
    this.clearAuthorization();
    if (sendZero) {
      this.sendControl([0, 0, 0], false, 'session-rotation-zero');
      this.handshakeSent = true;
    }
  }

  onBridgeChanged(id, reason) {
    if (this.bridgeId && this.bridgeId !== id) this.retiredBridgeIds.add(this.bridgeId);
    while (this.retiredBridgeIds.size > 16) this.retiredBridgeIds.delete(this.retiredBridgeIds.values().next().value);
    this.bridgeId = id;
    this.rotateSession(reason);
    this.mapping.reset('网桥来源已变化，等待新图');
    this.resetRemoteCaches({ keepBridgeId: true });
    this.emitEvent('bridge_changed', reason, { bridge_id: id });
  }

  resetRemoteCaches({ keepBridgeId = false } = {}) {
    if (!keepBridgeId) this.mapping.reset('连接来源已变化，等待新图');
    this.clearPendingMission('网关或目标已改变，原请求结果未知');
    for (const [request_id,pending] of this.pendingNavigation) {
      this.deliverMissionResult({v:1,type:'mission_result',request_id,
        action:pending.replyAction ?? `navigation_${pending.action}`,success:false,unknown:true,
        message:'网关或目标已改变，导航请求结果未知'},pending.socket);
    }
    this.pendingNavigation.clear();
    this.navigationTask = null;
    this.navigationTaskAt = null;
    this.missionOwner = null;
    this.vision = null;
    this.visionAt = null;
    this.visionSequence = null;
    this.status = null;
    this.statusSequence = null;
    this.lastStatusAt = null;
    this.bridgeEpoch = null;
    this.telemetrySequences.clear();
    this.telemetryLatest.clear();
    this.partials.clear();
    this.lastJetsonRxAt = null;
    this.online = false;
    this.jpeg = new JpegAssembler();
    this.videoAt = null;
    this.videoTimes = [];
    this.videoFrameLatest = null;
    this.videoWasStale = true;
    if (!keepBridgeId) this.bridgeId = null;
  }

  reject(reason) {
    this.rejections += 1;
    if (this.rejections === 1 || this.rejections % 25 === 0) this.emitEvent('message_rejected', reason, { count: this.rejections });
  }

  sourceAllowed(remote) { return this.target && remote.address === this.target.address && remote.port === this.target.port; }

  onUdpMessage(message, remote) {
    const rx = monoMs();
    if (!this.sourceAllowed(remote) || message.length > MAX_DATAGRAM) return this.reject('udp-source-or-size-invalid');
    if (message.length >= 12 && message.toString('ascii', 0, 4) === 'BXJV') {
      if (this.acceptVideoDatagram(message)) this.queueLatest('state');
      return;
    }
    let envelope;
    try { envelope = JSON.parse(message.toString('utf8')); } catch { return this.reject('udp-json-invalid'); }
    if (!isRecord(envelope) || envelope.v !== 1 || typeof envelope.type !== 'string') return this.reject('udp-envelope-invalid');
    if (envelope.type === 'status') return this.acceptStatus(envelope, rx);
    if (envelope.type === 'telemetry') return this.acceptTelemetry(envelope, rx);
    if (envelope.type === 'vision') return this.acceptVision(envelope, rx);
    if (envelope.type === 'mission_result') return this.acceptMissionResult(envelope);
    if (envelope.type === 'navigation_task_result') return this.acceptNavigationTaskResult(envelope);
    if (envelope.type === 'navigation_task_state_error') {
      if (envelope.session === this.session && envelope.bridge_id === this.bridgeId)
        this.emitEvent('navigation_task_state_error', 'Jetson 导航状态过大或无法转发');
      return;
    }
    this.reject(`udp-type-unknown:${envelope.type.slice(0, 32)}`);
  }

  acceptStatus(status, rx) {
    if (typeof status.session !== 'string' || status.session !== this.session || typeof status.bridge_id !== 'string' ||
        status.bridge_id.length < 1 || status.bridge_id.length > 128 || !u32(status.seq) ||
        !Number.isFinite(status.jetson_tx_mono_s)) return;
    if (this.retiredBridgeIds.has(status.bridge_id)) return;
    if (this.bridgeId && this.bridgeId !== status.bridge_id) {
      this.onBridgeChanged(status.bridge_id, 'new-bridge-id');
      return;
    }
    if (!this.bridgeId) this.bridgeId = status.bridge_id;
    if (this.statusSequence !== null && !newer(status.seq, this.statusSequence)) {
      const sequenceMovedBack = status.seq !== this.statusSequence;
      const timeMovedBack = this.status && Number(status.jetson_tx_mono_s) < Number(this.status.jetson_tx_mono_s);
      if (sequenceMovedBack && timeMovedBack && this.lastStatusAt !== null && rx - this.lastStatusAt > 1500) {
        this.onBridgeChanged(status.bridge_id, 'sequence-and-time-reset-after-silence');
      }
      return;
    }
    this.statusSequence = status.seq;
    this.lastStatusAt = rx;
    this.lastJetsonRxAt = rx;
    this.status = status;
    this.online = true;
    if (this.owner && status.pc_control_owner === false && (!this.pendingArm ||
        (u32(status.pc_seq) && (status.pc_seq === this.pendingArm.seq || newer(status.pc_seq, this.pendingArm.seq)))))
      this.disarm('control-owned-by-another-session', {log:true});
    if (this.pendingArm && this.armAcknowledged()) {
      this.pendingArm = null;
      this.emitEvent('armed', 'user-requested-control', { owner: this.owner.id });
    }
    this.queueLatest('state');
  }

  validateTelemetryEnvelope(envelope) {
    return typeof envelope.bridge_id === 'string' && envelope.bridge_id.length > 0 && envelope.bridge_id.length <= 128 &&
      STREAMS.has(envelope.stream) && u32(envelope.epoch) && envelope.valid === true &&
      Number.isFinite(envelope.jetson_rx_mono_s) && Number.isFinite(envelope.jetson_tx_mono_s) &&
      isRecord(envelope.data) && envelope.data.type === envelope.stream && u32(envelope.data.sequence) && u32(envelope.data.time_us);
  }

  acceptTelemetry(envelope, rx) {
    if (!this.validateTelemetryEnvelope(envelope)) return this.reject('telemetry-envelope-invalid');
    if (this.retiredBridgeIds.has(envelope.bridge_id)) return;
    if (this.bridgeId && envelope.bridge_id !== this.bridgeId) {
      this.onBridgeChanged(envelope.bridge_id, 'new-bridge-id');
      // 该包来自新网关，可在会话轮换后作为新状态的首个样本。
    } else if (!this.bridgeId) this.bridgeId = envelope.bridge_id;

    if (this.bridgeEpoch === null) this.bridgeEpoch = envelope.epoch;
    else if (envelope.epoch !== this.bridgeEpoch) {
      if (!newer(envelope.epoch, this.bridgeEpoch)) return;
      this.bridgeEpoch = envelope.epoch;
      this.rotateSession('jetson-epoch-changed');
      this.status = null;
      this.statusSequence = null;
      this.telemetrySequences.clear();
      this.telemetryLatest.clear();
      this.partials.clear();
      this.mapping.reset('遥测 epoch 已变化，等待新图');
      this.emitEvent('bridge_epoch_changed', 'jetson-epoch-changed', { bridge_id: envelope.bridge_id, epoch: envelope.epoch });
    }

    let complete = envelope;
    if (envelope.part !== undefined || envelope.parts !== undefined) {
      complete = this.acceptTelemetryPart(envelope, rx);
      if (!complete) return;
    }
    const stream = complete.stream;
    if (stream === 'navigation_task' && (complete.data.session !== this.session ||
        !isRecord(complete.data.state) || typeof complete.data.state.active !== 'boolean')) return;
    const sequence = complete.data.sequence;
    const previous = this.telemetrySequences.get(stream);
    if (previous !== undefined && !newer(sequence, previous)) return;
    this.telemetrySequences.set(stream, sequence);
    this.lastJetsonRxAt = rx;
    this.online = true;
    const output = { ...complete, pc_rx_wall_ms: Date.now(), pc_rx_mono_ms: rx };
    this.telemetryLatest.set(stream, output);
    if (stream === 'navigation_task') {
      this.navigationTask = output.data.state;
      this.navigationTaskAt = rx;
      this.queueLatest('state');
      return;
    }
    // 在浏览器消息降频/合并之前消费全部已校验和重组的扫描。
    if (['navigation','fusion','local_plan','local_grid'].includes(stream))
      this.mapping.consume(stream, output.data, rx);
    if (stream === 'navigation') {
      const data = output.data;
      if (data.active_task === true || this.lastNavigationLogKey !== null) {
        const fields = ['boot_id','odom_domain','task_id','last_task_id','status','mode','active_task',
          'error','guard_error','planner_error','rx_rejection','rejection_reason','rejected_task_id'];
        const details = Object.fromEntries(fields.filter(key =>
          Number.isInteger(data[key]) || typeof data[key] === 'boolean').map(key => [key,data[key]]));
        const key = JSON.stringify(details);
        if (key !== this.lastNavigationLogKey) {
          this.lastNavigationLogKey = key;
          this.log?.write({type:'navigation_transition',...details,
            goal_x_q16:data.goal_x_q16,goal_y_q16:data.goal_y_q16,progress:data.progress,
            vx_mm_s:data.vx_mm_s,vy_mm_s:data.vy_mm_s,wz_mrad_s:data.wz_mrad_s});
        }
      }
      if (data.active_task !== true) this.lastNavigationLogKey = null;
    }
    // 遥测已有独立消息；不以高频采样挤掉有界队列中的授权、停车和链路事件。
    if (stream !== 'local_grid') this.queueLatest(`telemetry:${stream}`, output);
    if (stream === 'navigation') this.queueLatest('state');
  }

  acceptTelemetryPart(envelope, rx) {
    if (!u32(envelope.part) || !Number.isInteger(envelope.parts) || envelope.parts < 1 || envelope.parts > 32 || envelope.part >= envelope.parts) {
      this.reject('telemetry-part-invalid');
      return null;
    }
    const key = `${envelope.bridge_id}:${envelope.epoch}:${envelope.stream}:${envelope.data.sequence}`;
    for (const [partialKey, value] of this.partials) if (rx - value.firstAt > 250) this.partials.delete(partialKey);
    let assembly = this.partials.get(key);
    if (!assembly) {
      const streamCount = [...this.partials.values()].filter(item => item.stream === envelope.stream).length;
      if (this.partials.size >= 15 || streamCount >= 3) {
        const oldestKey = this.partials.keys().next().value;
        this.partials.delete(oldestKey);
      }
      assembly = { bridge_id: envelope.bridge_id, epoch: envelope.epoch, stream: envelope.stream, parts: envelope.parts, firstAt: rx, data: new Map(), template: envelope };
      this.partials.set(key, assembly);
    }
    if (assembly.parts !== envelope.parts) { this.partials.delete(key); this.reject('telemetry-part-count-conflict'); return null; }
    const existing = assembly.data.get(envelope.part);
    const encoded = JSON.stringify(envelope.data);
    if (existing) {
      if (existing.encoded !== encoded) { this.partials.delete(key); this.reject('telemetry-part-conflict'); }
      return null;
    }
    assembly.data.set(envelope.part, { encoded, data: envelope.data });
    if (assembly.data.size !== assembly.parts) return null;
    const merged = {};
    for (let index = 0; index < assembly.parts; index += 1) {
      const part = assembly.data.get(index);
      if (envelope.stream === 'local_grid' || envelope.stream === 'navigation_task') {
        if (!mergeGridFragment(merged, part.data)) {
          this.partials.delete(key);
          this.reject('telemetry-grid-part-metadata-conflict');
          return null;
        }
      } else mergeObjects(merged, part.data);
    }
    this.partials.delete(key);
    const result = { ...assembly.template, data: merged };
    delete result.part;
    delete result.parts;
    return result;
  }

  jetsonOnline(now = monoMs()) { return this.lastJetsonRxAt !== null && now - this.lastJetsonRxAt <= this.options.linkTimeoutMs; }

  fpgaAckAgeMs() {
    if (!this.status) return null;
    const sinceStatus = this.lastStatusAt === null ? 0 : Math.max(0, monoMs() - this.lastStatusAt);
    if (Number.isFinite(this.status.fpga_ack_age_ms)) return Math.max(0, this.status.fpga_ack_age_ms + sinceStatus);
    if (Number.isFinite(this.status.fpga_ack_age)) return Math.max(0, this.status.fpga_ack_age * 1000 + sinceStatus);
    return null;
  }

  stopState(now = monoMs()) {
    const request = this.stopRequest;
    const motor = this.telemetryLatest.get('motor');
    const motorAgeMs = motor ? Math.max(0, now - motor.pc_rx_mono_ms) : null;
    let wheelsLow = null;
    const wheelSpeeds = motor?.data?.wheel_mm_s;
    if (motor && motorAgeMs <= 1000 && isRecord(wheelSpeeds)) {
      const values = ['A', 'B', 'C', 'D'].map(wheel => wheelSpeeds[wheel]);
      if (values.every(Number.isFinite)) wheelsLow = values.every(value => Math.abs(value) < 5);
    }

    let fpgaAcknowledged = null;
    const lastAck = this.status?.last_ack;
    if (request && isRecord(lastAck) && Number.isFinite(lastAck.jetson_rx_mono_s) &&
        Number.isFinite(this.status?.jetson_tx_mono_s) && this.lastStatusAt !== null) {
      const jetsonAckAge = this.status.jetson_tx_mono_s - lastAck.jetson_rx_mono_s;
      const localStatusAge = Math.max(0, now - this.lastStatusAt);
      if (jetsonAckAge >= 0 && jetsonAckAge * 1000 + localStatusAge <= 250 && localStatusAge <= this.options.linkTimeoutMs) {
        const seqMatches = u32(lastAck.pc_seq) && lastAck.pc_session === request.session &&
          (lastAck.pc_seq === request.seq || newer(lastAck.pc_seq, request.seq));
        const zeroVelocity = lastAck.type === 3 && Array.isArray(lastAck.velocity) && lastAck.velocity.length === 3 &&
          lastAck.velocity.every(value => Number.isFinite(value) && value === 0);
        fpgaAcknowledged = seqMatches && zeroVelocity;
      }
    }
    return {
      requested: Boolean(request),
      pc_seq: request?.seq ?? null,
      session: request?.session ?? null,
      fpga_acknowledged: fpgaAcknowledged,
      wheels_low: wheelsLow,
      threshold_mm_s: 5,
      motor_age_ms: motorAgeMs,
    };
  }

  fpgaReady() {
    const ackAge = this.fpgaAckAgeMs();
    return Boolean(this.status?.ready === true && this.status?.pc_timeout !== true &&
      ackAge !== null && ackAge <= 200 && this.lastStatusAt !== null && monoMs() - this.lastStatusAt <= this.options.linkTimeoutMs);
  }

  armAcknowledged() {
    const pending = this.pendingArm;
    const ack = this.status?.last_ack;
    if (!pending || !this.owner || monoMs() - pending.at >= 200 || !this.jetsonOnline() ||
        !this.fpgaReady() || this.status?.control_source !== 'pc' || this.status?.pc_control_owner === false ||
        !isRecord(ack) || ack.source !== 'pc' || ack.pc_session !== pending.session ||
        !u32(ack.pc_seq) || !(ack.pc_seq === pending.seq || newer(ack.pc_seq, pending.seq)) ||
        ack.type !== 3 || !Array.isArray(ack.velocity) || ack.velocity.length !== 3 ||
        !ack.velocity.every(value => value === 0) || !Number.isFinite(ack.jetson_rx_mono_s) ||
        !Number.isFinite(this.status.jetson_tx_mono_s)) return false;
    const age = this.status.jetson_tx_mono_s - ack.jetson_rx_mono_s;
    return age >= 0 && age * 1000 + Math.max(0, monoMs() - this.lastStatusAt) <= 200;
  }

  tick(now = monoMs()) {
    if (this.stopping) return;
    for (const [request_id,pending] of this.pendingNavigation) {
      if (now - pending.at <= 5000) continue;
      this.pendingNavigation.delete(request_id);
      if (pending.fallbackOnFailure && pending.action !== 'start')
        this.sendMissionCommand(null,'stop',{}, {skipNavigation:true});
      this.deliverMissionResult({v:1,type:'mission_result',request_id,
        action:pending.replyAction ?? `navigation_${pending.action}`,success:false,unknown:true,
        message:'Jetson 导航请求超时，结果未知，请核对导航状态'},pending.socket);
    }
    for (const [request_id, pending] of this.pendingMission) {
      if (now - pending.at <= 2500) continue;
      this.pendingMission.delete(request_id);
      this.deliverPendingMissionResult({v:1, type:'mission_result', request_id, action:pending.action,
        success:false, unknown:true, message:'命令超时，结果未知；未自动重试，请核对实际状态'}, pending);
    }
    this.jpeg.prune(now);
    if (!this.videoWasStale && this.videoAt !== null && now - this.videoAt > 1000) {
      this.jpeg = new JpegAssembler();
      this.videoWasStale = true;
      this.videoFrameLatest = null;
      this.emitEvent('video_stale', 'video-silent-for-one-second');
    }
    if (this.owner && (this.lastDriveAt === null || now - this.lastDriveAt > 200)) {
      this.disarm('page-command-expired', { log: true });
    }
    if (this.online && !this.jetsonOnline(now)) {
      this.online = false;
      this.disarm('jetson-udp-timeout', { log: true });
      this.rotateSession('jetson-udp-timeout', { sendOldZero: false });
      this.resetRemoteCaches({ keepBridgeId: true });
      this.emitEvent('jetson_offline', 'udp-timeout');
    }
    if (this.owner && this.pendingArm && now - this.pendingArm.at >= 200) this.disarm('fpga-handshake-timeout', { log: true });
    if (!this.fpgaReady() && this.owner && !this.pendingArm) this.disarm('fpga-ack-stale', { log: true });
    if (now - this.lastSendAt >= 50) {
      const movingAllowed = this.owner && !this.pendingArm && this.lastDriveAt !== null && now - this.lastDriveAt <= 200 && this.fpgaReady();
      this.sendControl(movingAllowed ? this.velocity : [0, 0, 0], false, movingAllowed ? 'page-command' : 'idle-zero');
    }
    for (const socket of this.clients.keys()) {
      const client = this.clients.get(socket);
      if (socket.readyState === WebSocket.OPEN && socket.bufferedAmount > this.options.maxClientQueueBytes) {
        this.closeSlowClient(socket, 'client send queue exceeded');
        continue;
      }
      if (client?.kind === 'video' && socket.readyState === WebSocket.OPEN && socket.bufferedAmount > this.options.maxVideoQueueBytes) {
        this.closeSlowClient(socket, 'video client too slow');
        continue;
      }
      const transportBlocked = client?.kind === 'json' &&
        socket.bufferedAmount >= Math.min(CLIENT_BATCH_BYTES, this.options.maxClientQueueBytes / 2);
      if (client?.kind === 'json' && !transportBlocked) client.backpressureSince = null;
      if (client?.kind === 'json' && client.deliveryAckEnabled && client.deliveryPending &&
          now - client.deliveryPending.at > CLIENT_DELIVERY_TIMEOUT_MS) {
        this.closeSlowClient(socket, transportBlocked ? 'client transport stalled' : 'client consumption stalled');
        continue;
      }
      if (client?.kind === 'json' && client.backpressureSince !== null && client.backpressureSince !== undefined &&
          now - client.backpressureSince > CLIENT_DELIVERY_TIMEOUT_MS) {
        this.closeSlowClient(socket, 'client transport stalled');
      }
    }
  }

  state() {
    const now = monoMs();
    this.videoTimes = this.videoTimes.filter(time => now - time <= 1000);
    return {
      v: 1,
      type: 'state',
      owner: this.owner?.id ?? null,
      armed: Boolean(this.owner && !this.pendingArm),
      reason: this.owner ? 'manual' : (this.online ? 'monitor' : 'offline'),
      jetson: {
        host: this.target?.host ?? null,
        port: this.target?.port ?? null,
        online: this.jetsonOnline(now),
        age_ms: this.lastJetsonRxAt === null ? null : Math.max(0, now - this.lastJetsonRxAt),
      },
      fpga: {
        ready: this.fpgaReady(),
        ack_age_ms: this.fpgaAckAgeMs(),
      },
      status: this.status,
      vision: this.visionState(now),
      navigation: this.navigationState(now),
      navigation_task: this.navigationTask && this.navigationTaskAt !== null && now-this.navigationTaskAt > 1500 ?
        {...this.navigationTask, stale:true, route:[], message:'Jetson 导航状态已过期'} : this.navigationTask,
      stop: this.stopState(now),
      video: {
        age_ms: this.videoAt === null ? null : Math.max(0, now - this.videoAt),
        fps: this.videoTimes.length,
        dropped: this.videoDropped + this.jpeg.dropped,
      },
      simulation: Boolean(this.options.simulation),
    };
  }

  publishState() {
    this.queueLatest('state');
  }

  publishMap() {
    const clients = [...this.clients.values()].filter(client => client.kind === 'map');
    if (!clients.length) return;
    const snapshot = JSON.stringify(this.mapping.snapshot(monoMs()));
    for (const client of clients) client.pendingMap = snapshot;
  }

  queueLatest(key, value = null) {
    for (const client of this.clients.values()) {
      if (client.kind !== 'json') continue;
      if (key.startsWith('telemetry:')) client.pendingTelemetry.set(key.slice('telemetry:'.length), value ?? this.telemetryLatest.get(key.slice('telemetry:'.length)));
      else if (key === 'state') client.pendingState = true;
    }
  }

  closeSlowClient(socket, reason) {
    const client = this.clients.get(socket);
    if (!client) return;
    this.log?.write({type:'client_disconnect', client_id:client.id, reason});
    if (socket.readyState === WebSocket.OPEN) socket.close(1013, reason);
    // 不等待慢连接的 close 握手；现有路径立即撤权并取消协同任务。
    this.onClientClose(socket);
  }

  sendText(socket, text) {
    if (socket.readyState !== WebSocket.OPEN) return false;
    try { socket.send(text, { binary: false }); return true; }
    catch { this.onClientClose(socket); return false; }
  }

  enqueueClientEvent(socket, client, value) {
    if (client.events.length >= 32) {
      this.closeSlowClient(socket, 'client event queue exceeded');
      return false;
    }
    client.events.push(value);
    return true;
  }

  flushClients(now = monoMs()) {
    for (const [socket, client] of this.clients) {
      if (socket.readyState !== WebSocket.OPEN) continue;
      if (socket.bufferedAmount > this.options.maxClientQueueBytes) {
        this.closeSlowClient(socket, 'client send queue exceeded');
        continue;
      }
      if (client.kind === 'map') {
        if (client.pendingMap && socket.bufferedAmount < this.options.maxClientQueueBytes / 2) {
          socket.send(client.pendingMap);
          client.pendingMap = null;
        }
      } else if (client.kind === 'json') {
        const limit = Math.min(CLIENT_BATCH_BYTES, this.options.maxClientQueueBytes / 2);
        if (socket.bufferedAmount >= limit) {
          if (client.backpressureSince == null) client.backpressureSince = now;
          continue;
        }
        client.backpressureSince = null;
        // 一个未确认的批次最多在浏览器中排队一次。旧监视器可以只读，不能取得控制权。
        if (client.deliveryAckEnabled && client.deliveryPending) continue;
        if (!client.events.length && client.lastBatchAt != null && now - client.lastBatchAt < CLIENT_BATCH_INTERVAL_MS) continue;
        let budget = limit - socket.bufferedAmount - 256;
        let sent = false;
        const send = value => {
          const text = JSON.stringify(value), bytes = Buffer.byteLength(text) + 14;
          if (bytes > limit - 256) {
            this.closeSlowClient(socket, 'client message exceeds delivery limit');
            return 'closed';
          }
          if (bytes > budget) return 'full';
          if (!this.sendText(socket, text)) return 'closed';
          budget -= bytes;
          sent = true;
          return 'sent';
        };
        while (client.events.length) {
          const outcome = send(client.events[0]);
          if (outcome !== 'sent') break;
          client.events.shift();
        }
        if (!this.clients.has(socket)) continue;
        if (client.events.length === 0 && client.pendingState) {
          if (send(this.state()) === 'sent') client.pendingState = null;
        }
        if (!this.clients.has(socket)) continue;
        if (client.events.length === 0 && !client.pendingState) {
          for (const [stream, envelope] of client.pendingTelemetry) {
            const maxAge = stream === 'health' ? 2000 : stream === 'navigation' ? 500 : 1000;
            if (Number.isFinite(envelope?.pc_rx_mono_ms) && now - envelope.pc_rx_mono_ms > maxAge) {
              client.pendingTelemetry.delete(stream);
              continue;
            }
            const outcome = send(envelope);
            if (outcome !== 'sent') break;
            client.pendingTelemetry.delete(stream);
          }
        }
        if (!this.clients.has(socket) || !sent) continue;
        client.lastBatchAt = now;
        if (client.deliveryAckEnabled) {
          client.deliverySeq = (client.deliverySeq + 1) >>> 0;
          client.deliveryPending = {id:client.deliverySeq, at:now};
          this.sendText(socket, JSON.stringify({v:1,type:'delivery_probe',id:client.deliverySeq}));
        }
      } else if (client.pendingVideo && socket.bufferedAmount <= this.options.maxVideoQueueBytes) {
        socket.send(client.pendingVideo, { binary: true }, error => { if (error) this.onClientClose(socket); });
        client.pendingVideo = null;
      }
    }
  }

  safeSend(socket, value) {
    if (socket.readyState !== WebSocket.OPEN) return;
    const client = this.clients.get(socket);
    if (client?.kind === 'json' && client.events &&
        (client.deliveryPending || client.events.length || socket.bufferedAmount >= CLIENT_BATCH_BYTES / 2)) {
      this.enqueueClientEvent(socket, client, value);
      return;
    }
    this.sendText(socket, JSON.stringify(value));
  }

  sendEvent(socket, event, reason, extra = {}) {
    this.safeSend(socket, makeEvent(event, reason, extra));
  }

  emitEvent(event, reason, extra = {}, record = true) {
    const value = makeEvent(event, reason, extra);
    const entry = { id: this.nextEventId++, value };
    this.events.push(entry);
    if (this.events.length > 64) this.events.shift();
    for (const [socket, client] of this.clients) {
      if (client.kind !== 'json') continue;
      this.enqueueClientEvent(socket, client, value);
    }
    if (record) this.log?.write(value);
  }

  acceptVideoDatagram(packet) {
    const assembled = this.jpeg.feed(packet);
    if (!assembled) return false;
    const now = monoMs();
    this.videoAt = now;
    this.videoWasStale = false;
    this.videoTimes.push(now);
    this.videoFrameLatest = assembled.jpeg;
    for (const client of this.clients.values()) if (client.kind === 'video') client.pendingVideo = assembled.jpeg;
    return true;
  }
}

export async function createDashboardServer(options = {}) {
  const bridge = new DashboardBridge(options);
  await bridge.start();
  return bridge;
}

function readArgs(argv) {
  const result = {};
  for (let index = 0; index < argv.length; index += 1) {
    const item = argv[index];
    if (!item.startsWith('--')) continue;
    const [rawKey, inline] = item.slice(2).split('=', 2);
    const key = rawKey.replaceAll('-', '_');
    const next = argv[index + 1];
    const rawValue = inline ?? (next && !next.startsWith('--') ? argv[++index] : undefined);
    if (rawValue === undefined || rawValue === 'true') result[key] = true;
    else if (rawValue === 'false') result[key] = false;
    else result[key] = rawValue;
  }
  return result;
}

if (process.argv[1] && path.resolve(process.argv[1]) === fileURLToPath(import.meta.url)) {
  const args = readArgs(process.argv.slice(2));
  const bridge = await createDashboardServer({
    bindHost: args.bind ?? '127.0.0.1',
    httpPort: Number(args.http_port ?? 8765),
    udpBindHost: args.udp_bind ?? '0.0.0.0',
    jetsonHost: args.jetson ?? '127.0.0.1',
    jetsonPort: Number(args.port ?? args.jetson_port ?? 6200),
    globalMapPort: Number(args.global_map_port ?? 8766),
    simulation: Boolean(args.simulation),
    speed: Number(args.speed ?? 400),
    turn: Number(args.turn ?? 800),
    logDirectory: args.log_dir ?? path.join(HERE, 'logs'),
  });
  const address = bridge.http.address();
  process.stdout.write(`BX72 dashboard ${address.address}:${address.port}; Jetson ${bridge.target.host}:${bridge.target.port}; simulation=${bridge.options.simulation}\n`);
  const shutdown = async () => { await bridge.close(); process.exit(0); };
  process.once('SIGINT', shutdown);
  process.once('SIGTERM', shutdown);
}
