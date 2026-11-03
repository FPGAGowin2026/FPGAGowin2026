// Windows + WSL 跨仓真实网关验收，仅 fake FPGA 的 localhost 端点。
import assert from 'node:assert/strict';
import { spawn, execFileSync } from 'node:child_process';
import { createRequire } from 'node:module';
import { fileURLToPath } from 'node:url';
import { once } from 'node:events';
import test from 'node:test';
import { createDashboardServer } from '../server.mjs';

const require = createRequire(new URL('../package.json', import.meta.url));
const WebSocket = require('ws');
// 直接运行本文件时 argv[2] 才是网关可执行文件；`node --test a.test.mjs b.test.mjs` 批量运行时 argv[2] 是测试文件名。
const candidate = process.argv[2] ?? process.env.BX72_CPP_BRIDGE;
const bridgeExecutable = candidate && !candidate.startsWith('-') && !/\.(?:test|spec)\.[cm]?js$/i.test(candidate) ? candidate : undefined;
const skipReason = bridgeExecutable ? false : '未提供 C++ 网关可执行文件参数，跳过跨仓验收';
if (!bridgeExecutable) {
  console.log(`SKIP: ${skipReason}；用法：node test/dashboard_cpp.test.mjs /path/to/ros_gateway_wrapper`);
}

test('C++ 网关跨仓验收', { skip: skipReason }, async () => {
  const distro = process.env.BX72_TEST_DISTRO ?? 'Ubuntu-22.04';
  const fixtureWindows = fileURLToPath(new URL('./dashboard_cpp_fixture.py', import.meta.url));
  const fixtureLinux = execFileSync('wsl.exe', ['-d', distro, '--exec', 'wslpath', '-a', fixtureWindows.replaceAll('\\', '/')], { encoding: 'utf8' }).trim();
  // mirrored 模式允许 Windows↔WSL localhost UDP；NAT 模式可显式设置 WSL IP。
  const address = process.env.BX72_TEST_WSL_IP ?? '127.0.0.1';
  const child = spawn('wsl.exe', ['-d', distro, '--exec', 'python3', '-u', fixtureLinux, '--bridge', bridgeExecutable, '--duration', '60'], { stdio: ['pipe', 'pipe', 'pipe'] });
  const requests = [];
  let fixtureReady, stderr = '', buffered = '';
  child.stderr.on('data', data => { stderr = (stderr + data).slice(-8000); });
  child.stdout.on('data', data => {
    buffered += data;
    const lines = buffered.split('\n');
    buffered = lines.pop();
    for (const line of lines) {
      try {
        const value = JSON.parse(line);
        if (value.ready) fixtureReady = value;
        if (value.request) { requests.push(value); if (requests.length > 3000) requests.shift(); }
      } catch { /* Fixture emits only small JSON records; partial lines wait for next data. */ }
    }
  });
  const delay = ms => new Promise(resolve => setTimeout(resolve, ms));
  async function until(predicate, label, timeout = 10000) {
    const end = Date.now() + timeout;
    while (Date.now() < end) {
      if (predicate()) return;
      if (child.exitCode !== null) throw Error(`fixture exited ${child.exitCode}: ${stderr}`);
      await delay(20);
    }
    throw Error(`等待超时：${label}; ${stderr}`);
  }
  let server, socket, video, heartbeat;
  const messages = [];
  const streams = new Map();
  let state, hello, jpegCount = 0, seq = 0;
  function send(type, extra = {}) { socket.send(JSON.stringify({ v: 1, type, ...extra })); }
  try {
    await until(() => fixtureReady, 'C++网关启动');
    server = await createDashboardServer({ httpPort: 0, jetsonHost: address, jetsonPort: fixtureReady.pc_port, simulation: true, log: false });
    const port = server.http.address().port;
    const origin = `http://127.0.0.1:${port}`;
    const page = await fetch(origin);
    assert.equal(page.status, 200);
    assert.match(await page.text(), /dashboard_network\.js/);
    socket = new WebSocket(`ws://127.0.0.1:${port}/ws`, { origin });
    socket.on('message', bytes => {
      const value = JSON.parse(bytes.toString());
      if (value.type === 'hello' && value.delivery_ack === 1) send('delivery_ready');
      if (value.type === 'delivery_probe') send('delivery_ack', {id:value.id});
      messages.push(value);
      if (messages.length > 500) messages.shift();
      if (value.type === 'state') state = value;
      if (value.type === 'hello') hello = value;
      if (value.type === 'telemetry') streams.set(value.stream, value);
    });
    await once(socket, 'open');
    video = new WebSocket(`ws://127.0.0.1:${port}/video`, { origin });
    video.on('message', bytes => {
      assert.equal(bytes.readUInt16BE(0), 0xffd8);
      assert.equal(bytes.readUInt16BE(bytes.length-2), 0xffd9);
      jpegCount++;
    });
    await until(() => state?.fpga?.ready && hello && streams.size === 6 && jpegCount >= 3 &&
      [...server.clients.values()].some(client => client.deliveryAckAt != null), '六类C++遥测、FPGA零速ACK与PT97视频转发');
    assert.equal(state.armed, false);
    assert.equal(streams.get('imu').data.who_am_i, 0x70);
    assert.equal(streams.get('motor').data.motor_channel_map, 0xd2);
    assert.equal(streams.get('lidar').data.points.length, 16);
    assert.equal(streams.get('fusion').data.state.x_m, 1);
    assert.equal(streams.get('navigation').data.status, 0);
    assert.ok(Number.isSafeInteger(streams.get('navigation').data.now_us));
    assert.equal(streams.get('navigation').data.last_task_id,0);
    assert.ok(Number.isFinite(streams.get('health').pc_rx_wall_ms));
    send('arm');
    heartbeat = setInterval(() => send('drive', { seq: ++seq, vx: 0, vy: 0, wz: 0 }), 50);
    await until(() => state?.armed && state.owner === hello.client_id, '网页取得唯一控制权');
    clearInterval(heartbeat);
    heartbeat = setInterval(() => send('drive', { seq: ++seq, vx: 60, vy: -20, wz: 100 }), 50);
    await until(() => requests.some(row => row.velocity[0] === 60 && row.velocity[1] === -20 && row.velocity[2] === 100), 'WebSocket→PC→C++→FPGA三轴报文');
    clearInterval(heartbeat);
    const frozenAt = requests.length;
    await until(() => state?.armed === false && requests.slice(frozenAt).some(row => row.request === 3 && row.velocity.every(v => v === 0)), '网页指令冻结后独立停车/解除');
    await until(() => state?.stop?.requested && state.stop.fpga_acknowledged === true && state.stop.wheels_low === true, '本次PC序号关联的零速ACK和独立轮速阈值证据');
    send('arm');
    heartbeat = setInterval(() => send('drive', { seq: ++seq, vx: 0, vy: 0, wz: 0 }), 50);
    await until(() => state?.armed, '重新主动授权');
    const type5BeforeLegacyAuto = requests.filter(row => row.request === 5 && row.auto).length;
    send('auto', { enabled: true });
    await until(() => messages.some(value => value.type === 'event' && value.event === 'auto_rejected'), '无目标自动模式明确退役提示');
    assert.equal(requests.filter(row => row.request === 5 && row.auto).length, type5BeforeLegacyAuto,
      '无目标自动请求不得转发为FPGA type5自动模式');
    send('stop', { reason: 'cross-repo-acceptance' });
    clearInterval(heartbeat);
    const stoppedAt = requests.length;
    await until(() => state?.armed === false && state?.status?.actual_mode === 'manual' &&
      requests.slice(stoppedAt).some(row => !row.auto && row.velocity.every(v => v === 0)), '明确停车与解除控制');
    await until(() => state?.stop?.fpga_acknowledged === true && state.stop.wheels_low === true,
      '零速ACK及独立轮速阈值证据');
    send('local_task', {action:'start', mode:'cooperative', allow_disconnect:false, goal_x_m:1.25, goal_y_m:-0.5,
      hold_yaw_rad:0, final_yaw_rad:0, max_speed_mm_s:80, max_accel_mm_s2:200,
      tolerance_mm:64, yaw_tolerance_mrad:100, duration_s:30});
    await until(() => messages.some(value => value.type === 'mission_result' && value.action === 'local_start' && !value.success),
      '旧协同局部任务明确拒绝并提示使用 Jetson 全局航点');
    assert.equal(requests.filter(row => row.request === 7).length,0,'网页不应直接发 FPGA 任务分片');
    console.log(JSON.stringify({ result: 'PASS', actual_cpp_gateway: true, software_simulation_only: true, fake_fpga_only: true, streams: [...streams.keys()], jpeg_frames: jpegCount, fpga_requests: requests.length, stop_evidence: state.stop, checks: ['六类二进制遥测解码', 'PT97/RTP→C++OpenCV→BXJV→独立视频WebSocket', '三轴控制', '网页冻结停车', '重新主动授权', '无目标自动模式拒绝', '零速ACK与独立轮速阈值', '旧协同航点不绕过 Jetson 全局调度'] }, null, 2));
  } catch (error) {
    console.error(JSON.stringify({ error: error.message, address, fixture: fixtureReady, state, streams: [...streams.keys()], jpegCount, requests: requests.slice(-3), stderr }, null, 2));
    throw error;
  } finally {
    clearInterval(heartbeat);
    socket?.terminate();
    video?.terminate();
    if (server) await server.close();
    child.stdin.write('stop\n');
    await Promise.race([once(child, 'exit'), delay(5000).then(() => child.kill())]);
  }
});
