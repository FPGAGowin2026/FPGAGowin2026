/* ACG720 独立状态、传感器预览与复用的六足手控。 */
(() => {
  'use strict';
  const $ = id => document.getElementById(id);
  if (!$('acg720Panel')) return;
  const text = (id, value) => { $(id).textContent = value; };
  let previewVersion = -1, state = null, lastStatusAt = 0, connected = false;
  const commandNames = ['停止','前进','后退','左转','右转','左移','右移'];
  const mode = value => value === true ? '涟漪' : value === false ? '标准三角' : '未知';
  function ready() {
    return connected && Date.now()-lastStatusAt <= 2500 && state?.fpga_connected === true &&
      state.teleop_state === 'READY' && !state.error && Number.isFinite(state.teleop_age_s) &&
      state.teleop_age_s >= 0 && state.teleop_age_s+(Date.now()-lastStatusAt)/1000 <= 1.5;
  }
  const manual = window.HexapodManualControls.create({
    id:'acg720',name:'ACG720 六足',url:'/acg720/api/control',
    keys:[...document.querySelectorAll('[data-acg720-action]')],
    arm:$('acg720Arm'),stand:$('acg720Stand'),stop:$('acg720Stop'),result:$('acg720ControlResult'),
    drive:$('acg720DriveStatus'),panel:$('acg720Remote'),ready,
    connected:()=>connected && Date.now()-lastStatusAt<=2500,canStand:armed=>armed && ready(),
    extra:[{button:$('acg720Release'),action:'release',always:true},
      {button:$('acg720Tripod'),action:'tripod',preserveHold:true},
      {button:$('acg720Ripple'),action:'ripple',preserveHold:true}],
    message:action=>`${({stand:'回低姿态',stop:'停止步态',release:'关闭 PWM',tripod:'标准三角模式',ripple:'涟漪模式'})[action]}指令已发送 · 等待 HXS1`
  });
  const image = $('acg720Preview');
  image.addEventListener('error', () => { image.hidden = true; previewVersion = -1; });
  async function get(path) {
    const response = await fetch(path, {cache: 'no-store', signal: AbortSignal.timeout(2000)});
    if (!response.ok) throw Error(`HTTP ${response.status}`);
    return response.json();
  }
  async function refresh() {
    const [fleet, robot] = await Promise.allSettled([get('/api/fleet'), get('/acg720/api/status')]);
    if (fleet.status === 'fulfilled') {
      text('fleetMembers', (fleet.value.members || []).map(member => `${member.name} · ${member.host}`).join(' / '));
      const ros = fleet.value.acg720_ros;
      text('acg720Ros', ros && ros.age_ms < 2500 ? `已连接 · ${ros.age_ms} ms` : '等待 ROS 2 心跳');
    } else {
      text('fleetMembers', '集群成员信息不可用');
      text('acg720Ros', '总控连接中断');
    }
    if (robot.status === 'fulfilled') {
      state = robot.value; connected = true; lastStatusAt = Date.now();
      text('acg720Connection', state.fpga_connected ? '树莓派与 FPGA 在线' : '树莓派在线 · 等待 FPGA');
      text('acg720Pwm', state.pwm_enabled === null ? '未知' : state.pwm_enabled ? '已开启' : '已关闭');
      text('acg720Imu', state.imu_ready ? '实时接收' : '等待有效 IMU');
      text('acg720Lidar', state.lidar_state === 'READY' ? `${state.lidar_points} 点 / 圈` : '等待雷达');
      text('acg720Camera', state.camera_ready ? `实时接收 · 完整帧 ${state.complete_frames}` : '等待相机');
      text('acg720Navigation', state.navigation_ready ? state.navigation_reason :
        '自动导航未接入');
      text('acg720ManualConnection', state.fpga_connected ? '树莓派与 FPGA 在线 · /acg720' : '树莓派在线 · 等待 FPGA 状态');
      text('acg720Motion', `请求 ${commandNames[state.requested_command] || '未知'} · 实际 ${commandNames[state.active_gait] || '未知'}${state.homing ? ' · 正在回低姿态' : ''}`);
      text('acg720Mode', `所选 ${mode(state.ripple_selected)} · 当前轨迹 ${mode(state.trajectory_ripple)}`);
      text('acg720ManualPwm', state.pwm_enabled === true ? '已开启' : state.pwm_enabled === false ? '已关闭' : '未知');
      text('acg720Feedback', `HXS1 #${state.teleop_status_version ?? '—'} · 接收年龄 ${Number.isFinite(state.teleop_age_s) ? state.teleop_age_s.toFixed(2)+' s' : '未知'}${state.error ? ' · '+state.error : ''}`);
      $('acg720Tripod').setAttribute('aria-pressed', String(state.ripple_selected === false));
      $('acg720Ripple').setAttribute('aria-pressed', String(state.ripple_selected === true));
      if (state.camera_ready && state.preview_version !== previewVersion) {
        previewVersion = state.preview_version;
        image.src = `/acg720/frame.jpg?v=${previewVersion}`;
        image.hidden = false;
      } else if (!state.camera_ready) image.hidden = true;
    } else {
      connected = false; state = null;
      text('acg720Connection', 'ACG720 接入服务未连接');
      for (const id of ['acg720Pwm', 'acg720Imu', 'acg720Lidar', 'acg720Camera']) text(id, '不可用');
      text('acg720Navigation', '等待树莓派接入服务');
      text('acg720ManualConnection', 'ACG720 接入服务不可用');
      for (const id of ['acg720Motion','acg720Mode','acg720ManualPwm']) text(id, '不可用');
      text('acg720Feedback', '等待 ACG720 状态');
      image.hidden = true;
    }
    manual.refresh();
    setTimeout(refresh, 1000);
  }
  refresh();
})();
