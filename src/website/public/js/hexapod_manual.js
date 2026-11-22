/* Shared keyboard ownership for the car and ACG720; each target keeps its own API. */
(() => {
  'use strict';
  const targets = new Map();
  const motions = {q:'turn_left',w:'forward',e:'turn_right',a:'strafe_left',s:'backward',d:'strafe_right'};
  let owner = null, stopCar = null;
  const targetLabel = () => {
    const element = document.getElementById('manualTarget');
    if (element) element.textContent = owner === 'car' ? 'BX72 小车' :
      targets.get(owner)?.name || '未选择';
  };
  function select(id) {
    const previous = owner;
    owner = id;
    for (const [key, target] of targets) if (key !== id) target.disarm();
    if (previous === 'car' && id !== 'car') stopCar?.();
    targetLabel();
  }
  async function post(url, data) {
    const controller = new AbortController(), timer = setTimeout(() => controller.abort(), 2000);
    try {
      const response = await fetch(url, {method:'POST',cache:'no-store',keepalive:true,
        headers:{'Content-Type':'application/json'},body:JSON.stringify(data),signal:controller.signal});
      let result;
      try { result = await response.json(); } catch (_) { result = {}; }
      if (!response.ok || result.accepted === false) throw Error(result.error || `请求失败 HTTP ${response.status}`);
      return result;
    } finally { clearTimeout(timer); }
  }
  function create(options) {
    const {id, name, keys, arm, stand, stop, result, drive, panel} = options;
    const client = `web-${id}-` + (globalThis.crypto?.randomUUID?.() || `${Date.now()}-${Math.random().toString(36).slice(2)}`);
    let armed = false, held = null, timer = null, seq = 0, generation = 0;
    let inFlight = null, releaseBarrier = null;
    const extra = options.extra || [];
    const say = message => { result.textContent = message; };
    const control = (action, pressed) => post(options.url, {action,pressed,client_id:client,seq:++seq,lease_ms:500});
    function release(priorFailure = '') {
      if (timer) clearInterval(timer);
      timer = null;
      if (!held) return releaseBarrier;
      const action = held, prior = inFlight?.promise;
      held = null; generation++;
      keys.forEach(button => button.classList.remove('active'));
      const sendRelease = () => control(action, false).catch(error => say(
        `${typeof priorFailure === 'string' && priorFailure ? priorFailure+'；' : ''}松开请求失败：${error.message}`));
      const released = sendRelease();
      // A delayed movement response must not arrive after the last release.
      const settled = prior ? Promise.allSettled([prior, released]).then(() => sendRelease()) : released;
      releaseBarrier = settled;
      settled.finally(() => { if (releaseBarrier === settled) releaseBarrier = null; });
      return settled;
    }
    function disarm() {
      armed = false;
      generation++;
      const released = release();
      if (owner === id) { owner = null; targetLabel(); }
      refresh();
      return released;
    }
    function refresh() {
      if (armed && !options.ready()) { disarm(); return; }
      arm.disabled = !options.ready();
      arm.textContent = `${armed ? '停用' : '启用'} ${name} 手控`;
      drive.textContent = armed ? `${name} · 当前手控目标` : `${name} · 未启用手控`;
      panel.classList[armed ? 'add' : 'remove']('manual-selected');
      keys.forEach(button => { button.disabled = !armed || !options.ready(); });
      stand.disabled = !options.canStand(armed);
      stop.disabled = !options.connected();
      for (const entry of extra) entry.button.disabled = entry.always
        ? !options.connected() : !armed || !options.ready();
    }
    function press(action, button) {
      if (!armed || owner !== id || !options.ready() || held) return;
      held = action;
      const current = ++generation;
      button ||= keys.find(key => key.dataset.acg720Action === action);
      button?.classList.add('active');
      const send = () => {
        if (inFlight || !armed || held !== action || current !== generation) return;
        if (!options.ready()) { disarm(); return; }
        const request = {generation:current};
        inFlight = request;
        request.promise = control(action, true).catch(error => {
          if (inFlight === request) inFlight = null;
          if (current !== generation) return;
          const failure = `动作请求失败：${error.message}`;
          release(failure); armed = false;
          if (owner === id) { owner = null; targetLabel(); }
          say(failure); refresh();
        }).finally(() => { if (inFlight === request) inFlight = null; });
      };
      const start = () => {
        if (current !== generation || held !== action) return;
        send(); timer = setInterval(send, 200);
      };
      if (releaseBarrier) releaseBarrier.then(start);
      else start();
    }
    function oneShot(action, preserveHold = false) {
      const released = preserveHold ? null : release();
      const current = preserveHold ? generation : ++generation;
      const invoke = () => {
        if (!preserveHold && current !== generation) return;
        return control(action, true);
      };
      const urgent = action === 'stop' || action === 'release';
      const request = urgent || !released ? invoke() : released.then(invoke);
      const finished = urgent && released
        ? Promise.allSettled([released, request]).then(invoke) : Promise.resolve(request);
      finished.then(value => { if (value !== undefined) say(options.message(action)); })
        .catch(error => say(error.message));
    }
    arm.onclick = () => {
      if (!options.ready()) return;
      if (armed) disarm();
      else { select(id); armed = true; refresh(); }
    };
    for (const button of keys) {
      const action = button.dataset.acg720Action;
      button.addEventListener('pointerdown', event => {
        if (!armed || !options.ready() || held) return;
        event.preventDefault(); button.setPointerCapture(event.pointerId); press(action, button);
      });
      for (const type of ['pointerup','pointercancel','lostpointercapture']) button.addEventListener(type, () => release());
    }
    stand.onclick = () => { if (options.canStand(armed)) oneShot('stand'); };
    stop.onclick = () => {
      if (!options.connected()) return;
      disarm(); oneShot('stop');
    };
    for (const entry of extra) entry.button.onclick = () => {
      if (entry.button.disabled) return;
      if (entry.always) disarm();
      oneShot(entry.action, entry.preserveHold);
    };
    const target = {name,refresh,disarm,release,press,stop:()=>stop.onclick(),
      releaseKey(action) { if (held === action) release(); }};
    targets.set(id, target);
    refresh();
    return target;
  }
  document.addEventListener('keydown', event => {
    const target = targets.get(owner);
    if (!target || event.repeat || event.ctrlKey || event.altKey || event.metaKey ||
        event.target?.isContentEditable || /^(INPUT|TEXTAREA|SELECT)$/.test(event.target?.tagName)) return;
    const action = motions[event.key.toLowerCase()];
    if (action) { event.preventDefault(); target.press(action); }
    if (event.code === 'Space') { event.preventDefault(); target.stop(); }
  });
  document.addEventListener('keyup', event => targets.get(owner)?.releaseKey(motions[event.key.toLowerCase()]));
  const disarmAll = () => { for (const target of targets.values()) target.disarm(); };
  window.addEventListener('blur', disarmAll);
  window.addEventListener('pagehide', disarmAll);
  document.addEventListener('visibilitychange', () => { if (document.hidden) disarmAll(); });
  window.HexapodManualControls = {
    create,keyboardOwner:()=>owner,selectCar:()=>select('car'),registerCar:callback=>{ stopCar=callback; },
    releaseCar:()=>{ if (owner==='car') { owner=null;targetLabel(); } }
  };
  targetLabel();
})();
