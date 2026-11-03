/* 读取 Jetson 全局地图；绘制由同一个地图视图负责。 */
(function (root, factory) {
  const codec = typeof module === 'object' && module.exports ? require('./grid_codec.js') : root.BX72GridCodec;
  const api = factory(codec);
  if (typeof module === 'object' && module.exports) module.exports = api;
  if (root) root.BX72GlobalMap = api;
})(typeof globalThis === 'undefined' ? this : globalThis, function (codec) {
  'use strict';

  function validHeader(map) {
    return map?.v === 1 && map.type === 'global_mapping' &&
      map.protocol === 'slam_toolbox_view_v1' && map.source === 'slam_toolbox' &&
      typeof map.available === 'boolean';
  }

  function decode(map) {
    if (!validHeader(map) || map.available !== true ||
        ![map.width,map.height].every(n => Number.isInteger(n) && n > 0 && n <= 2048) ||
        map.width*map.height > 2000000 || !Number.isFinite(map.resolution_m) || map.resolution_m <= 0 ||
        !Number.isFinite(map.origin?.x) || !Number.isFinite(map.origin?.y)) throw new Error('全局地图格式无效');
    return codec.decodeCells(map.cells_rle, map.width * map.height);
  }

  function mapKey(map) {
    return JSON.stringify([map.domain,map.generation,map.revision,map.width,map.height,
      map.resolution_m,map.origin?.x,map.origin?.y]);
  }

  function createSource({window:win, fetchMap = (...args) => win.fetch(...args), onUpdate = () => {}} = {}) {
    let active = false, timer = null, controller = null, token = 0;
    let fullMap = null, cells = null;

    async function request(url) {
      const requestController = new AbortController();
      controller = requestController;
      const started = typeof win.performance?.now === 'function' ? win.performance.now() : Date.now();
      const timeout = win.setTimeout(() => requestController.abort(), 3000);
      try {
        const response = await fetchMap(url, {cache:'no-store',signal:requestController.signal});
        if (!response.ok) throw new Error('全局地图服务不可用');
        const value = await response.json();
        const finished = typeof win.performance?.now === 'function' ? win.performance.now() : Date.now();
        for (const key of ['map_from_odom_age_ms','map_age_ms'])
          if (Number.isFinite(value?.[key])) value[key] += Math.max(0,finished-started);
        return value;
      } finally {
        win.clearTimeout(timeout);
        if (controller === requestController) controller = null;
      }
    }

    async function refresh() {
      if (!active) return;
      const current = token;
      try {
        let meta = await request('/global-map-meta.json');
        if (!active || current !== token) return;
        if (!validHeader(meta)) throw new Error('全局地图状态无效');
        if (!meta.available) {
          fullMap = null; cells = null;
          onUpdate({map:meta,cells:null,online:true});
        } else {
          if (!fullMap || mapKey(meta) !== mapKey(fullMap)) {
            const next = await request('/global-map.json');
            if (!active || current !== token) return;
            if (!validHeader(next) || !next.available || next.domain !== meta.domain ||
                next.generation !== meta.generation) throw new Error('全局地图坐标域变化');
            cells = decode(next);
            fullMap = next;
            // Full snapshot may have advanced while the small metadata request was in flight.
            meta = next;
          }
          onUpdate({map:{...fullMap,...meta,cells_rle:fullMap.cells_rle},cells,online:true});
        }
      } catch (_) {
        if (active && current === token) onUpdate({map:fullMap,cells,online:false,reason:'全局地图服务不可用'});
      } finally {
        if (active && current === token) timer = win.setTimeout(refresh, 250);
      }
    }

    function start() {
      if (active) return;
      active = true; token += 1; refresh();
    }
    function stop() {
      active = false; token += 1;
      if (timer) win.clearTimeout(timer);
      if (controller) controller.abort();
      timer = null;
    }
    return {start,stop,refresh};
  }
  return {decode,createSource};
});
