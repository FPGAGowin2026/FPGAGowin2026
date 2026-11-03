/* 局部任务参数校验，供网页和本机服务共用。 */
(function (root, factory) {
  const api = factory();
  if (typeof module === 'object' && module.exports) module.exports = api;
  if (root) root.BX72LocalTaskValidation = api;
})(typeof globalThis === 'undefined' ? this : globalThis, function () {
  'use strict';

  const rules = [
    ['goal_x_m', '目标 X (m)', '有限数值，范围 -32767.999 至 32767.999', v => Number.isFinite(v) && Math.abs(v) <= 32767.999],
    ['goal_y_m', '目标 Y (m)', '有限数值，范围 -32767.999 至 32767.999', v => Number.isFinite(v) && Math.abs(v) <= 32767.999],
    ['hold_yaw_rad', '保持航向 (rad)', '有限数值，范围 -π 至 π', v => Number.isFinite(v) && Math.abs(v) <= Math.PI],
    ['final_yaw_rad', '终点航向 (rad)', '有限数值，范围 -π 至 π', v => Number.isFinite(v) && Math.abs(v) <= Math.PI],
    ['max_speed_mm_s', '最大速度 (mm/s)', '整数，范围 1 至 400', v => Number.isInteger(v) && v >= 1 && v <= 400],
    ['max_accel_mm_s2', '最大加速度 (mm/s²)', '整数，范围 1 至 1000', v => Number.isInteger(v) && v >= 1 && v <= 1000],
    ['tolerance_mm', '位置容差 (mm)', '整数，范围 1 至 500', v => Number.isInteger(v) && v >= 1 && v <= 500],
    ['yaw_tolerance_mrad', '航向容差 (mrad)', '整数，范围 1 至 1000', v => Number.isInteger(v) && v >= 1 && v <= 1000],
    ['duration_s', '任务期限 (s)', '有限数值，必须大于 0 且不超过 120', v => Number.isFinite(v) && v > 0 && v <= 120],
    ['mode', '任务模式', 'cooperative 或 independent', v => v === 'cooperative' || v === 'independent'],
    ['allow_disconnect', '允许断链', '布尔值', v => v === undefined || typeof v === 'boolean'],
    ['position_only', '仅位置到达', '布尔值', v => v === undefined || typeof v === 'boolean'],
  ];

  function describe(value) {
    if (value === undefined) return '缺失';
    if (typeof value === 'number' && !Number.isFinite(value)) return String(value);
    if (value === null || ['number', 'boolean'].includes(typeof value)) return String(value);
    if (typeof value === 'string') return JSON.stringify(value.slice(0, 80));
    return '非标量';
  }

  function validate(fields) {
    for (const [field, label, requirement, valid] of rules) {
      const value = fields?.[field];
      if (!valid(value)) return {field, message:`${label}（${field}）无效：收到 ${describe(value)}；要求${requirement}。`};
    }
    return null;
  }

  return {validate};
});
