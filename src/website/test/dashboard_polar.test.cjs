const test = require('node:test');
const assert = require('node:assert/strict');
const dashboard = require('../public/js/dashboard_network.js');

test('polar-v2 error estimates retain mm/mrad semantics and never use count words as covariance', () => {
  const result = dashboard.poseUncertainty({flags:0xd0004001,
    error_estimate:[30,40,50], covariance_diag:[900,1600,2500,0x12345678,0x23456789]});
  assert.equal(result.polar, true);
  assert.equal(result.label, '估计误差');
  assert.deepEqual(result.values, [0.03,0.04,0.05,null,null,null,null]);
  assert.deepEqual(dashboard.poseUncertainty({flags:0x4001,covariance_diag:[1,1,1]}).values,
    [null,null,null,null,null,null,null]);
});

test('legacy covariance stays in SI and an absent or invalid estimate stays unavailable', () => {
  assert.deepEqual(dashboard.poseUncertainty({flags:1,covariance_diag:[4,9,0.04]}).values,
    [2,3,0.2,null,null,null,null]);
  assert.deepEqual(dashboard.poseUncertainty({flags:0x8001,covariance_diag:[4,9,0.04]}).values,
    [2,3,0.2,null,null,null,null]);
  assert.deepEqual(dashboard.poseUncertainty({flags:0x4001,error_estimate:[-1,NaN,10]}).values,
    [null,null,0.01,null,null,null,null]);
  assert.equal(dashboard.poseUncertainty(null).polar, false);
});

test('grid-v1 retains estimate units instead of interpreting counters as covariance', () => {
  const result=dashboard.poseUncertainty({flags:0xd000c001,error_estimate:[30,40,50],
    covariance_diag:[900,1600,2500,0xffffffff,7]});
  assert.equal(result.label,'估计误差');
  assert.deepEqual(result.values,[0.03,0.04,0.05,null,null,null,null]);
});
