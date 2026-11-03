import assert from 'node:assert/strict';
import test from 'node:test';
import {parseSemanticCommand} from '../semantic_command.mjs';

test('中文语义复用视觉任务、局部目标和停车命令', () => {
  assert.deepEqual(parseSemanticCommand('找到红色水瓶并移动到它附近。'), {
    type:'mission_command', action:'start', target_id:'', class_id:'39', behavior:'search', color:'red',
  });
  assert.equal(parseSemanticCommand('跟随人员').behavior, 'follow');
  assert.equal(parseSemanticCommand('靠近杯子').class_id, '41');
  assert.deepEqual(parseSemanticCommand('前往坐标（1.5，-2）'), {
    type:'local_task', action:'start', goal_x_m:1.5, goal_y_m:-2,
  });
  assert.equal(parseSemanticCommand('停止导航。').type, 'stop');
});

test('未知地点、否定句和不支持的颜色不转成运动任务', () => {
  for (const text of ['去厨房', '不要跟随人员', '寻找蓝色水瓶', '找水瓶然后停止', '寻找水瓶和杯子', '寻找桌子旁边的水瓶']) {
    assert.throws(() => parseSemanticCommand(text));
  }
});
