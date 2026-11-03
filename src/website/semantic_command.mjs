const OBJECTS = new Map([
  ['水瓶','39'], ['瓶子','39'], ['饮料瓶','39'], ['杯子','41'], ['水杯','41'],
  ['椅子','56'], ['人员','0'], ['行人','0'], ['人','0'], ['猫','15'], ['狗','16'],
]);

export function parseSemanticCommand(text) {
  if (typeof text !== 'string' || !text.trim() || text.length > 200) {
    throw new Error('请输入不超过200字的任务指令');
  }
  const command = text.trim().replace(/[。！!？?，,；;\s]+$/u, '')
    .replace(/^(?:请帮我|帮我|请|机器人)[，,\s]*/u, '').trim();
  if (/^(?:停止(?:导航|跟随|跟踪|任务|移动)?|停车|停下|取消(?:导航|任务|跟随)?)$/u.test(command)) {
    return {type:'stop', reason:'语音停车'};
  }
  const point = command.match(/^(?:前往|去|移动到|导航到)(?:坐标|位置)?\s*[（(]?\s*([+-]?\d+(?:\.\d+)?)\s*[,，]\s*([+-]?\d+(?:\.\d+)?)\s*[）)]?$/u);
  if (point) {
    const x = Number(point[1]), y = Number(point[2]);
    return {type:'local_task', action:'start', goal_x_m:x, goal_y_m:y};
  }
  const action = command.match(/^(跟随|跟踪|寻找|搜索|找到|找|靠近|接近|移动到)(?:一个|那个|这个|指定的)?(红色?|)(水瓶|瓶子|饮料瓶|杯子|水杯|椅子|人员|行人|人|猫|狗)(?:附近|旁边)?(?:并(?:移动到|靠近|接近)(?:它|目标)?(?:的)?(?:附近|旁边))?$/u);
  if (!action) {
    throw new Error('暂不支持这条指令；可说“寻找红色水瓶”“跟随人员”“前往坐标1，2”或“停止导航”');
  }
  const behavior = /^(跟随|跟踪)$/u.test(action[1]) ? 'follow' :
    /^(靠近|接近|移动到)$/u.test(action[1]) ? 'approach' : 'search';
  return {type:'mission_command', action:'start', target_id:'', class_id:OBJECTS.get(action[3]),
    behavior, color:action[2] ? 'red' : ''};
}
