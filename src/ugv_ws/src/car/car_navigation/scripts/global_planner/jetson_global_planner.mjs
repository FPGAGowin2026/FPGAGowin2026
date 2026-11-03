// Jetson 内部规划进程：ROS 适配器负责数据和执行器，网页不参与规划。
import crypto from 'node:crypto';
import http from 'node:http';
import readline from 'node:readline';
import {performance} from 'node:perf_hooks';
import {FpgaGridMap} from './fpga_grid_map.mjs';
import {GlobalNavigation} from './global_navigation.mjs';
import {runWorker} from './route_runner.mjs';

const localMap = new FpgaGridMap();
let nav = null, navAt = -Infinity, executorReady = false, nextTask = 0, navDomain = null;
let tickRunning = false;
const u32 = n => Number.isInteger(n) && n >= 0 && n <= 0xffffffff;
const newer = (a,b) => { const d = (a-b) >>> 0; return d !== 0 && d < 0x80000000; };
const send = message => process.stdout.write(`${JSON.stringify(message)}\n`);
const idle = () => ({status:'idle',active:false,goal_index:0,goal_count:0,goals:[],route:[]});

function fetchMap() {
  return new Promise((resolve, reject) => {
    const request = http.get({host:'127.0.0.1',port:8766,path:'/global-map.json',timeout:3000}, response => {
      if (response.statusCode !== 200) { response.resume(); reject(new Error('SLAM 地图服务不可用')); return; }
      const chunks = []; let size = 0;
      response.on('data', chunk => {
        size += chunk.length;
        if (size > 8 * 1024 * 1024) response.destroy(new Error('SLAM 地图超过容量'));
        else chunks.push(chunk);
      });
      response.on('end', () => {
        const bytes = Buffer.concat(chunks);
        const input = bytes.buffer.slice(bytes.byteOffset,bytes.byteOffset+bytes.byteLength);
        runWorker(input,'decode').then(resolve,reject);
      });
      response.on('error', reject);
    });
    request.on('timeout', () => request.destroy(new Error('SLAM 地图请求超时')));
    request.on('error', reject);
  });
}

function context() {
  const now = performance.now(), fresh = !!nav && now-navAt <= 750;
  return {online:fresh,ready:fresh && executorReady && u32(nav.last_task_id) &&
      nav.active_task === false && nav.pose_valid === true,
    ownerReady:true, emergency:nav?.emergency_stop === true,
    nav:fresh ? nav : null,localMap:localMap.snapshot(now)};
}

function sendLocal(action, fields) {
  const current = context();
  // A stale state must never start motion, but should not suppress a stop request
  // for the last known session; the executor rejects a changed boot/domain.
  const source = action === 'local_start' ? current.nav : nav;
  if (!executorReady || !source || !u32(source.boot_id) || !u32(source.odom_domain) ||
      !u32(source.last_task_id)) return null;
  const watermark = source.last_task_id;
  if (u32(watermark) && (!nextTask || newer(watermark,nextTask))) nextTask = watermark;
  nextTask = (nextTask+1) >>> 0;
  if (!nextTask) nextTask = 1;
  const requestId = crypto.randomUUID();
  send({kind:'executor_command',request_id:requestId,action,task_id:nextTask,
    boot_id:source.boot_id,odom_domain:source.odom_domain,fields});
  return {requestId,taskId:nextTask};
}

const controller = new GlobalNavigation({getContext:context,fetchMap,sendLocal,
  plan:input=>runWorker(input,'plan'),checkSegment:input=>runWorker(input,'segment'),
  now:()=>performance.now(),onUpdate:state=>send({kind:'state',state})});

function executorResult(message) {
  controller.acceptResult({request_id:message.request_id,action:message.action,
    success:message.success === true,unknown:message.unknown === true,
    ...(message.action === 'local_start' ? {task_id:message.task_id} :
      {transaction_task_id:message.task_id})});
}

async function command(message) {
  const {id,request} = message;
  try {
    if (!request || !['start','stop','cancel'].includes(request.action)) throw new Error('导航操作无效');
    let state;
    if (request.action === 'start') state = await controller.start('ros',request);
    else state = controller.stop(request.action === 'stop' ? '用户停止导航' : '用户取消导航') ?? idle();
    send({kind:'command_result',id,success:true,message:request.action === 'start' ?
      '目标已接收；规划和执行状态以反馈为准' : '已请求停止；停车状态以反馈为准',state});
  } catch (error) {
    send({kind:'command_result',id,success:false,message:String(error.message || error),state:controller.snapshot()});
  }
}

const input = readline.createInterface({input:process.stdin,crlfDelay:Infinity});
input.on('line', line => {
  let message;
  try { message = JSON.parse(line); } catch { return; }
  const now = performance.now();
  switch (message.kind) {
    case 'navigation': {
      const value = message.data;
      if (!value || !u32(value.boot_id) || !value.boot_id || !u32(value.odom_domain) || !value.odom_domain ||
          !u32(value.now_us) || !['nav_grid_v1','jetson_nav_v1'].includes(value.protocol)) return;
      const domain = `${value.boot_id}:${value.odom_domain}`;
      // 不允许同一设备时间域内迟到的状态把已确认的执行/停止状态倒退。
      if (domain === navDomain && nav && ((value.now_us-nav.now_us)|0) < 0) return;
      if (domain !== navDomain) { navDomain=domain; nextTask=u32(value.last_task_id) ? value.last_task_id : 0; }
      nav = value; navAt = now; localMap.consume('navigation',value,now);
      break;
    }
    case 'fusion': localMap.consume('fusion',message.data,now); break;
    case 'local_grid': localMap.consume('local_grid',message.data,now); break;
    case 'executor_ready': executorReady = message.ready === true; break;
    case 'executor_result': executorResult(message); break;
    case 'command': void command(message); break;
    default: break;
  }
});
input.on('close', () => process.exit(0));
setInterval(() => {
  if (tickRunning) return;
  tickRunning = true;
  Promise.resolve(controller.tick()).catch(error => {
    send({kind:'error',message:String(error.message || error)});
    controller.stop('全局调度发生异常');
  }).finally(() => {tickRunning=false;});
}, 200);
send({kind:'state',state:idle()});
