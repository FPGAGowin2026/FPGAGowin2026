import {Worker} from 'node:worker_threads';

// 地图解析和路径搜索位于独立线程，避免阻塞 ROS 命令及任务状态。
export function runWorker(input, operation, {timeoutMs=5000, WorkerClass=Worker}={}) {
  return new Promise((resolve, reject) => {
    const worker = new WorkerClass(new URL('./global_route_worker.mjs', import.meta.url),
      {workerData:{operation,input}, ...(operation === 'decode' ? {transferList:[input]} : {})});
    let settled = false;
    const finish = (error, result) => {
      if (settled) return;
      settled = true;
      clearTimeout(timer);
      if (error) reject(error); else resolve(result);
    };
    const timer = setTimeout(() => {
      finish(new Error(operation === 'decode' ? '全局地图解析超时' : '全局路线搜索超时'));
      void worker.terminate();
    }, timeoutMs);
    worker.once('message', result => {
      if (!result?.error) { finish(null,result); return; }
      const error = new Error(result.error);
      if (result.domain) error.domain = result.domain;
      if (result.generation) error.generation = result.generation;
      finish(error);
    });
    worker.once('error', error => finish(error));
    worker.once('exit', code => {
      if (!settled) finish(new Error(`全局路线线程退出，未返回结果 (${code})`));
    });
  });
}
