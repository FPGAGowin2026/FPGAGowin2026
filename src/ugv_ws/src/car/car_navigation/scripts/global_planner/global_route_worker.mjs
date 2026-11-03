import {parentPort, workerData} from 'node:worker_threads';
import {checkGlobalSegment, decodeGlobalMap, planGlobalRoute} from './global_route.mjs';

let snapshot;
try {
  if (workerData.operation === 'decode') {
    snapshot = JSON.parse(Buffer.from(workerData.input).toString('utf8'));
    const map = decodeGlobalMap(snapshot);
    parentPort.postMessage(map, [map.cells.buffer]);
  } else {
    parentPort.postMessage(workerData.operation === 'segment' ?
      checkGlobalSegment(workerData.input) : planGlobalRoute(workerData.input));
  }
} catch (error) {
  parentPort.postMessage({error:error.message,domain:snapshot?.domain,generation:snapshot?.generation});
}
