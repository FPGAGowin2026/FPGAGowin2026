import assert from 'node:assert/strict';
import test from 'node:test';
import {setTimeout as delay} from 'node:timers/promises';
import {createDashboardServer} from '../server.mjs';
import {createSimulator} from '../simulator.mjs';

test('device-format route history and fused trace survive task termination',async()=>{
  const simulator=await createSimulator({bind:'127.0.0.1',port:0,quiet:true,
    navigationArchitecture:'grid',statusIntervalMs:100,telemetryIntervalMs:100});
  Object.assign(simulator.navigationTask,{task_id:77,last_task_id:77,status:3,mode:2,
    expiry_us:0,goal_x_q16:Math.round(1.9*65536),goal_y_q16:Math.round(-.12*65536)});
  const bridge=await createDashboardServer({bindHost:'127.0.0.1',httpPort:0,udpBindHost:'127.0.0.1',
    jetsonHost:'127.0.0.1',jetsonPort:simulator.socket.address().port,simulation:true,log:false});
  try {
    assert.equal(bridge.target.address,'127.0.0.1');
    const url=`http://127.0.0.1:${bridge.http.address().port}/map.json`;
    const until=async predicate=>{
      const end=Date.now()+5000;
      while (Date.now()<end) {
        const snapshot=await (await fetch(url)).json();
        if (predicate(snapshot)) return snapshot;
        await delay(50);
      }
      throw Error('simulated map did not reach expected state');
    };
    const active=await until(map=>map.status==='building' && map.active_goal?.task_id===77 &&
      map.planned_route?.task_id===77 && map.planned_history?.length>0 && map.trajectory?.length>0);
    assert.equal(active.source,'fpga');
    assert.equal(active.planned_history[0].task_id,77);
    assert.equal(active.planned_history[0].points.length,2);
    simulator.navigationTask.status=7;
    const stopped=await until(map=>map.planned_route===null && map.active_goal===null &&
      map.planned_history?.some(segment=>segment.task_id===77));
    assert.ok(stopped.trajectory.length>0);
    assert.deepEqual(stopped.planned_history[0].points,active.planned_history[0].points);
  } finally {
    await bridge.close();
    await simulator.close();
  }
});
