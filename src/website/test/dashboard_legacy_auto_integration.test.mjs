import assert from 'node:assert/strict';
import test from 'node:test';
import { setTimeout as delay } from 'node:timers/promises';
import { WebSocket } from 'ws';
import dashboard from '../public/js/dashboard_network.js';
import { createDashboardServer } from '../server.mjs';
import { createSimulator } from '../simulator.mjs';

async function waitUntil(predicate, label) {
  const deadline = Date.now() + 3000;
  while (!predicate()) {
    if (Date.now() >= deadline) assert.fail(`timed out waiting for ${label}`);
    await delay(5);
  }
}

test('legacy adapter auto requests reach the real bridge and auto(false) sends zero to the simulator', async t => {
  const simulator = await createSimulator({ bind:'127.0.0.1', port:0, quiet:true });
  let bridge, adapter;
  t.after(async () => {
    adapter?.closeSocket();
    if (bridge) await bridge.close();
    await simulator.close();
  });
  bridge = await createDashboardServer({
    bindHost:'127.0.0.1', httpPort:0, udpBindHost:'127.0.0.1',
    jetsonHost:'127.0.0.1', jetsonPort:simulator.socket.address().port,
    simulation:true, log:false,
  });
  const port = bridge.http.address().port;
  class LocalWebSocket extends WebSocket {
    constructor(url) { super(url, { origin:`http://127.0.0.1:${port}` }); }
  }
  const events = [];
  adapter = new dashboard.DashboardNetworkAdapter({ WebSocketImpl:LocalWebSocket, onEvent:event => events.push(event) });
  adapter.connect(`ws://127.0.0.1:${port}/ws`);
  await waitUntil(() => adapter.state?.jetson?.online && adapter.state?.fpga?.ready &&
    [...bridge.clients.values()].some(client => client.deliveryAckAt !== null), 'ready simulator and page consumption');

  assert.equal(adapter.arm(), true);
  await waitUntil(() => events.some(event => event.event === 'armed'), 'manual authorization');
  assert.equal(adapter.drive(0, 0, 0), true);
  assert.equal(adapter.auto(true), true);
  await waitUntil(() => events.some(event => event.event === 'auto_rejected'), 'rejected legacy auto enable');
  assert.equal(simulator.command.auto, false);

  assert.equal(adapter.drive(40, 0, 0), true);
  const moving = await simulator.waitForCommand(snapshot => snapshot.command.velocity[0] === 40);
  const stopped = simulator.waitForCommand(snapshot => snapshot.command.seq > moving.command.seq &&
    snapshot.command.velocity.every(value => value === 0));
  assert.equal(adapter.auto(false), true);
  const zero = await stopped;
  await waitUntil(() => events.some(event => event.event === 'auto_disabled'), 'legacy auto disable');
  assert.equal(zero.command.auto, false);
  assert.equal(bridge.owner?.id, adapter.hello.client_id, 'legacy disable sends zero without releasing ownership');
});
