import assert from 'node:assert/strict';
import http from 'node:http';
import {createRequire} from 'node:module';
import test from 'node:test';
import {createDashboardServer} from '../server.mjs';

const {decode, createSource} = createRequire(import.meta.url)('../public/js/global_map.js');
const example = {v:1,type:'global_mapping',protocol:'slam_toolbox_view_v1',source:'slam_toolbox',
  available:true,status:'building',reason:'SLAM 建图中 · 已启用回环检测',domain:'1:1',generation:'g',revision:1,
  width:2,height:2,resolution_m:0.05,origin:{x:-1,y:-2},cells_rle:[0,1,1,2,2,1],
  counts:{unknown:1,free:2,occupied:1},pose:{x:0,y:0,yaw:0},map_age_ms:100};

test('global map decoder accepts SLAM snapshots and rejects incomplete or excessive grids', () => {
  assert.deepEqual([...decode(example)],[0,1,1,2]);
  for (const bad of [
    {...example,source:'fpga'}, {...example,cells_rle:[0,3]}, {...example,cells_rle:[3,4]},
    {...example,width:2049}, {...example,width:2000,height:2000}, {...example,resolution_m:0},
  ]) assert.throws(() => decode(bad));
});

test('global source polls small metadata and fetches grids only when their revision changes', async () => {
  const timers=new Map(), calls=[], updates=[]; let id=0, now=0, current={...example}, failing=false;
  const window={setTimeout:(fn,delay)=>{timers.set(++id,{fn,delay}); return id;},
    clearTimeout:key=>timers.delete(key),performance:{now:()=>now}};
  const fetchMap=async url=>{
    calls.push(url);
    if (failing) throw new Error('offline');
    now+=20;
    const result={...current};
    if (url.includes('-meta')) delete result.cells_rle;
    return {ok:true,json:async()=>result};
  };
  const source=createSource({window,fetchMap,onUpdate:update=>updates.push(update)});
  const flush=()=>new Promise(resolve=>setImmediate(resolve));
  const tick=async()=>{
    const entry=[...timers.entries()].find(([,timer])=>timer.delay===250);
    assert.ok(entry,'next metadata poll is scheduled');
    timers.delete(entry[0]); entry[1].fn(); await flush();
  };
  source.start(); await flush();
  assert.deepEqual(calls,['/global-map-meta.json','/global-map.json']);
  assert.deepEqual([...updates.at(-1).cells],[0,1,1,2]);
  assert.equal(updates.at(-1).map.map_age_ms,120,'reported map age includes transfer time');
  current={...current,map_from_odom:{x:1,y:2,yaw:.1},map_from_odom_age_ms:5};
  await tick();
  assert.equal(calls.filter(url=>url==='/global-map.json').length,1);
  assert.equal(updates.at(-1).map.map_from_odom.x,1);
  current={...current,revision:2,cells_rle:[2,4]}; await tick();
  assert.equal(calls.filter(url=>url==='/global-map.json').length,2);
  assert.deepEqual([...updates.at(-1).cells],[2,2,2,2]);
  failing=true; await tick();
  assert.equal(updates.at(-1).online,false);
  failing=false; current={...current,available:false,reason:'坐标域已变化'}; await tick();
  assert.equal(updates.at(-1).map.available,false);
  assert.equal(updates.at(-1).cells,null);
  source.stop();
});

test('dashboard proxies the selected Jetson global map and reports source failure', async () => {
  let answer = example;
  const source = http.createServer((req,res) => {
    if (!['/global-map.json','/global-map-meta.json'].includes(req.url) || !answer) {res.writeHead(503).end(); return;}
    const result={...answer}; if (req.url.includes('-meta')) delete result.cells_rle;
    res.writeHead(200,{'Content-Type':'application/json'}).end(JSON.stringify(result));
  });
  await new Promise(resolve=>source.listen(0,'127.0.0.1',resolve));
  const bridge = await createDashboardServer({httpPort:0,udpBindHost:'127.0.0.1',jetsonHost:'127.0.0.1',
    jetsonPort:9,globalMapPort:source.address().port,log:false});
  try {
    const url = `http://127.0.0.1:${bridge.http.address().port}/global-map.json`;
    const first = await fetch(url);
    assert.equal(first.status,200);
    assert.deepEqual(await first.json(),example);
    const metadata=await (await fetch(`http://127.0.0.1:${bridge.http.address().port}/global-map-meta.json`)).json();
    assert.equal(metadata.revision,1);
    assert.equal('cells_rle' in metadata,false);
    answer = {...example,revision:2,cells_rle:[2,4]};
    assert.equal((await (await fetch(url)).json()).revision,2);
    answer = null;
    const failed = await fetch(url);
    assert.equal(failed.status,502);
    assert.match((await failed.json()).error,/不可用/);
  } finally {
    await bridge.close();
    await new Promise(resolve=>source.close(resolve));
  }
});
