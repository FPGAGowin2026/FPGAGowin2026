'use strict';

const assert = require('node:assert/strict');
const test = require('node:test');
const dashboard = require('../public/js/dashboard_network.js');

const SERIES_COLORS = ['#67e8f9', '#60a5fa', '#34d399', '#fbbf24'];

function makeContext() {
  const context = {
    strokes:[],
    texts:[],
    currentPath:[],
    strokeStyle:'',
    beginPath() { this.currentPath=[]; },
    moveTo(x,y) { this.currentPath.push({op:'moveTo',x,y}); },
    lineTo(x,y) { this.currentPath.push({op:'lineTo',x,y}); },
    stroke() { this.strokes.push({color:this.strokeStyle,path:this.currentPath.slice()}); },
    clearRect() { this.strokes=[]; this.texts=[]; },
    setTransform() {},
    resetTransform() {},
    fillText(text) { this.texts.push(String(text)); },
    fillRect() {}
  };
  return context;
}

function makeElement(id, width=640, height=190) {
  const context = makeContext();
  const element = {
    id,
    textContent:'',
    width:0,
    height:0,
    clientWidth:width,
    clientHeight:height,
    parentElement:{id:`${id}-container`, clientWidth:width, clientHeight:height},
    getContext:() => context,
    getBoundingClientRect:() => ({width,height})
  };
  return {element, context};
}

function makePresentation({resizeObserver=false}={}) {
  const elements = new Map();
  const ids = ['wbTransport','wbConnection','wbSimulation','wbActualMode','telemetryTrendStatus'];
  for (const id of ids) elements.set(id, {id,textContent:'',dataset:{}});
  const trend = makeElement('telemetryTrend',640,190);
  const radar = makeElement('radarCloud',400,240);
  elements.set('telemetryTrend',trend.element);
  elements.set('radarCloud',radar.element);
  const document = {getElementById:id => elements.get(id) || null};
  const observerState = {instances:[]};
  class FakeResizeObserver {
    constructor(callback) { this.callback=callback; this.targets=[]; observerState.instances.push(this); }
    observe(target) { this.targets.push(target); }
    emit(sizes) {
      this.callback(sizes.map(({target,width,height})=>({target,contentRect:{width,height}})));
    }
  }
  const window = {devicePixelRatio:1};
  if (resizeObserver) window.ResizeObserver=FakeResizeObserver;
  let redrawCount=0;
  const presentation = dashboard.createWorkbenchPresentation({document,window,redraw:()=>redrawCount++});
  return {presentation,elements,trend,radar,observerState,get redrawCount(){return redrawCount;}};
}

function snapshot(overrides={}) {
  return {
    source:'network', connected:true, simulation:null,
    transportText:'网络 UDP', connectionText:'UDP 已连接', actualModeText:'实际模式未知',
    wheels:[10,20,30,40], fresh:true, nowMs:1000, sampleAt:1000,
    ...overrides
  };
}

function linePoints(context, color) {
  return context.strokes.filter(item=>item.color===color).flatMap(item=>item.path);
}

test('workbench presentation updates header state and deduplicates repeated receive timestamps', () => {
  const page=makePresentation();
  const {presentation,elements,trend}=page;
  presentation.update(snapshot());
  assert.equal(elements.get('wbTransport').textContent,'网络 UDP');
  assert.equal(elements.get('wbConnection').textContent,'UDP 已连接');
  assert.equal(elements.get('wbSimulation').textContent,'模拟状态未知');
  assert.equal(elements.get('wbSimulation').dataset.state,'unknown');
  assert.equal(elements.get('wbConnection').dataset.state,'fresh');
  assert.equal(elements.get('wbActualMode').textContent,'实际模式未知');
  assert.match(elements.get('telemetryTrendStatus').textContent,/mm\/s/);
  assert.equal(linePoints(trend.context,SERIES_COLORS[0]).filter(point=>point.op==='lineTo').length,0);

  presentation.update(snapshot({nowMs:1250,sampleAt:1000,wheels:[999,999,999,999]}));
  assert.equal(linePoints(trend.context,SERIES_COLORS[0]).filter(point=>point.op==='lineTo').length,0,
    'a repeated sampleAt updates presentation state without adding a second measured point');

  presentation.update(snapshot({nowMs:1300,sampleAt:1250,wheels:[12,22,32,42],simulation:false}));
  assert.equal(elements.get('wbSimulation').textContent,'非模拟');
  assert.equal(elements.get('wbSimulation').dataset.state,'live');
  for (const color of SERIES_COLORS)
    assert.equal(linePoints(trend.context,color).filter(point=>point.op==='lineTo').length,1);
});

test('missing wheels, stale samples and disconnects create gaps instead of zeroes or bridged lines', () => {
  const {presentation,trend,elements}=makePresentation();
  presentation.update(snapshot({sampleAt:1000,nowMs:1000,wheels:[10,20,30,40]}));
  presentation.update(snapshot({sampleAt:1250,nowMs:1250,wheels:[15,null,35,45]}));
  assert.equal(linePoints(trend.context,SERIES_COLORS[0]).filter(point=>point.op==='lineTo').length,1);
  assert.equal(linePoints(trend.context,SERIES_COLORS[1]).filter(point=>point.op==='lineTo').length,0,
    'the missing B wheel value breaks only the B series');

  presentation.update(snapshot({sampleAt:1250,nowMs:2301,fresh:true}));
  assert.match(elements.get('telemetryTrendStatus').textContent,/过期/,
    'a sample older than one second is stale even if the caller has not flipped fresh yet');
  presentation.update(snapshot({sampleAt:2400,nowMs:2400,wheels:[25,35,45,55]}));
  assert.equal(linePoints(trend.context,SERIES_COLORS[0]).filter(point=>point.op==='lineTo').length,1,
    'the new sample starts a fresh segment after a stale gap');
  assert.equal(linePoints(trend.context,SERIES_COLORS[1]).filter(point=>point.op==='lineTo').length,0);

  presentation.update(snapshot({sampleAt:2500,nowMs:2500,connected:false,wheels:[0,0,0,0]}));
  assert.match(elements.get('telemetryTrendStatus').textContent,/过期/);
  assert.equal(elements.get('wbConnection').textContent,'UDP 已连接',
    'caller-provided connection wording remains authoritative');
  assert.equal(elements.get('wbConnection').dataset.state,'stale');
  presentation.update(snapshot({sampleAt:2600,nowMs:2600,wheels:null}));
  assert.match(elements.get('telemetryTrendStatus').textContent,/等待新鲜轮速/);
});

test('reset clears trend segments and repeated stale samples stay cleared', () => {
  const {presentation,trend}=makePresentation();
  presentation.update(snapshot({sampleAt:1000,nowMs:1000}));
  presentation.update(snapshot({sampleAt:1100,nowMs:1100,wheels:[11,21,31,41]}));
  assert.equal(linePoints(trend.context,SERIES_COLORS[0]).filter(point=>point.op==='lineTo').length,1);
  presentation.reset();
  assert.equal(linePoints(trend.context,SERIES_COLORS[0]).filter(point=>point.op==='lineTo').length,0);
  presentation.update(snapshot({sampleAt:1100,nowMs:1300,wheels:[90,90,90,90]}));
  assert.equal(linePoints(trend.context,SERIES_COLORS[0]).filter(point=>point.op==='lineTo').length,0,
    'reset does not recreate a cleared point from the same receive timestamp');
  presentation.update(snapshot({sampleAt:1400,nowMs:1400,wheels:[14,24,34,44]}));
  assert.equal(linePoints(trend.context,SERIES_COLORS[0]).filter(point=>point.op==='lineTo').length,0,
    'the first point after reset starts a new segment without connecting to the discarded history');
});

test('long receive gaps and repeated invalid updates break trend continuity', () => {
  const {presentation,trend,elements}=makePresentation();
  presentation.update(snapshot({sampleAt:1000,nowMs:1000,wheels:[10,20,30,40]}));
  presentation.update(snapshot({sampleAt:2201,nowMs:2201,wheels:[20,30,40,50]}));
  assert.equal(linePoints(trend.context,SERIES_COLORS[0]).filter(point=>point.op==='lineTo').length,0,
    'newly received samples separated by more than one second begin a new segment');
  presentation.update(snapshot({sampleAt:2250,nowMs:2250,wheels:[21,31,41,51]}));
  assert.equal(linePoints(trend.context,SERIES_COLORS[0]).filter(point=>point.op==='lineTo').length,1,
    'samples resume connecting after the fresh segment begins');

  presentation.update(snapshot({sampleAt:2300,nowMs:2300,wheels:null}));
  presentation.update(snapshot({sampleAt:2400,nowMs:2400,wheels:[null,null,null,null]}));
  assert.match(elements.get('telemetryTrendStatus').textContent,/等待新鲜轮速/);
  presentation.update(snapshot({sampleAt:2500,nowMs:2500,wheels:[25,35,45,55]}));
  assert.equal(linePoints(trend.context,SERIES_COLORS[0]).filter(point=>point.op==='lineTo').length,1,
    'repeated invalid updates keep one gap until a new fresh measurement arrives');
});

test('software simulation and caller-provided network transport text are explicit in the header', () => {
  const {presentation,elements}=makePresentation();
  presentation.update(snapshot({simulation:true,transportText:'网络 UDP'}));
  assert.equal(elements.get('wbTransport').textContent,'网络 UDP');
  assert.equal(elements.get('wbSimulation').textContent,'软件模拟');
  assert.equal(elements.get('wbSimulation').dataset.state,'simulation');
  presentation.update(snapshot({simulation:null,transportText:'网络 UDP',connected:false}));
  assert.equal(elements.get('wbTransport').textContent,'网络 UDP');
  assert.equal(elements.get('wbSimulation').textContent,'模拟状态未知');
  assert.equal(elements.get('wbSimulation').dataset.state,'unknown');
  assert.equal(elements.get('wbConnection').dataset.state,'stale');
});

test('trend retains only the recent 20 seconds and resize observation redraws only on dimension changes', () => {
  const page=makePresentation({resizeObserver:true});
  const {presentation,trend,observerState}=page;
  presentation.update(snapshot({sampleAt:1000,nowMs:1000}));
  presentation.update(snapshot({sampleAt:1100,nowMs:1100,wheels:[11,21,31,41]}));
  assert.equal(linePoints(trend.context,SERIES_COLORS[0]).filter(point=>point.op==='lineTo').length,1);

  presentation.update(snapshot({sampleAt:21101,nowMs:21101,wheels:[21,31,41,51]}));
  assert.equal(linePoints(trend.context,SERIES_COLORS[0]).filter(point=>point.op==='lineTo').length,0,
    'the sample at 1000 ms has left the 20 second window');
  const observer=observerState.instances[0];
  assert.ok(observer);
  assert.equal(observer.targets.length,2);
  const entries=observer.targets.map(target=>({target,width:400,height:240}));
  observer.emit(entries);
  assert.equal(page.redrawCount,1);
  observer.emit(entries);
  assert.equal(page.redrawCount,1,'unchanged container measurements do not trigger redraw loops');
  observer.emit([{target:observer.targets[0],width:420,height:240}]);
  assert.equal(page.redrawCount,2);
  presentation.resize();
  assert.equal(page.redrawCount,2,'explicit chart resize does not call the host redraw callback');
});
