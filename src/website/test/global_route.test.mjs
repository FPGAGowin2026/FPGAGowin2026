import test from 'node:test';
import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import {checkGlobalSegment,decodeGlobalMap,planGlobalRoute} from '../../ugv_ws/src/car/car_navigation/scripts/global_planner/global_route.mjs';

const IDENTITY={map_anchor_x_mm:0,map_anchor_y_mm:0,odom_anchor_x_mm:0,
  odom_anchor_y_mm:0,yaw_offset_mrad:0};
function rle(cells) {
  const result=[];
  for(let i=0;i<cells.length;) {
    let end=i+1;
    while(end<cells.length && cells[end]===cells[i])end++;
    result.push(cells[i],end-i);i=end;
  }
  return result;
}
function snapshot({width=80,height=80,resolution=.05,origin={x:0,y:0},
                   cells=new Uint8Array(width*height).fill(1),
                   transform={x:0,y:0,yaw:0}}={}) {
  return {v:1,type:'global_mapping',protocol:'slam_toolbox_view_v1',source:'slam_toolbox',
    available:true,status:'building',domain:'7:3',generation:'generation-1',revision:1,
    width,height,resolution_m:resolution,origin,cells_rle:rle(cells),
    map_from_odom:transform,pose:{x:1,y:1,yaw:0}};
}
function local({cells=new Uint8Array(4096),origin={x:0,y:0},transform=IDENTITY}={}) {
  return {v:1,type:'mapping',protocol:'fpga_grid_view_v1',source:'fpga',
    available:true,status:'building',domain:'7:3',width:64,height:64,resolution_m:.05,
    origin,grid_from_odom:transform,cells_rle:rle(cells)};
}
const centre=(x,y,r=.05,origin={x:0,y:0})=>({x:origin.x+(x+.5)*r,y:origin.y+(y+.5)*r});

test('未知区可探索，已知占据仍绕行，当前实际位置可作为起点',()=>{
  const width=80,height=80,cells=new Uint8Array(width*height);
  const start=centre(10,30),goal=centre(70,30);
  cells[30*width+10]=2;
  cells[30*width+40]=2;
  const map=decodeGlobalMap(snapshot({width,height,cells}));
  const route=planGlobalRoute({map,start,goal,radius:0,currentSegment:[start,goal]});
  assert.equal(route.reason,null);
  assert.equal(route.segmentClear,false,'实际道路上的已知占据不能因未知可探索而消失');
  assert.ok(route.path.length>2,'绕过已知占据格');
  assert.deepEqual(route.path[0],start);
  assert.deepEqual(route.path.at(-1),goal);
  assert.equal(map.cells[30*width+10],2,'起点豁免不修改原始地图');
  assert.equal(planGlobalRoute({map,start,goal:centre(40,30),radius:0}).reason,'goal_blocked');
});

test('现场快照的两个目标可穿行未知区，周边已知占据仍用于绕行',()=>{
  const observed=JSON.parse(readFileSync(new URL('./fixtures/field_map_20261102.json',import.meta.url),'utf8'));
  const map=decodeGlobalMap(observed.global),start={x:map.pose.x,y:map.pose.y};
  for(const goal of observed.goals) {
    const route=planGlobalRoute({map,localMap:observed.local,start,goal,radius:0});
    assert.equal(route.reason,null);
    assert.deepEqual(route.path[0],start);
    assert.deepEqual(route.path.at(-1),goal);
    for(let i=1;i<route.path.length;i++)
      assert.equal(checkGlobalSegment({map,localMap:observed.local,
        segment:[route.path[i-1],route.path[i]],radius:0}).segmentClear,true);
  }
});

test('地图边界外的前后左右目标可临时扩展未知区规划，不改变原图',()=>{
  const map=decodeGlobalMap(snapshot({width:30,height:30,resolution:.1}));
  const original=map.cells.slice(),origin={...map.origin},start={x:1.5,y:1.5};
  for(const goal of [{x:-1,y:1.5},{x:4,y:1.5},{x:1.5,y:-1},{x:1.5,y:4}]) {
    const result=planGlobalRoute({map,start,goal,currentSegment:[start,goal]});
    assert.equal(result.reason,null);
    assert.deepEqual(result.path[0],start);
    assert.deepEqual(result.path.at(-1),goal);
    assert.equal(result.segmentClear,true);
    assert.equal(checkGlobalSegment({map,segment:[start,goal]}).segmentClear,true);
    const reverse=planGlobalRoute({map,start:goal,goal:start});
    assert.equal(reverse.reason,null);
    assert.deepEqual(reverse.path[0],goal);
    assert.deepEqual(reverse.path.at(-1),start);
  }
  assert.equal(planGlobalRoute({map,start:{x:-1,y:1.5},goal:{x:4,y:1.5}}).reason,null);
  assert.equal(map.width,30);assert.equal(map.height,30);
  assert.deepEqual(map.origin,origin);
  assert.deepEqual(map.cells,original);
});

test('目标在图内而活动段在图外时，规划与线段检查使用相同的扩展范围',()=>{
  const map=decodeGlobalMap(snapshot({width:30,height:30,resolution:.1}));
  const start={x:1.5,y:1.5},goal={x:2,y:1.5},segment=[start,{x:-1,y:1.5}];
  const plan=planGlobalRoute({map,start,goal,currentSegment:segment});
  assert.equal(plan.reason,null);
  assert.equal(plan.segmentClear,true);
  assert.equal(checkGlobalSegment({map,segment}).segmentClear,true);
});

test('车宽跨出地图边界时临时补未知格，并保留图外局部真实障碍',()=>{
  const map=decodeGlobalMap(snapshot({width:20,height:20,resolution:.1}));
  const start={x:.1,y:1},goal={x:.8,y:1};
  const route=planGlobalRoute({map,start,goal,radius:.191,currentSegment:[start,goal]});
  assert.equal(route.reason,null,'未知边界外也可探索');
  assert.equal(route.segmentClear,true);
  assert.equal(checkGlobalSegment({map,segment:[start,goal],radius:.191}).segmentClear,true);
  assert.equal(map.width,20,'扩展只影响本次计算');
  const cells=new Uint8Array(4096).fill(1);
  cells[52*64+31]=2;
  const grid=local({cells,origin:{x:-1.6,y:-1.6}});
  assert.equal(planGlobalRoute({map,localMap:grid,start,goal,radius:.191}).reason,'start_blocked',
    '图外的局部真实占据不能因补未知格而丢失');
});

test('扩展地图仍绕开真实地图中的障碍，超出资源上限明确报告',()=>{
  const cells=new Uint8Array(30*30).fill(1);
  for(let y=0;y<=20;y++)cells[y*30+20]=2;
  const map=decodeGlobalMap(snapshot({width:30,height:30,resolution:.1,cells}));
  const start={x:1.05,y:1.05},goal={x:4.05,y:1.05};
  const route=planGlobalRoute({map,start,goal,currentSegment:[start,goal]});
  assert.equal(route.reason,null);
  assert.equal(route.segmentClear,false);
  assert.ok(route.path.length>2);
  assert.equal(checkGlobalSegment({map,segment:[start,goal]}).segmentClear,false);
  assert.equal(planGlobalRoute({map,start,goal:{x:32767,y:1}}).reason,'map_too_large');
  assert.equal(checkGlobalSegment({map,segment:[start,{x:32767,y:1}]}).reason,'map_too_large');
});

test('扩展到原图外时仍叠加新鲜局部障碍并绕行',()=>{
  const map=decodeGlobalMap(snapshot({width:30,height:30,resolution:.1}));
  const cells=new Uint8Array(4096).fill(1);cells[32*64+22]=2;
  const grid=local({cells,origin:{x:-1.6,y:-1.6}});
  const start={x:.2,y:.025},goal={x:-1,y:.025};
  const route=planGlobalRoute({map,localMap:grid,start,goal,currentSegment:[start,goal]});
  assert.equal(route.reason,null);
  assert.equal(route.segmentClear,false);
  assert.ok(route.path.length>2);
  assert.equal(checkGlobalSegment({map,localMap:grid,segment:[start,goal]}).segmentClear,false);
  for(let i=1;i<route.path.length;i++)
    assert.equal(checkGlobalSegment({map,localMap:grid,
      segment:[route.path[i-1],route.path[i]]}).segmentClear,true);
});

test('部署后前视地图仍可为边界外后方航点规划',()=>{
  const saved=JSON.parse(readFileSync(new URL('./fixtures/front_view_after_deploy.json',import.meta.url),'utf8'));
  const map=decodeGlobalMap(saved.global),start={x:map.pose.x,y:map.pose.y},goal=saved.goals[1];
  const route=planGlobalRoute({map,localMap:saved.local,start,goal});
  assert.equal(route.reason,null);
  assert.deepEqual(route.path[0],start);
  assert.deepEqual(route.path.at(-1),goal);
  assert.equal(checkGlobalSegment({map,localMap:saved.local,segment:route.path.slice(0,2)}).segmentClear,true);
});

function assertContinuousClear(path, snapshot, radius) {
  const cells=decodeGlobalMap(snapshot).cells, {width:w,height:h,resolution_m:r,origin:o}=snapshot;
  for(let p=1;p<path.length;p++) {
    const a=path[p-1],b=path[p],length=Math.hypot(b.x-a.x,b.y-a.y);
    for(let j=0;j<=Math.ceil(length/(r/4));j++) {
      const t=length===0 ? 0 : Math.min(1,j*r/4/length);
      const x=a.x+(b.x-a.x)*t,y=a.y+(b.y-a.y)*t;
      assert.ok(x-o.x>=radius-1e-8 && y-o.y>=radius-1e-8 &&
        o.x+w*r-x>=radius-1e-8 && o.y+h*r-y>=radius-1e-8,'footprint within map');
      const minX=Math.max(0,Math.floor((x-o.x-radius)/r));
      const maxX=Math.min(w-1,Math.floor((x-o.x+radius)/r));
      const minY=Math.max(0,Math.floor((y-o.y-radius)/r));
      const maxY=Math.min(h-1,Math.floor((y-o.y+radius)/r));
      for(let gy=minY;gy<=maxY;gy++)for(let gx=minX;gx<=maxX;gx++) {
        if(cells[gy*w+gx]!==2)continue;
        const left=o.x+gx*r,bottom=o.y+gy*r;
        const dx=Math.max(left-x,0,x-(left+r)),dy=Math.max(bottom-y,0,y-(bottom+r));
        assert.ok(Math.hypot(dx,dy)>radius-1e-8,'footprint cannot intersect occupied or unknown cells');
      }
    }
  }
}

test('decode validates identity, dimensions, transform, pose and complete RLE exactly once',()=>{
  const source=snapshot({width:2,height:2,cells:Uint8Array.from([0,1,1,2])});
  const map=decodeGlobalMap(source);
  assert.deepEqual([...map.cells],[0,1,1,2]);
  assert.equal('cells_rle' in map,false);
  for(const bad of [
    null,
    {...source,v:2},{...source,type:'mapping'},{...source,source:'fpga'},
    {...source,protocol:'wrong'},{...source,available:false},{...source,domain:''},
    {...source,generation:null},{...source,pose:null},{...source,map_from_odom:{x:0,y:0,yaw:NaN}},
    {...source,width:2049},{...source,width:2000,height:2000},{...source,resolution_m:0},
    {...source,cells_rle:[0,3]},{...source,cells_rle:[3,4]},
    {...source,cells_rle:[1,5]},{...source,cells_rle:[1,0]},{...source,cells_rle:[1,4,0]},
  ]) assert.throws(()=>decodeGlobalMap(bad));
});

test('routes around an L wall beyond the local grid without crossing the inflated wall',()=>{
  const width=120,height=100,cells=new Uint8Array(width*height).fill(1);
  for(let y=0;y<=70;y++)cells[y*width+60]=2;
  for(let x=20;x<=60;x++)cells[70*width+x]=2;
  const source=snapshot({width,height,cells}),map=decodeGlobalMap(source);
  const start=centre(30,30),goal=centre(100,30);
  const result=planGlobalRoute({map,localMap:local(),start,goal,radius:.241,currentSegment:[start,goal]});
  assert.equal(result.reason,null);
  assert.equal(result.segmentClear,false);
  assert.deepEqual(result.path[0],start);
  assert.deepEqual(result.path.at(-1),goal);
  assert.ok(result.path.length>=4,'must navigate around both arms of L');
  assert.ok(Math.min(...result.path.map(p=>p.x))<1,'escape left of the horizontal arm');
  assert.ok(Math.max(...result.path.map(p=>p.y))>3.55,'pass above the vertical arm');
  assertContinuousClear(result.path,source,.241);
});

test('dynamic local obstacles force a detour, can close a route, and release it again',()=>{
  const map=decodeGlobalMap(snapshot());
  const start={x:1,y:2.025},goal={x:3,y:2.025};
  let observations=new Uint8Array(4096).fill(1);
  let free=local({cells:observations,origin:{x:.4,y:.4}});
  const baseline=planGlobalRoute({map,localMap:free,start,goal,radius:.241,currentSegment:[start,goal]});
  assert.equal(baseline.reason,null);assert.equal(baseline.segmentClear,true);
  assert.deepEqual(baseline.path,[start,goal]);
  observations=observations.slice();
  observations[32*64+32]=2;
  let blocked=local({cells:observations,origin:{x:.4,y:.4}});
  const detour=planGlobalRoute({map,localMap:blocked,start,goal,radius:.241,currentSegment:[start,goal]});
  assert.equal(detour.reason,null);assert.equal(detour.segmentClear,false);
  assert.deepEqual(checkGlobalSegment({map,localMap:blocked,segment:[start,goal],radius:.241}),
    {segmentClear:false,reason:'segment_blocked'});
  assert.ok(detour.path.length>2);
  const obstacleCells=new Uint8Array(80*80).fill(1);obstacleCells[40*80+40]=2;
  assertContinuousClear(detour.path,snapshot({cells:obstacleCells}),.241);
  for(let y=0;y<64;y++)observations[y*64+32]=2;
  blocked=local({cells:observations,origin:{x:.4,y:.4}});
  const closed=planGlobalRoute({map,localMap:blocked,start,goal,radius:.241,currentSegment:[start,goal]});
  assert.equal(closed.reason,'no_path');assert.deepEqual(closed.path,[]);assert.equal(closed.segmentClear,false);
  const restored=planGlobalRoute({map,localMap:free,start,goal,radius:.241,currentSegment:[start,goal]});
  assert.equal(restored.reason,null);assert.equal(restored.segmentClear,true);
  assert.deepEqual(checkGlobalSegment({map,localMap:free,segment:[start,goal],radius:.241}),
    {segmentClear:true,reason:null});
  assert.deepEqual(restored.path,[start,goal]);
});

test('起点距已知障碍边缘大于191mm时可进行车宽绕行',()=>{
  const map=decodeGlobalMap(snapshot({width:120,height:120,origin:{x:-3,y:-3}}));
  const cells=new Uint8Array(4096).fill(1);
  cells[35*64+35]=2;
  const grid=local({cells,origin:{x:-1.6,y:-1.6}});
  const start={x:.005,y:.005},goal={x:.755,y:.005};
  const clearance=Math.hypot(.15-start.x,.15-start.y);
  assert.ok(clearance>.191 && clearance<.241);
  const narrow=planGlobalRoute({map,localMap:grid,start,goal,radius:.191,
    currentSegment:[start,goal]});
  assert.equal(narrow.reason,null);
  assert.equal(narrow.segmentClear,false);
  assert.ok(narrow.path.length>2,'当前段贴障，应生成绕行折点');
  assert.equal(planGlobalRoute({map,localMap:grid,start,goal,radius:.241}).reason,'start_blocked',
    '旧的额外余量会拒绝实际可通的起点');
  for(let i=1;i<narrow.path.length;i++)
    assert.equal(checkGlobalSegment({map,localMap:grid,
      segment:[narrow.path[i-1],narrow.path[i]],radius:.191}).segmentClear,true);
});

function corridorScenario({narrow=false,rotated=false,remote=false,globalBlock=false}={}) {
  const transform=rotated ? {x:.71,y:-.43,yaw:.31} : {x:0,y:0,yaw:0};
  const gridTransform=rotated ? {map_anchor_x_mm:187,map_anchor_y_mm:-20,
    odom_anchor_x_mm:194,odom_anchor_y_mm:3,yaw_offset_mrad:-67} : IDENTITY;
  const toMap=(x,y)=>{
    const a=gridTransform.yaw_offset_mrad/1000,dx=x-gridTransform.map_anchor_x_mm/1000;
    const dy=y-gridTransform.map_anchor_y_mm/1000;
    const ox=gridTransform.odom_anchor_x_mm/1000+Math.cos(a)*dx+Math.sin(a)*dy;
    const oy=gridTransform.odom_anchor_y_mm/1000-Math.sin(a)*dx+Math.cos(a)*dy;
    return {x:transform.x+Math.cos(transform.yaw)*ox-Math.sin(transform.yaw)*oy,
      y:transform.y+Math.sin(transform.yaw)*ox+Math.cos(transform.yaw)*oy};
  };
  const localCells=new Uint8Array(4096);
  for(let x=32;x<64;x++) {
    localCells[27*64+x]=2;
    localCells[(narrow ? 35 : 36)*64+x]=2;
  }
  const grid=local({cells:localCells,origin:{x:-1.6,y:-1.6},transform:gridTransform});
  const globalCells=new Uint8Array(120*120);
  if(remote)for(let x=60;x<120;x++) {
    globalCells[55*120+x]=2;
    globalCells[64*120+x]=2;
  }
  if(globalBlock)globalCells[60*120+(remote ? 98 : 72)]=2;
  const map=decodeGlobalMap(snapshot({width:120,height:120,origin:{x:-3,y:-3},
    cells:globalCells,transform}));
  return {map,localMap:grid,start:toMap(-.5,.1),goal:toMap(remote ? 2.5 : 1,narrow ? -.025 : 0),
    radius:.191};
}

test('40cm通道的偏心起点可先居中，再按191mm余量连续通过',()=>{
  const args=corridorScenario(),route=planGlobalRoute(args);
  assert.equal(route.reason,null);
  assert.deepEqual(route.path[0],args.start);
  assert.deepEqual(route.path.at(-1),args.goal);
  assert.ok(route.path.length>=3,'直接斜入碰壁，需要先朝中线修正');
  let pieces=0;
  for(let i=1;i<route.path.length;i++) {
    const a=route.path[i-1],b=route.path[i],count=Math.ceil(Math.hypot(b.x-a.x,b.y-a.y)/.75);
    for(let n=0;n<count;n++) {
      const point=t=>({x:a.x+(b.x-a.x)*t,y:a.y+(b.y-a.y)*t});
      assert.equal(checkGlobalSegment({...args,segment:[point(n/count),point((n+1)/count)]}).segmentClear,true);
      pieces++;
    }
  }
  assert.ok(pieces>=3,'全路线可拆成连续短段');
});

test('局部细采样保持旋转平移和191mm真实碰撞判断',()=>{
  const args=corridorScenario({rotated:true}),route=planGlobalRoute(args);
  assert.equal(route.reason,null);
  assert.deepEqual(route.path.at(-1),args.goal);
  for(let i=1;i<route.path.length;i++)
    assert.equal(checkGlobalSegment({...args,segment:[route.path[i-1],route.path[i]]}).segmentClear,true);
  const narrow=corridorScenario({narrow:true});
  assert.equal(planGlobalRoute(narrow).reason,'goal_blocked','350mm净宽小于382mm扫掠直径');
});

test('目标在局部图外时只返回可精确直连的完整全局路线',()=>{
  const args=corridorScenario({remote:true});
  const route=planGlobalRoute(args);
  assert.equal(route.reason,null);
  assert.deepEqual(route.path.at(-1),args.goal);
  for(let i=1;i<route.path.length;i++)
    assert.equal(checkGlobalSegment({...args,segment:[route.path[i-1],route.path[i]]}).segmentClear,true);
  const blocked=corridorScenario({remote:true,globalBlock:true});
  assert.equal(planGlobalRoute(blocked).reason,'no_path','局部未知不能忽略远处全局已知占据');
});

test('接近容量上限的大地图仍能绕过局部障碍并穿过远处窄门',()=>{
  const args=corridorScenario(),width=2048,height=976,cells=new Uint8Array(width*height);
  for(let y=0;y<height;y++)if(y<484 || y>=492)cells[y*width+1600]=2;
  args.map=decodeGlobalMap(snapshot({width,height,origin:{x:-10,y:-24.4},cells}));
  args.goal={x:80,y:0};
  const route=planGlobalRoute(args);
  assert.equal(route.reason,null);
  assert.deepEqual(route.path[0],args.start);
  assert.deepEqual(route.path.at(-1),args.goal);
  assert.ok(route.path.length>=3);
  for(let i=1;i<route.path.length;i++)
    assert.equal(checkGlobalSegment({...args,segment:[route.path[i-1],route.path[i]]}).segmentClear,true);
});

test('局部细搜不能直连远目标时继续全局搜索，保留多折点绕行',()=>{
  const width=120,height=120,cells=new Uint8Array(width*height);
  for(let y=40;y<80;y++)cells[y*width+100]=2;
  const map=decodeGlobalMap(snapshot({width,height,origin:{x:-3,y:-3},cells}));
  const localCells=new Uint8Array(4096);
  localCells[32*64+40]=2;
  const grid=local({cells:localCells,origin:{x:-1.6,y:-1.6}});
  const args={map,localMap:grid,start:{x:-.5,y:0},goal:{x:2.7,y:0},radius:.191};
  const route=planGlobalRoute(args);
  assert.equal(route.reason,null);
  assert.deepEqual(route.path.at(-1),args.goal);
  assert.ok(route.path.length>=3);
  for(let i=1;i<route.path.length;i++)
    assert.equal(checkGlobalSegment({...args,segment:[route.path[i-1],route.path[i]]}).segmentClear,true);
});

test('取消后现场局部栅格按原方格保留198.85mm的真实起点余量',()=>{
  const saved=JSON.parse(readFileSync(new URL('./fixtures/local_blocked_20261102.json',import.meta.url),'utf8'));
  const grid=saved.local,start=grid.pose_odom,goal=saved.previous_task_goal_odom;
  const map=decodeGlobalMap(snapshot({width:160,height:160,origin:{x:-5,y:-5},
    cells:new Uint8Array(160*160)}));
  const t=grid.grid_from_odom,a=t.yaw_offset_mrad/1000;
  const gx=t.map_anchor_x_mm/1000+Math.cos(a)*(start.x-t.odom_anchor_x_mm/1000)-
    Math.sin(a)*(start.y-t.odom_anchor_y_mm/1000);
  const gy=t.map_anchor_y_mm/1000+Math.sin(a)*(start.x-t.odom_anchor_x_mm/1000)+
    Math.cos(a)*(start.y-t.odom_anchor_y_mm/1000);
  const left=grid.origin.x+32*grid.resolution_m,bottom=grid.origin.y+36*grid.resolution_m;
  const dx=Math.max(left-gx,0,gx-left-grid.resolution_m);
  const dy=Math.max(bottom-gy,0,gy-bottom-grid.resolution_m);
  assert.ok(Math.abs(Math.hypot(dx,dy)-.198850956)<1e-8);
  const plan=planGlobalRoute({map,localMap:grid,start,goal,radius:.191,currentSegment:[start,goal]});
  assert.equal(plan.reason,null);
  assert.equal(plan.segmentClear,true);
  assert.equal(checkGlobalSegment({map,localMap:grid,segment:[start,goal],radius:.191}).segmentClear,true);
  assert.equal(planGlobalRoute({map,localMap:grid,start,goal,radius:.241}).reason,'start_blocked');
});

test('旋转平移后的局部方格仍执行191mm精确扫掠和多格拐角检查',()=>{
  const transform={x:.37,y:-.41,yaw:.31};
  const map=decodeGlobalMap(snapshot({width:120,height:120,origin:{x:-3,y:-3},transform}));
  const gridTransform={map_anchor_x_mm:187,map_anchor_y_mm:-20,odom_anchor_x_mm:194,
    odom_anchor_y_mm:3,yaw_offset_mrad:-67};
  const cells=new Uint8Array(4096).fill(1);
  cells[35*64+35]=2;cells[36*64+36]=2;cells[35*64+36]=2;
  const grid=local({cells,origin:{x:-1.6,y:-1.6},transform:gridTransform});
  const toMap=(x,y)=>{
    const a=gridTransform.yaw_offset_mrad/1000,dx=x-gridTransform.map_anchor_x_mm/1000;
    const dy=y-gridTransform.map_anchor_y_mm/1000;
    const ox=gridTransform.odom_anchor_x_mm/1000+Math.cos(a)*dx+Math.sin(a)*dy;
    const oy=gridTransform.odom_anchor_y_mm/1000-Math.sin(a)*dx+Math.cos(a)*dy;
    return {x:transform.x+Math.cos(transform.yaw)*ox-Math.sin(transform.yaw)*oy,
      y:transform.y+Math.sin(transform.yaw)*ox+Math.cos(transform.yaw)*oy};
  };
  const start=toMap(.005,.005),goal=toMap(.755,.005);
  const result=planGlobalRoute({map,localMap:grid,start,goal,radius:.191,currentSegment:[start,goal]});
  assert.equal(result.reason,null);
  assert.equal(result.segmentClear,false);
  assert.ok(result.path.length>2);
  for(let i=1;i<result.path.length;i++)
    assert.equal(checkGlobalSegment({map,localMap:grid,
      segment:[result.path[i-1],result.path[i]],radius:.191}).segmentClear,true);
  assert.equal(planGlobalRoute({map,localMap:grid,start:toMap(.015,.015),goal,radius:.191}).reason,
    'start_blocked','实际边缘小于191mm时不能通过');

  const a=toMap(.05,.21),b=toMap(.21,.05);
  assert.equal(checkGlobalSegment({map,localMap:grid,segment:[a,a],radius:.05}).segmentClear,true);
  assert.equal(checkGlobalSegment({map,localMap:grid,segment:[b,b],radius:.05}).segmentClear,true);
  assert.equal(checkGlobalSegment({map,localMap:grid,segment:[a,b],radius:.05}).segmentClear,false,
    '端点虽可通，连接线扫掠仍会触及障碍角');
});

test('a 60 cm corridor remains reachable with the real radius; narrower than the diameter does not',()=>{
  const width=80,height=40,cells=new Uint8Array(width*height).fill(2);
  for(let y=0;y<height;y++)for(let x=34;x<=45;x++)cells[y*width+x]=1;
  const map=decodeGlobalMap(snapshot({width,height,cells}));
  const start=centre(39,10),goal=centre(39,29);
  const result=planGlobalRoute({map,start,goal,radius:.241,currentSegment:[start,goal]});
  assert.equal(result.reason,null);assert.equal(result.segmentClear,true);
  assert.deepEqual(result.path,[start,goal]);
  assertContinuousClear(result.path,snapshot({width,height,cells}),.241);
  for(let y=0;y<height;y++)for(let x=34;x<=45;x++)cells[y*width+x]=(x>=35 && x<=44) ? 1 : 2;
  const centred=decodeGlobalMap(snapshot({width,height,cells}));
  const centredStart={x:2,y:.525},centredGoal={x:2,y:1.475};
  const halfMetre=planGlobalRoute({map:centred,start:centredStart,goal:centredGoal,radius:.241});
  assert.equal(halfMetre.reason,null,'exact centreline fits even when neither neighbouring grid centre fits');
  assert.deepEqual(halfMetre.path,[centredStart,centredGoal]);
  for(let y=0;y<height;y++)for(let x=34;x<=45;x++)cells[y*width+x]=(x>=35 && x<=43) ? 1 : 2;
  const narrow=decodeGlobalMap(snapshot({width,height,cells}));
  const cannotFit=planGlobalRoute({map:narrow,start,goal,radius:.241,currentSegment:[start,goal]});
  assert.equal(cannotFit.reason,'goal_blocked');assert.equal(cannotFit.segmentClear,false);
});

test('zero-radius reachability agrees with an independent no-corner-cutting breadth-first search',()=>{
  let seed=0x51a72;
  const random=()=>{seed^=seed<<13;seed^=seed>>>17;seed^=seed<<5;return seed>>>0;};
  const width=8,height=8;
  const moves=[[-1,0],[1,0],[0,-1],[0,1],[-1,-1],[-1,1],[1,-1],[1,1]];
  for(let sample=0;sample<200;sample++) {
    const cells=Uint8Array.from({length:width*height},()=>random()%5===0 ? 2 : 1);
    cells[0]=1;cells[cells.length-1]=1;
    const visited=new Uint8Array(cells.length),queue=[0];visited[0]=1;
    for(let head=0;head<queue.length;head++) {
      const id=queue[head],x=id%width,y=Math.floor(id/width);
      for(const [dx,dy] of moves) {
        const nx=x+dx,ny=y+dy;
        if(nx<0 || nx>=width || ny<0 || ny>=height)continue;
        const next=ny*width+nx;
        if(cells[next]!==1 || visited[next] ||
          (dx && dy && (cells[y*width+nx]!==1 || cells[ny*width+x]!==1)))continue;
        visited[next]=1;queue.push(next);
      }
    }
    const map=decodeGlobalMap(snapshot({width,height,resolution:1,cells}));
    const result=planGlobalRoute({map,start:{x:.5,y:.5},goal:{x:7.5,y:7.5},radius:0});
    assert.equal(result.reason===null,visited[63]===1,`sample ${sample}`);
  }
});

test('lightweight segment check agrees with route planning and fails closed on invalid context',()=>{
  let seed=0x3af912;
  const random=()=>{seed^=seed<<13;seed^=seed>>>17;seed^=seed<<5;return seed>>>0;};
  for(let sample=0;sample<80;sample++) {
    const cells=Uint8Array.from({length:30*30},()=>random()%11===0 ? 2 : 1);
    const map=decodeGlobalMap(snapshot({width:30,height:30,resolution:.05,cells}));
    const start=centre(6+random()%18,6+random()%18),goal=centre(6+random()%18,6+random()%18);
    const full=planGlobalRoute({map,start,goal,radius:.1,currentSegment:[start,goal]});
    const quick=checkGlobalSegment({map,segment:[start,goal],radius:.1});
    assert.equal(quick.segmentClear,full.segmentClear,`sample ${sample}`);
  }
  const map=decodeGlobalMap(snapshot());
  assert.deepEqual(checkGlobalSegment({map,localMap:{},segment:[{x:1,y:1},{x:2,y:2}]}),
    {segmentClear:false,reason:'invalid_local_map'});
  assert.deepEqual(checkGlobalSegment({map,segment:[{x:1,y:1},{x:20,y:2}]}),
    {segmentClear:true,reason:null});
  assert.equal(checkGlobalSegment({map,segment:[{x:1,y:1},{x:2,y:2}],radius:-1}).reason,'invalid_radius');
  assert.equal(planGlobalRoute({map,start:{x:1,y:1},goal:{x:2,y:2},radius:-1}).reason,'invalid_radius');
});

test('line clearance uses actual swept radius instead of rejecting the whole neighbouring cell',()=>{
  const cells=new Uint8Array(80*80).fill(1);cells[40*80+40]=2;
  const map=decodeGlobalMap(snapshot({cells}));
  const start={x:1,y:2.292},goal={x:3,y:2.292};
  const clear=planGlobalRoute({map,start,goal,radius:.241,currentSegment:[start,goal]});
  assert.equal(clear.reason,null);assert.equal(clear.segmentClear,true);
  const nearStart={...start,y:2.290},nearGoal={...goal,y:2.290};
  const blocked=planGlobalRoute({map,start:nearStart,goal:nearGoal,radius:.241,currentSegment:[nearStart,nearGoal]});
  assert.equal(blocked.reason,null);assert.equal(blocked.segmentClear,false);
  assert.ok(blocked.path.length>2);
  assertContinuousClear(blocked.path,snapshot({cells}),.241);
});

test('diagonal endpoints can be clear while the swept circle hits an obstacle between them',()=>{
  const width=30,height=30,cells=new Uint8Array(width*height).fill(1);
  cells[12*width+9]=2;
  const source=snapshot({width,height,cells}),map=decodeGlobalMap(source);
  const start=centre(10,10),goal=centre(11,11);
  const result=planGlobalRoute({map,start,goal,radius:.075,currentSegment:[start,goal]});
  assert.equal(result.segmentClear,false);
  assert.equal(result.reason,null);
  assert.ok(result.path.length>2);
  assertContinuousClear(result.path,source,.075);
});

test('planned paths retain circular clearance on varied obstacle fields',()=>{
  let seed=0x7ac52,found=0;
  const random=()=>{seed^=seed<<13;seed^=seed>>>17;seed^=seed<<5;return seed>>>0;};
  for(let sample=0;sample<80;sample++) {
    const cells=Uint8Array.from({length:40*40},()=>random()%55===0 ? 2 : 1);
    cells[6*40+6]=1;
    const source=snapshot({width:40,height:40,cells}),map=decodeGlobalMap(source);
    const result=planGlobalRoute({map,start:centre(6,6),goal:centre(33,33),radius:.075});
    if(result.reason===null) {found++;assertContinuousClear(result.path,source,.075);}
  }
  assert.ok(found>=5,'exercise successful paths across varied obstacle fields');
});

test('局部空闲覆盖历史障碍，局部未知保留障碍，全局未知可以探索',()=>{
  const cells=new Uint8Array(80*80).fill(1);
  cells[40*80+40]=2;
  const map=decodeGlobalMap(snapshot({cells}));
  const start={x:1,y:2.025},goal={x:3,y:2.025},segment=[start,goal];
  const unknown=local({cells:new Uint8Array(4096),origin:{x:.4,y:.4}});
  const keep=planGlobalRoute({map,localMap:unknown,start,goal,currentSegment:segment});
  assert.equal(keep.segmentClear,false);
  const free=local({cells:new Uint8Array(4096).fill(1),origin:{x:.4,y:.4}});
  const cleared=planGlobalRoute({map,localMap:free,start,goal,currentSegment:segment});
  assert.equal(cleared.reason,null);assert.equal(cleared.segmentClear,true);
  assert.equal(map.cells[40*80+40],2,'overlay must not mutate cached global cells');
  cells[40*80+40]=0;
  const unknownMap=decodeGlobalMap(snapshot({cells}));
  const exploreUnknown=planGlobalRoute({map:unknownMap,localMap:free,start,goal,
    currentSegment:segment});
  assert.equal(exploreUnknown.segmentClear,true);
  const unknownGoal=planGlobalRoute({map:unknownMap,localMap:free,start,goal:centre(40,40)});
  assert.equal(unknownGoal.reason,null);
  const notObserved=planGlobalRoute({map:unknownMap,localMap:unknown,start,goal:centre(40,40)});
  assert.equal(notObserved.reason,null,'局部未观测并不把未知当成占据');
});

test('map and grid rotations, translations and anchors project dynamic obstacles correctly',()=>{
  const transform={map_anchor_x_mm:300,map_anchor_y_mm:-400,odom_anchor_x_mm:400,
    odom_anchor_y_mm:-100,yaw_offset_mrad:1200};
  const mapTf={x:1.1,y:-.4,yaw:-.8},gridOrigin={x:-1.6,y:-1.6};
  const obstacle=centre(32,32,.05,gridOrigin),a=1.2,b=mapTf.yaw;
  const dx=obstacle.x-.3,dy=obstacle.y+.4;
  const odom={x:.4+Math.cos(a)*dx+Math.sin(a)*dy,
    y:-.1-Math.sin(a)*dx+Math.cos(a)*dy};
  const world={x:mapTf.x+Math.cos(b)*odom.x-Math.sin(b)*odom.y,
    y:mapTf.y+Math.sin(b)*odom.x+Math.cos(b)*odom.y};
  const cells=new Uint8Array(4096).fill(1);cells[32*64+32]=2;
  const map=decodeGlobalMap(snapshot({width:200,height:200,origin:{x:-5,y:-5},transform:mapTf}));
  const grid=local({cells,origin:gridOrigin,transform});
  const start={x:world.x-1,y:world.y},goal={x:world.x+1,y:world.y};
  const blocked=planGlobalRoute({map,localMap:grid,start,goal,currentSegment:[start,goal]});
  assert.equal(blocked.reason,null);assert.equal(blocked.segmentClear,false);
  assert.ok(blocked.path.length>2,'rotated current observation must be avoided');
  const moved=local({cells:new Uint8Array(4096).fill(1),origin:gridOrigin,transform});
  const clear=planGlobalRoute({map,localMap:moved,start,goal,currentSegment:[start,goal]});
  assert.equal(clear.reason,null);assert.equal(clear.segmentClear,true);
});

test('禁止穿角、允许范围外目标并拒绝已知障碍和无效地图',()=>{
  const cells=Uint8Array.from([1,2,1, 2,1,1, 1,1,1]);
  const map=decodeGlobalMap(snapshot({width:3,height:3,resolution:1,cells}));
  const start={x:.5,y:.5},goal={x:1.5,y:1.5};
  const closed=planGlobalRoute({map,start,goal,radius:0,currentSegment:[start,goal]});
  assert.equal(closed.reason,'no_path');assert.equal(closed.segmentClear,false);
  assert.equal(planGlobalRoute({map,start,goal:{x:3,y:1},radius:0}).reason,null);
  assert.equal(planGlobalRoute({map,start,goal:{x:1.5,y:.5},radius:0}).reason,'goal_blocked');
  assert.equal(planGlobalRoute({map,start,goal,localMap:{},radius:0}).reason,'invalid_local_map');
  const boundary=decodeGlobalMap(snapshot({width:4,height:4,resolution:1}));
  const a={x:1.5,y:1.5},b={x:1,y:2};
  const exact=planGlobalRoute({map:boundary,start:a,goal:b,radius:0,currentSegment:[a,b]});
  assert.equal(exact.reason,null);assert.equal(exact.segmentClear,true);
  const cornerCells=new Uint8Array(16).fill(1);cornerCells[2*4]=2;
  const touched=decodeGlobalMap(snapshot({width:4,height:4,resolution:1,cells:cornerCells}));
  const corner=planGlobalRoute({map:touched,start:a,goal:b,radius:0,currentSegment:[a,b]});
  assert.equal(corner.reason,'goal_blocked');assert.equal(corner.segmentClear,false);
});

test('只放开实际车位所在格，后续实测障碍仍须绕行',()=>{
  const cells=new Uint8Array(40*40).fill(1);cells[10*40+10]=2;
  const map=decodeGlobalMap(snapshot({width:40,height:40,resolution:.1,cells}));
  const start={x:1.025,y:1.025},goal={x:2.025,y:1.025};
  assert.equal(planGlobalRoute({map,start,goal}).reason,null);
  const seen=new Uint8Array(4096);seen[20*64+20]=1;
  const correction=planGlobalRoute({map,localMap:local({cells:seen}),start,goal});
  assert.equal(correction.reason,null);
  seen[20*64+23]=2;
  const currentObstacle=planGlobalRoute({map,localMap:local({cells:seen}),start,goal,
    currentSegment:[start,goal]});
  assert.equal(currentObstacle.reason,null);
  assert.equal(currentObstacle.segmentClear,false);
  assert.ok(currentObstacle.path.length>2);
});
