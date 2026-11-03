// 使用 SLAM 快照和最新局部栅格进行纯计算；对外坐标均为米和弧度。
import gridCodec from './public/js/grid_codec.js';
const {decodeCells} = gridCodec;
const MAX_SIDE = 2048;
const MAX_CELLS = 2_000_000;
const MAX_ROUTE_POINTS = 4096;
const MAX_TURNS = 20000;
const INF = 1_000_000_000;

const finitePoint = point => Number.isFinite(point?.x) && Number.isFinite(point?.y);
const validTransform = transform => finitePoint(transform) && Number.isFinite(transform?.yaw);
const gridIndex = value => Math.abs(value-Math.round(value))<1e-10 ? Math.round(value) : Math.floor(value);
const validDimensions = map =>
  Number.isInteger(map?.width) && map.width > 0 && map.width <= MAX_SIDE &&
  Number.isInteger(map?.height) && map.height > 0 && map.height <= MAX_SIDE &&
  map.width * map.height <= MAX_CELLS &&
  Number.isFinite(map.resolution_m) && map.resolution_m > 0 &&
  finitePoint(map.origin);

// 每份全局快照只解码一次，后续可复用栅格并叠加新的局部地图。
// 数据新鲜度由协调器在选取快照时检查。
export function decodeGlobalMap(snapshot) {
  if (snapshot?.v !== 1 || snapshot.type !== 'global_mapping' ||
      snapshot.protocol !== 'slam_toolbox_view_v1' || snapshot.source !== 'slam_toolbox' ||
      snapshot.available !== true || !validDimensions(snapshot) ||
      typeof snapshot.domain !== 'string' || !snapshot.domain ||
      typeof snapshot.generation !== 'string' || !snapshot.generation ||
      !validTransform(snapshot.map_from_odom) || !validTransform(snapshot.pose))
    throw new Error('invalid global map');
  const {cells_rle, ...metadata} = snapshot;
  return {...metadata, origin:{...snapshot.origin}, pose:{...snapshot.pose},
    map_from_odom:{...snapshot.map_from_odom},
    cells:decodeCells(cells_rle, snapshot.width * snapshot.height)};
}

function decodeLocalMap(local) {
  // 局部快照由 FpgaGridMap 和协调器校验，只需解开传入工作线程的栅格。
  return {...local, cells:decodeCells(local.cells_rle, local.width * local.height)};
}

function transforms(map, local) {
  const t = local.grid_from_odom;
  const a = t.yaw_offset_mrad / 1000, b = map.map_from_odom.yaw;
  const ca = Math.cos(a), sa = Math.sin(a), cb = Math.cos(b), sb = Math.sin(b);
  const c = Math.cos(a - b), s = Math.sin(a - b);
  const mapX = map.map_from_odom.x, mapY = map.map_from_odom.y;
  const anchorX = t.map_anchor_x_mm / 1000, anchorY = t.map_anchor_y_mm / 1000;
  const odomX = t.odom_anchor_x_mm / 1000, odomY = t.odom_anchor_y_mm / 1000;
  const tx = anchorX - (c*mapX - s*mapY) - (ca*odomX - sa*odomY);
  const ty = anchorY - (s*mapX + c*mapY) - (sa*odomX + ca*odomY);
  return {
    c,s,tx,ty,
    toGrid: (x,y) => ({x:c*x-s*y+tx, y:s*x+c*y+ty}),
    toMap: (x,y) => {
      const dx=x-anchorX, dy=y-anchorY;
      const ox=odomX+ca*dx+sa*dy, oy=odomY-sa*dx+ca*dy;
      return {x:mapX+cb*ox-sb*oy, y:mapY+sb*ox+cb*oy};
    },
  };
}

// 局部占据格与全局格有交叠就阻挡；全部被局部空闲覆盖的全局格更新为空闲。
// 未知格保留原值，由路线搜索作为可探索区域处理。
function overlayLocal(map, local, region=null, projectOccupied=true) {
  const merged = map.cells.slice();
  if (!local) return merged;
  const {toMap,c,s,tx,ty} = transforms(map,local);
  const gx0=local.origin.x, gy0=local.origin.y, lr=local.resolution_m;
  const gx1=gx0+local.width*lr, gy1=gy0+local.height*lr;
  const corners=[toMap(gx0,gy0),toMap(gx1,gy0),toMap(gx0,gy1),toMap(gx1,gy1)];
  const minX=Math.min(...corners.map(p=>p.x)), maxX=Math.max(...corners.map(p=>p.x));
  const minY=Math.min(...corners.map(p=>p.y)), maxY=Math.max(...corners.map(p=>p.y));
  const r=map.resolution_m, ox=map.origin.x, oy=map.origin.y;
  const x0=Math.max(0,Math.floor((minX-ox)/r),region ? Math.floor((region.minX-ox)/r)-2 : 0);
  const x1=Math.min(map.width-1,Math.floor((maxX-ox)/r),region ? Math.floor((region.maxX-ox)/r)+2 : map.width-1);
  const y0=Math.max(0,Math.floor((minY-oy)/r),region ? Math.floor((region.minY-oy)/r)-2 : 0);
  const y1=Math.min(map.height-1,Math.floor((maxY-oy)/r),region ? Math.floor((region.maxY-oy)/r)+2 : map.height-1);
  const eps=1e-8;
  const xx=c*r,xy=s*r,yx=-s*r,yy=c*r;
  const minDx=Math.min(0,xx,yx,xx+yx),maxDx=Math.max(0,xx,yx,xx+yx);
  const minDy=Math.min(0,xy,yy,xy+yy),maxDy=Math.max(0,xy,yy,xy+yy);
  for (let y=y0; y<=y1; y++) for (let x=x0; x<=x1; x++) {
    const mx=ox+x*r, my=oy+y*r;
    const baseX=c*mx-s*my+tx,baseY=s*mx+c*my+ty;
    const lx0=baseX+minDx,lx1=baseX+maxDx,ly0=baseY+minDy,ly1=baseY+maxDy;
    let ax=Math.floor((lx0-gx0)/lr+eps), bx=Math.ceil((lx1-gx0)/lr-eps)-1;
    let ay=Math.floor((ly0-gy0)/lr+eps), by=Math.ceil((ly1-gy0)/lr-eps)-1;
    if (bx<0 || by<0 || ax>=local.width || ay>=local.height || bx<ax || by<ay) continue;
    let allFree=ax>=0 && ay>=0 && bx<local.width && by<local.height;
    ax=Math.max(0,ax); ay=Math.max(0,ay);
    bx=Math.min(local.width-1,bx); by=Math.min(local.height-1,by);
    let occupied=false;
    for (let ly=ay; ly<=by && !occupied; ly++) for (let lx=ax; lx<=bx; lx++) {
      const value=local.cells[ly*local.width+lx];
      if (value===2) { occupied=true; break; }
      if (value!==1) allFree=false;
    }
    const i=y*map.width+x;
    if (occupied) { if(projectOccupied) merged[i]=2; }
    else if (allFree) merged[i]=1;
  }
  return merged;
}

// 一维精确欧氏距离平方变换。
function distanceLine(f,d,n,v,z) {
  let k=0;
  v[0]=0; z[0]=-Infinity; z[1]=Infinity;
  for(let q=1;q<n;q++) {
    let p=v[k], cross=((f[q]+q*q)-(f[p]+p*p))/(2*(q-p));
    while(cross<=z[k]) {
      k--;
      p=v[k];
      cross=((f[q]+q*q)-(f[p]+p*p))/(2*(q-p));
    }
    k++;v[k]=q;z[k]=cross;z[k+1]=Infinity;
  }
  k=0;
  for(let q=0;q<n;q++) {
    while(z[k+1]<q)k++;
    const delta=q-v[k];
    d[q]=delta*delta+f[v[k]];
  }
}

// 快速筛选可容纳圆形车体的格心；线段仍需做实际扫掠检查。
// 距离足够远的整格可跳过逐障碍检查。未知区域可探索，地图外不可搜索。
function inflate(map,cells,radius) {
  const {width:w,height:h,resolution_m:r}=map, size=w*h;
  const safe=new Uint8Array(size), guaranteed=new Uint8Array(size);
  if(radius===0) {
    for(let i=0;i<size;i++) safe[i]=cells[i]!==2 ? 1 : 0;
    return {safe,guaranteed};
  }
  const radiusCells=radius/r;
  if(radiusCells*2>=w || radiusCells*2>=h) return {safe,guaranteed};
  const temporary=new Int32Array(size), n=Math.max(w,h);
  const f=new Int32Array(n),d=new Int32Array(n),v=new Int32Array(n),z=new Float64Array(n+1);
  for(let y=0;y<h;y++) {
    const row=y*w;
    for(let x=0;x<w;x++)f[x]=cells[row+x]!==2 ? INF : 0;
    distanceLine(f,d,w,v,z);
    temporary.set(d.subarray(0,w),row);
  }
  const centreClearance=radiusCells+Math.SQRT2/2;
  const wholeCellClearance=radiusCells+Math.SQRT2;
  const centreThreshold=centreClearance*centreClearance;
  const wholeCellThreshold=wholeCellClearance*wholeCellClearance;
  for(let x=0;x<w;x++) {
    for(let y=0;y<h;y++)f[y]=temporary[y*w+x];
    distanceLine(f,d,h,v,z);
    for(let y=0;y<h;y++) {
      const i=y*w+x;
      if(cells[i]===2)continue;
      if(x+.5>=radiusCells && w-x-.5>=radiusCells &&
         y+.5>=radiusCells && h-y-.5>=radiusCells && d[y]>centreThreshold)safe[i]=1;
      if(x>=radiusCells && w-x-1>=radiusCells &&
         y>=radiusCells && h-y-1>=radiusCells && d[y]>wholeCellThreshold)guaranteed[i]=1;
    }
  }
  return {safe,guaranteed};
}

function cellFor(map,point) {
  if(!finitePoint(point))return null;
  const x=gridIndex((point.x-map.origin.x)/map.resolution_m);
  const y=gridIndex((point.y-map.origin.y)/map.resolution_m);
  return x>=0 && y>=0 && x<map.width && y<map.height ? {x,y,index:y*map.width+x} : null;
}

// 建图范围会随观测扩大。对范围外的合法目标临时补未知格，不更改真实 SLAM 快照。
function expandForPoints(map,points,radius) {
  const right=map.origin.x+map.width*map.resolution_m;
  const top=map.origin.y+map.height*map.resolution_m;
  if(points.every(point=>cellFor(map,point) && point.x-map.origin.x>=radius &&
      point.y-map.origin.y>=radius && right-point.x>=radius && top-point.y>=radius))return map;
  const r=map.resolution_m, margin=Math.ceil(radius/r)+1;
  const x=points.map(point=>Math.floor((point.x-map.origin.x)/r));
  const y=points.map(point=>Math.floor((point.y-map.origin.y)/r));
  const x0=Math.min(0,...x.map(value=>value-margin));
  const y0=Math.min(0,...y.map(value=>value-margin));
  const x1=Math.max(map.width-1,...x.map(value=>value+margin));
  const y1=Math.max(map.height-1,...y.map(value=>value+margin));
  const width=x1-x0+1,height=y1-y0+1;
  if(!(width<=MAX_SIDE && height<=MAX_SIDE && width*height<=MAX_CELLS))return null;
  const cells=new Uint8Array(width*height);
  for(let row=0;row<map.height;row++)
    cells.set(map.cells.subarray(row*map.width,(row+1)*map.width),(row-y0)*width-x0);
  return {...map,width,height,origin:{x:map.origin.x+x0*r,y:map.origin.y+y0*r},cells};
}

// 返回线段与占据格矩形的最短距离是否不大于车体半径。
function touchesObstacle(ax,ay,bx,by,left,bottom,radiusSquared) {
  const right=left+1,top=bottom+1,dx=bx-ax,dy=by-ay;
  let lo=0,hi=1;
  const clip=(p,q,min,max)=>{
    if(q===0)return p>=min && p<=max;
    let t0=(min-p)/q,t1=(max-p)/q;
    if(t0>t1)[t0,t1]=[t1,t0];
    lo=Math.max(lo,t0);hi=Math.min(hi,t1);
    return lo<=hi;
  };
  if(clip(ax,dx,left,right) && clip(ay,dy,bottom,top))return true;
  const pointToBox=(x,y)=>{
    const px=Math.max(left-x,0,x-right),py=Math.max(bottom-y,0,y-top);
    return px*px+py*py;
  };
  let minimum=Math.min(pointToBox(ax,ay),pointToBox(bx,by));
  const lengthSquared=dx*dx+dy*dy;
  for(const [x,y] of [[left,bottom],[left,top],[right,bottom],[right,top]]) {
    const t=lengthSquared===0 ? 0 : Math.max(0,Math.min(1,((x-ax)*dx+(y-ay)*dy)/lengthSquared));
    const px=x-(ax+t*dx),py=y-(ay+t*dy);
    minimum=Math.min(minimum,px*px+py*py);
  }
  return minimum<=radiusSquared+1e-12;
}

// 车宽绕行直接使用局部原始方格，避免旋转投影后再膨胀造成重复放大。
function localSweptCheck(map,local,radius) {
  if(!local || !local.cells.includes(2))return ()=>true;
  const {toGrid}=transforms(map,local),r=local.resolution_m,range=radius/r;
  return (a,b)=>{
    const p=toGrid(a.x,a.y),q=toGrid(b.x,b.y);
    const ax=(p.x-local.origin.x)/r,ay=(p.y-local.origin.y)/r;
    const bx=(q.x-local.origin.x)/r,by=(q.y-local.origin.y)/r;
    const x0=Math.max(0,Math.floor(Math.min(ax,bx)-range)-1);
    const x1=Math.min(local.width-1,Math.floor(Math.max(ax,bx)+range));
    const y0=Math.max(0,Math.floor(Math.min(ay,by)-range)-1);
    const y1=Math.min(local.height-1,Math.floor(Math.max(ay,by)+range));
    for(let y=y0;y<=y1;y++)for(let x=x0;x<=x1;x++)
      if(local.cells[y*local.width+x]===2 &&
          touchesObstacle(ax,ay,bx,by,x,y,range*range))return false;
    return true;
  };
}

// 检查线段触及的所有栅格，并精确检查车体扫掠与邻近障碍的距离。
function segmentIsClear(map,cells,guaranteed,a,b,radius) {
  if(!cellFor(map,a) || !cellFor(map,b))return false;
  const r=map.resolution_m, ox=map.origin.x, oy=map.origin.y;
  const x0=(a.x-ox)/r, y0=(a.y-oy)/r, x1=(b.x-ox)/r, y1=(b.y-oy)/r;
  const radiusCells=radius/r, range=Math.ceil(radiusCells)+1;
  if(Math.min(x0,x1,y0,y1)<radiusCells ||
      Math.max(x0,x1)>map.width-radiusCells || Math.max(y0,y1)>map.height-radiusCells)
    return false;
  const dx=x1-x0,dy=y1-y0, sx=Math.sign(dx),sy=Math.sign(dy);
  let x=gridIndex(x0),y=gridIndex(y0);
  const tx=gridIndex(x1),ty=gridIndex(y1);
  const onV=dx===0 && Math.abs(x0-Math.round(x0))<1e-10;
  const onH=dy===0 && Math.abs(y0-Math.round(y0))<1e-10;
  const valid=(cx,cy)=>{
    if(cx<0 || cy<0 || cx>=map.width || cy>=map.height || cells[cy*map.width+cx]===2)
      return false;
    if(radius===0 || guaranteed?.[cy*map.width+cx])return true;
    const minX=Math.max(0,cx-range),maxX=Math.min(map.width-1,cx+range);
    const minY=Math.max(0,cy-range),maxY=Math.min(map.height-1,cy+range);
    for(let gy=minY;gy<=maxY;gy++)for(let gx=minX;gx<=maxX;gx++)
      if(cells[gy*map.width+gx]===2 &&
          touchesObstacle(x0,y0,x1,y1,gx,gy,radiusCells*radiusCells))return false;
    return true;
  };
  const check=(cx,cy)=>valid(cx,cy) && (!onV || valid(cx-1,cy)) &&
    (!onH || valid(cx,cy-1)) && (!(onV && onH) || valid(cx-1,cy-1));
  const endpointClear=(px,py)=>{
    const cx=gridIndex(px),cy=gridIndex(py);
    const onX=Math.abs(px-Math.round(px))<1e-10;
    const onY=Math.abs(py-Math.round(py))<1e-10;
    return valid(cx,cy) && (!onX || valid(cx-1,cy)) && (!onY || valid(cx,cy-1)) &&
      (!(onX && onY) || valid(cx-1,cy-1));
  };
  if(!endpointClear(x0,y0) || !endpointClear(x1,y1))return false;
  if(!check(x,y))return false;
  let nextX=sx>0 ? (x+1-x0)/dx : sx<0 ? (x0-x)/-dx : Infinity;
  let nextY=sy>0 ? (y+1-y0)/dy : sy<0 ? (y0-y)/-dy : Infinity;
  const stepX=sx===0 ? Infinity : 1/Math.abs(dx);
  const stepY=sy===0 ? Infinity : 1/Math.abs(dy);
  while(x!==tx || y!==ty) {
    // 负方向的终点恰在格线上时已属于目标格，不能在终点继续跨格。
    if(x===tx)nextX=Infinity;
    if(y===ty)nextY=Infinity;
    if(Math.abs(nextX-nextY)<1e-12) {
      if(!check(x+sx,y) || !check(x,y+sy))return false;
      x+=sx;y+=sy;nextX+=stepX;nextY+=stepY;
    } else if(nextX<nextY) {x+=sx;nextX+=stepX;}
    else {y+=sy;nextY+=stepY;}
    if(!check(x,y))return false;
  }
  return true;
}

function findPath(map,cells,safe,guaranteed,start,goal,radius,localClear,search=null) {
  const grid=search?.grid || map,{width:w,height:h}=grid,size=w*h;
  const toGrid=search?.toGrid || ((x,y)=>({x,y}));
  const toMap=search?.toMap || ((x,y)=>({x,y}));
  const from=cellFor(grid,toGrid(start.x,start.y)),to=cellFor(grid,toGrid(goal.x,goal.y));
  const outsideGoal=!!search && !to;
  if(!from || (!to && !outsideGoal))return {path:[],reason:'no_path'};
  const centre=id=>toMap(grid.origin.x+(id%w+.5)*grid.resolution_m,
    grid.origin.y+(Math.floor(id/w)+.5)*grid.resolution_m);
  const clear=(a,b)=>segmentIsClear(map,cells,guaranteed,a,b,radius) && localClear(a,b);
  const cached=search ? new Uint8Array(size) : null;
  const safeNode=id=>{
    if(!search)return !!safe[id];
    if(!cached[id]) {
      const point=centre(id);
      cached[id]=clear(point,point) ? 1 : 2;
    }
    return cached[id]===1;
  };
  // 精确位置可能处于安全格边缘，但该格的中心不可用；尝试连接邻近格心。
  function nearby(point,cell) {
    const result=[];
    for(let dy=-1;dy<=1;dy++)for(let dx=-1;dx<=1;dx++) {
      const x=cell.x+dx,y=cell.y+dy;
      if(x<0 || y<0 || x>=w || y>=h)continue;
      const id=y*w+x;
      if(safeNode(id) && clear(point,centre(id)))result.push(id);
    }
    return result;
  }
  const starts=nearby(start,from);
  const ends=outsideGoal ? null : new Set(nearby(goal,to));
  if(!starts.length || (ends && !ends.size))return {path:[],reason:'no_path'};
  const g=new Float64Array(size);g.fill(Infinity);
  const f=new Float64Array(size),parent=new Int32Array(size),pos=new Int32Array(size);
  const heap=new Int32Array(size+1);
  pos.fill(-1);parent.fill(-1);
  let count=0;
  const heuristic=id=>{
    if(outsideGoal) {
      const p=centre(id);
      return Math.hypot(p.x-goal.x,p.y-goal.y)/grid.resolution_m;
    }
    const x=id%w,y=Math.floor(id/w),dx=Math.abs(x-to.x),dy=Math.abs(y-to.y);
    return Math.max(dx,dy)+(Math.SQRT2-1)*Math.min(dx,dy);
  };
  const less=(a,b)=>f[a]<f[b] || (f[a]===f[b] && g[a]>g[b]);
  function pushOrDecrease(id,score) {
    f[id]=score;
    let p=pos[id];
    if(p<0){p=++count;heap[p]=id;pos[id]=p;}
    while(p>1) {
      const up=p>>1;
      if(!less(heap[p],heap[up]))break;
      const temp=heap[p];heap[p]=heap[up];heap[up]=temp;
      pos[heap[p]]=p;pos[heap[up]]=up;p=up;
    }
  }
  function pop() {
    const result=heap[1],last=heap[count--];
    pos[result]=-2;
    if(count) {
      heap[1]=last;pos[last]=1;
      let p=1;
      for(;;) {
        const left=p*2,right=left+1;
        let best=p;
        if(left<=count && less(heap[left],heap[best]))best=left;
        if(right<=count && less(heap[right],heap[best]))best=right;
        if(best===p)break;
        const temp=heap[p];heap[p]=heap[best];heap[best]=temp;
        pos[heap[p]]=p;pos[heap[best]]=best;p=best;
      }
    }
    return result;
  }
  for(const id of starts) {
    const p=centre(id);
    g[id]=Math.hypot(p.x-start.x,p.y-start.y)/grid.resolution_m;
    pushOrDecrease(id,g[id]+heuristic(id));
  }
  const moves=[[-1,0],[1,0],[0,-1],[0,1],[-1,-1],[-1,1],[1,-1],[1,1]];
  let reached=-1;
  while(count) {
    const id=pop();
    if(outsideGoal ? clear(centre(id),goal) : ends.has(id)){reached=id;break;}
    const x=id%w,y=Math.floor(id/w);
    for(const [dx,dy] of moves) {
      const nx=x+dx,ny=y+dy;
      if(nx<0 || ny<0 || nx>=w || ny>=h)continue;
      const next=ny*w+nx;
      if(!safeNode(next) || pos[next]===-2)continue;
      if(dx && dy && (!safeNode(y*w+nx) || !safeNode(ny*w+x)))continue;
      const a=centre(id),b=centre(next);
      if(search ? !clear(a,b) : ((!(guaranteed[id] && guaranteed[next]) &&
          !segmentIsClear(map,cells,guaranteed,a,b,radius)) || !localClear(a,b)))continue;
      const candidate=g[id]+(dx && dy ? Math.SQRT2 : 1);
      if(candidate>=g[next])continue;
      parent[next]=id;g[next]=candidate;
      pushOrDecrease(next,candidate+heuristic(next));
    }
  }
  if(reached<0)return {path:[],reason:'no_path'};
  const indices=[];
  for(let id=reached;id!==-1;id=parent[id]) {
    indices.push(id);
    if(indices.length>size)return {path:[],reason:'no_path'};
  }
  indices.reverse();
  const points=[{x:start.x,y:start.y}];
  const startCentre=centre(indices[0]);
  if(startCentre.x!==start.x || startCentre.y!==start.y)points.push(startCentre);
  for(let i=1;i<indices.length-1;i++) {
    const prev=indices[i-1],id=indices[i],next=indices[i+1];
    if(id%w-prev%w!==next%w-id%w ||
       Math.floor(id/w)-Math.floor(prev/w)!==Math.floor(next/w)-Math.floor(id/w)) {
      points.push(centre(id));
      if(points.length>MAX_TURNS)return {path:[],reason:'route_too_complex'};
    }
  }
  const goalCentre=centre(reached);
  if((goalCentre.x!==goal.x || goalCentre.y!==goal.y) && (outsideGoal || reached!==indices[0]))
    points.push(goalCentre);
  points.push({x:goal.x,y:goal.y});
  const simplified=[points[0]];
  let anchor=0;
  while(anchor<points.length-1) {
    let end=anchor+1;
    if(!clear(points[anchor],points[end]))
      return {path:[],reason:'no_path'};
    const limit=Math.min(points.length-1,anchor+32);
    while(end<limit && clear(points[anchor],points[end+1]))end++;
    simplified.push(points[end]);anchor=end;
    if(simplified.length>MAX_ROUTE_POINTS)return {path:[],reason:'route_too_complex'};
  }
  return {path:simplified,reason:null};
}

function mergedForStart(map,localMap,startCell,region=null,projectOccupied=true) {
  const local=localMap===null ? null : decodeLocalMap(localMap);
  const merged=overlayLocal(map,local,region,projectOccupied);
  // 实际车位作为路线起点；只放开起点所在的一个格，不更改原始地图。
  merged[startCell.index]=1;
  return {merged,local};
}

// 只复核当前线段，不计算全图距离场或搜索路径，适合高频动态障碍复查。
export function checkGlobalSegment(options={}) {
  const {map:sourceMap,localMap=null,segment,radius=0}=options ?? {};
  const fail=reason=>({segmentClear:false,reason});
  if(!Number.isFinite(radius) || radius<0)return fail('invalid_radius');
  if(!Array.isArray(segment) || segment.length!==2 || !segment.every(finitePoint))
    return fail('invalid_segment');
  const map=expandForPoints(sourceMap,segment,radius);
  if(!map)return fail('map_too_large');
  const startCell=cellFor(map,segment[0]);
  if(!startCell || !cellFor(map,segment[1]))return fail('segment_outside');
  let merged,local;
  const [a,b]=segment;
  const region={minX:Math.min(a.x,b.x)-radius,maxX:Math.max(a.x,b.x)+radius,
    minY:Math.min(a.y,b.y)-radius,maxY:Math.max(a.y,b.y)+radius};
  try { ({merged,local}=mergedForStart(map,localMap,startCell,region,radius===0)); }
  catch { return fail('invalid_local_map'); }
  const localClear=radius>0 ? localSweptCheck(map,local,radius) : ()=>true;
  return segmentIsClear(map,merged,null,segment[0],segment[1],radius) && localClear(segment[0],segment[1]) ?
    {segmentClear:true,reason:null} : fail('segment_blocked');
}

// 成功时 reason 为 null，失败时为稳定的原因码。
// 未提供当前线段时 segmentClear 为 null，否则返回布尔值。
export function planGlobalRoute(options={}) {
  const {map:sourceMap,localMap=null,start,goal,radius=0,currentSegment=null}=options ?? {};
  const fail=(reason,segmentClear=currentSegment===null ? null : false)=>({path:[],reason,segmentClear});
  if(!Number.isFinite(radius) || radius<0)return fail('invalid_radius');
  if(!finitePoint(start))return fail('invalid_start');
  if(!finitePoint(goal))return fail('invalid_goal');
  const validSegment=Array.isArray(currentSegment) && currentSegment.length===2 &&
    currentSegment.every(finitePoint);
  const points=validSegment ? [start,goal,...currentSegment] : [start,goal];
  const map=expandForPoints(sourceMap,points,radius);
  if(!map)return fail('map_too_large');
  const startCell=cellFor(map,start),goalCell=cellFor(map,goal);
  if(!startCell)return fail('start_outside');
  if(!goalCell)return fail('goal_outside');
  let merged,local;
  try { ({merged,local}=mergedForStart(map,localMap,startCell,null,radius===0)); }
  catch { return fail('invalid_local_map'); }
  const {safe,guaranteed}=inflate(map,merged,radius);
  const localClear=radius>0 ? localSweptCheck(map,local,radius) : ()=>true;
  const clear=(a,b)=>segmentIsClear(map,merged,guaranteed,a,b,radius) && localClear(a,b);
  const segmentClear=currentSegment===null ? null :
    validSegment && clear(currentSegment[0],currentSegment[1]);
  if(merged[goalCell.index]===2 || !clear(goal,goal))
    return fail('goal_blocked',segmentClear);
  if(merged[startCell.index]===2 || !clear(start,start))
    return fail('start_blocked',segmentClear);
  // 真实起终点可能落在安全的格间位置；先验直线，避免仅因格心不可用而拒绝。
  if(clear(start,goal))
    return {path:start.x===goal.x && start.y===goal.y ? [{x:start.x,y:start.y}] :
      [{x:start.x,y:start.y},{x:goal.x,y:goal.y}],reason:null,segmentClear};
  const refine=()=>{
    // 仅在局部地图中补充格边中点；占用证据和扫掠函数仍使用原始地图。
    const r=local.resolution_m/2,transform=transforms(map,local);
    const grid={width:local.width*2,height:local.height*2,resolution_m:r,
      origin:{x:local.origin.x-r/2,y:local.origin.y-r/2}};
    return findPath(map,merged,safe,guaranteed,start,goal,radius,localClear,
      {grid,toGrid:transform.toGrid,toMap:transform.toMap});
  };
  // 直线被局部已知障碍挡住时，先尝试有界细格，避免先搜索整个无路全局图。
  let refined;
  if(local && radius>0 && !localClear(start,goal)) {
    refined=refine();
    if(refined.path.length)return {...refined,segmentClear};
  }
  const normal=findPath(map,merged,safe,guaranteed,start,goal,radius,localClear);
  if(normal.path.length || normal.reason!=='no_path' || !local || radius===0)
    return {...normal,segmentClear};
  return {...(refined || refine()),segmentClear};
}
