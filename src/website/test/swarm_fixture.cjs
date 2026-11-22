const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');

const root = path.join(__dirname, '..', 'public');
const html = fs.readFileSync(path.join(root, 'index.html'), 'utf8');
const robot = (id, extra={}) => ({name:id,selected:true,online:true,map_ready:true,
  map_frame:id==='car'?'map':`${id}_map`,map_age_s:1,pose_local:[.5,.5,0],
  pose_common:extra.aligned===true?{x:.5,y:.5,yaw:0}:null,
  pose_reference:id==='acg720'?'acg720_lidar':`${id}_base_link`,
  aligned:false,alignment_status:'waiting_calibration',camera_model_ready:id==='car',
  camera_transform_ready:id==='car',manual_available:true,auto_navigation_available:false,...extra});
const stateFor = (participants, extra={}) => ({participants,anchor:participants[0],
  frame:participants[0]==='car'?'map':`${participants[0]}_map`,
  robots:{car:robot('car'),acg720:robot('acg720')},alignments:{},mapping_status:'local_only',
  mapping_readiness:['ACG720 等待相机标定'],exploring:false,
  readiness:['ACG720 未接入自动导航'],car:{mode:'idle',robot:{x:.5,y:.5,yaw:0},
    localization:{state:'tracking',reason:'laser_tracking',confirmations:0,required_confirmations:3,reset_counter:0}},...extra});
const flush = async () => {for(let i=0;i<40;i++) await Promise.resolve();};

async function dashboard(initial, localGrids={}) {
  const noop=()=>{}, markers=[], images=[], strokes=[], requests=[], commands=[], timers=[], mapResponses=[], events=[];
  let rasterCount=0;
  const element=id=>{
    const value={id,hidden:false,disabled:false,open:true,checked:false,dataset:{},handlers:{},
      paint:{markers:[],strokes:[],images:[],texts:[],fills:[]},
      classList:{add:noop,remove:noop},addEventListener(type,fn){this.handlers[type]=fn;}};
    const context={path:[],dash:[],createImageData:(w,h)=>({data:new Uint8ClampedArray(w*h*4)}),
      putImageData:image=>images.push(image),
      fillRect(){value.paint={markers:[],strokes:[],images:[],texts:[],fills:[]};},
      drawImage(image,x,y,width,height){value.paint.images.push({x,y,width,height,sourceWidth:image.width,sourceHeight:image.height});},
      beginPath(){this.path=[];},moveTo(x,y){this.path.push({command:'moveTo',x,y});},
      lineTo(x,y){this.path.push({command:'lineTo',x,y});},
      closePath(){this.path.push({command:'closePath'});},
      setLineDash(value){this.dash=[...value];},
      stroke(){const line={id,color:this.strokeStyle,width:this.lineWidth,path:this.path.map(p=>({...p})),dash:[...this.dash]};strokes.push(line);value.paint.strokes.push(line);},
      arc(x,y,radius){const marker={id,x,y,radius,color:this.fillStyle};markers.push(marker);value.paint.markers.push(marker);this.path.push({command:'arc',x,y,radius});},
      fill(){value.paint.fills.push({color:this.fillStyle,path:this.path.map(p=>({...p}))});},
      measureText(text){const size=Number((this.font || '10px sans-serif').match(/([\d.]+)px/)[1]);
        return {width:Array.from(String(text)).reduce((width,char)=>width+(char.charCodeAt(0)>255?size:.6*size),0)};},
      strokeText(text,x,y,maxWidth){value.paint.texts.push({operation:'stroke',text,x,y,maxWidth,font:this.font,
        width:this.measureText(text).width,color:this.strokeStyle,lineWidth:this.lineWidth,align:this.textAlign,baseline:this.textBaseline});},
      fillText(text,x,y,maxWidth){value.paint.texts.push({operation:'fill',text,x,y,maxWidth,font:this.font,
        width:this.measureText(text).width,color:this.fillStyle,align:this.textAlign,baseline:this.textBaseline});}};
    value.getContext=()=>context;
    return value;
  };
  const elements=Object.fromEntries([...html.matchAll(/\bid="([^"]+)"/g)].map(([,id])=>[id,element(id)]));
  let state=initial, grids=localGrids, now=10000;
  const network={available:true,suspend:false,postStatus:202,postBody:{accepted:true},holdPost:false};
  const grid=(mode,frame,contributors,extra={})=>({width:2,height:2,resolution:1,origin:[0,0,0],
    cells:Buffer.from([1,101,0,51]).toString('base64'),mode,frame,contributors,...extra});
  const response=(value,status=200)=>({ok:status>=200&&status<300,status,json:async()=>value});
  const mapResponse=value=>{mapResponses.push(value);return response(value);};
  const fetch=async(url,options={})=>{
    if(options.method==='POST'){
      commands.push({url,body:JSON.parse(options.body)});
      if(network.holdPost) return new Promise(resolve=>{
        network.resolvePost=()=>resolve(response(network.postBody,network.postStatus));
      });
      return response(network.postBody,network.postStatus);
    }
    requests.push(url);
    if(network.suspend) return new Promise(()=>{});
    if(!network.available) return response({},503);
    if(url.endsWith('/api/state')) return response(state);
    const view=new URL(url,'http://localhost').searchParams.get('view');
    if(state.participants.includes(view)) {
      const robot=state.robots[view];
      return mapResponse(grid(`${view}_local`,robot.map_frame,[view],{
        anchor:null,map_version:robot.map_version,map_session:robot.map_session,...(grids[view] || {})
      }));
    }
    throw Error('unexpected request '+url);
  };
  class Clock extends Date {static now(){return now;}}
  const sandbox={document:{hidden:false,getElementById:id=>elements[id],addEventListener:noop,
      createElement:tag=>{if(tag!=='canvas')throw Error('unexpected element '+tag);return element('raster-'+(++rasterCount));}},
    window:{dispatchEvent(event){events.push(event);return true;}},
    CustomEvent:class {constructor(type,options={}){this.type=type;this.detail=options.detail ?? null;}},
    fetch,Date:Clock,Math,Number,String,Object,Array,Promise,
    Uint8ClampedArray,atob:value=>Buffer.from(value,'base64').toString('binary'),
    AbortController:class {constructor(){this.signal={};}abort(){}},setTimeout:()=>1,clearTimeout:noop,
    setInterval:fn=>{timers.push(fn);return timers.length;},clearInterval:noop};
  for(const name of ['map_palette.js','swarm_app.js'])
    vm.runInNewContext(fs.readFileSync(path.join(root,'js',name),'utf8'),sandbox,{filename:name});
  await flush();
  const poll=async(elapsed=1000)=>{now+=elapsed;await timers[0]();await flush();};
  return {elements,requests,commands,markers,images,strokes,mapResponses,events,network,poll,paint:id=>elements[id].paint,
    async update(next,gridUpdate,elapsed=2100){state=next;if(gridUpdate)grids=gridUpdate;await poll(elapsed);},
    async disconnect(){network.available=false;await poll(3000);}};
}

module.exports = {dashboard,robot,stateFor,html,flush};
