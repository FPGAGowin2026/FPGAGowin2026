const assert=require('node:assert/strict');
const fs=require('node:fs');
const path=require('node:path');
const test=require('node:test');
const vm=require('node:vm');

test('map palette keeps unknown, free and occupied cells distinct',()=>{
  const source=fs.readFileSync(path.join(__dirname,'../public/js/map_palette.js'),'utf8');
  const values=[127,255,0,128,125], bytes=new Uint8ClampedArray(values.flatMap(v=>[v,v,v,255]));
  let painted;
  const context={drawImage(){},getImageData:()=>({data:bytes}),putImageData:image=>{painted=image.data;}};
  const document={createElement:()=>({width:0,height:0,getContext:()=>context})}, window={};
  vm.runInNewContext(source,{document,window,Math},{filename:'map_palette.js'});
  window.BX72MapPalette.colorizeImage({naturalWidth:5,naturalHeight:1});
  assert.deepEqual(Array.from(painted.slice(0,12)),[211,231,238,255,252,255,251,255,33,68,81,255]);
  assert.notDeepEqual(Array.from(painted.slice(12,16)),Array.from(painted.slice(0,4)));
  assert.notDeepEqual(Array.from(painted.slice(16,20)),Array.from(painted.slice(0,4)));
  for(let occupancy=0;occupancy<=100;occupancy++) {
    assert.notEqual(255-Math.floor(occupancy*255/100),127,'known occupancy cannot collide with the unknown pixel');
  }
});
