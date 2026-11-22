/* The PNG encoder reserves gray 127 for unknown; known occupancy never encodes to 127. */
(() => {
  'use strict';
  const unknown=[211,231,238], free=[252,255,251], occupied=[33,68,81];
  function write(data, offset, brightness) {
    if (brightness===null) {
      data[offset]=unknown[0];data[offset+1]=unknown[1];data[offset+2]=unknown[2];
    } else {
      const t=brightness/255;
      for (let n=0;n<3;n++) data[offset+n]=Math.round(occupied[n]+(free[n]-occupied[n])*t);
    }
    data[offset+3]=255;
  }
  function colorizeImage(image) {
    const canvas=document.createElement('canvas');
    canvas.width=image.naturalWidth;canvas.height=image.naturalHeight;
    const context=canvas.getContext('2d',{willReadFrequently:true});
    context.drawImage(image,0,0);
    const pixels=context.getImageData(0,0,canvas.width,canvas.height);
    for(let i=0;i<pixels.data.length;i+=4) {
      const gray=pixels.data[i];write(pixels.data,i,gray===127?null:gray);
    }
    context.putImageData(pixels,0,0);
    return canvas;
  }
  function writeOccupancy(data, offset, occupancy) {
    write(data,offset,occupancy<0?null:(100-Math.min(occupancy,100))*255/100);
  }
  window.BX72MapPalette={colorizeImage,writeOccupancy};
})();
