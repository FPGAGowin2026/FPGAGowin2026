/* 全局图和局部图共用的栅格游程解码；地图头和坐标域由调用方校验。 */
(function (root, factory) {
  const api = factory();
  if (typeof module === 'object' && module.exports) module.exports = api;
  if (root) root.BX72GridCodec = api;
})(typeof globalThis === 'undefined' ? this : globalThis, function () {
  'use strict';

  function decodeCells(rle, size) {
    if (!Array.isArray(rle) || rle.length % 2 !== 0 || rle.length > size * 2)
      throw new Error('栅格游程无效');
    const cells = new Uint8Array(size);
    let cursor = 0;
    for (let i = 0; i < rle.length; i += 2) {
      const value = rle[i], count = rle[i + 1];
      if (!Number.isInteger(value) || value < 0 || value > 2 ||
          !Number.isInteger(count) || count < 1 || cursor + count > size)
        throw new Error('栅格游程无效');
      cells.fill(value, cursor, cursor + count);
      cursor += count;
    }
    if (cursor !== size) throw new Error('栅格游程不完整');
    return cells;
  }

  return {decodeCells};
});
