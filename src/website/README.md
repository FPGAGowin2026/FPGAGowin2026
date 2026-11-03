# 网页控制台

网页展示视频、局部和全局地图、遥测与任务状态，可在地图上选择航点并提交给 Jetson。全局 A* 和航点调度在 Jetson 执行；网页服务复用同级 `../ugv_ws` 的地图解码实现。

```bash
cd src/website
npm ci
node server.mjs --jetson 192.168.1.103 --port 6200
```

使用 Node.js 22 或更新版本，将示例地址改为实际 Jetson 地址，在本机访问 `http://127.0.0.1:8765`。任务提交后由 Jetson 持有；关闭网页不代表任务已取消。
