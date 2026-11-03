# ByteTrack tracker subset

Source: [`ifzhang/ByteTrack` at
`d1bf0191adff59bc8fcfeaa0b33d3d1642552a99`](https://github.com/ifzhang/ByteTrack/tree/d1bf0191adff59bc8fcfeaa0b33d3d1642552a99/deploy/ncnn/cpp).
The source was copied directly from that commit's `deploy/ncnn/cpp` tree; no
repository checkout, detector, model, NCNN integration, demo, or binary is
included here. The upstream MIT license is in `LICENSE`.

Included upstream files:

- `include/BYTETracker.h`
- `include/STrack.h`
- `include/dataType.h`
- `include/kalmanFilter.h`
- `include/lapjv.h`
- `src/BYTETracker.cpp`
- `src/STrack.cpp`
- `src/kalmanFilter.cpp`
- `src/lapjv.cpp`
- `src/utils.cpp`

Local changes preserve the upstream two-stage high/low confidence association:

- Carry the current input detection index and original box through track
  updates, so vision output uses the matched detection rather than a Kalman
  prediction.
- Keep removed tracks only for the current update and prune them from lost
  tracks, avoiding an ever-growing history vector.
- Use correctly sized RAII buffers in LAPJV and throw on unsupported/error
  paths instead of pausing or exiting the process.
- Omit unused upstream gating, display and conversion helpers and types.
