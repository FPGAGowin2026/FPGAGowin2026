# GoWin workspace snapshot

This directory is a Git-friendly snapshot of the outer `GoWin` workspace.

Included:

- `choiceRobot`, `fpga_rtl`, `fpga_gateway`, `robot_bringup`, `sensor_examples`, and `doc`
- source files, Gowin project inputs, constraints, testbenches, scripts, reports, and required assets
- `sensor_examples/acg720_tutorial/raw` as the slim source-only copy

Excluded to keep the repository small and reproducible:

- nested `.git` metadata
- Gowin `impl/` and firmware/simulation `build/` outputs
- `sim/work/` databases and generated netlists/reports
- waveforms, bitstreams, P&R databases, compiled objects, logs, and temporary files

The original full workspace remains outside this repository at the local outer `GoWin` path.
