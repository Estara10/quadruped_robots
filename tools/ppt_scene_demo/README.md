# Go2 PPT scene demo

This tool generates three visual-only static obstacle scenes. They are **not** formal experiment scenarios, benchmark results, or Phase/Gate evidence. It reuses the existing Go2 model and `scripts/launch_abs_obstacle.sh` (ROS2 controller, automatic stand and forward command). No training or switching code is changed.

From the repository root:

```bash
python3 tools/ppt_scene_demo/run.py --scene sparse
python3 tools/ppt_scene_demo/run.py --scene medium
python3 tools/ppt_scene_demo/run.py --scene dense
```

`--camera ppt` is the default. The XML generated in `unitree_mujoco/unitree_robots/go2/scene_ppt_<scene>.xml` includes the original `go2.xml`, uses the blue sky and checker ground from `scene_flat.xml`, and sets the same free-camera defaults in each scene. Run only one demo at a time. Stop with Ctrl+C in the launching terminal.

The obstacle tuples in `run.py` are `(x, y, half_length_x, half_width_y, height)` in metres. Go2 starts at `(0, 0)` facing +X. Boxes are staggered at different lateral offsets, including positions close to the middle. Keep enough clearance around `y=0` when editing them. `CAMERA` controls the common view center, extent, azimuth and elevation. For comparable screenshots, use the same window size and do not move the camera between runs.

The MuJoCo UI's Screenshot button currently does not write a file in this repository build. Use a desktop/window screenshot tool. For 1920×1080 output, press `F5` for full screen, then `Tab` and `Shift+Tab` to hide the left and right panels. Capture during the initial forward segment; the reused controller may later invoke its safety veto. The scene contains no HUD or labels. The three scene XMLs can be generated without starting the runtime using `--prepare-only`.

The supplied `screenshots/{sparse,medium,dense}.png` are example 1920×1080 window captures from separate live launches. They show scene appearance only and are not evidence of obstacle avoidance or sustained controller performance.
