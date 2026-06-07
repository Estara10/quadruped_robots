# CLAUDE.md — unitree_mujoco

**Standalone MuJoCo simulator** for Go2. Communicates with ROS2 via DDS.
Parent: `/home/lidio/quadruped_robots/CLAUDE.md`

## Key Files

| File | Role |
|------|------|
| `simulate/src/unitree_sdk2_bridge.h` | **Core**: DDS bridge, ray2d computation, qpos shm |
| `simulate/src/main.cc` | Entry point, scene loading |
| `simulate/config.yaml` | Default robot + scene selection |
| `unitree_robots/go2/go2.xml` | Go2 MJCF model (joints, sensors, actuators) |
| `unitree_robots/go2/scene.xml` | Flat ground scene |
| `unitree_robots/go2/scene_terrain.xml` | Terrain + obstacles scene |
| `unitree_robots/go2/scene_test1~5.xml` | Random obstacle test scenes |

## Ray2d (unitree_sdk2_bridge.h)

2D ray-circle intersection. 11 rays (θ=-45°~+45°, step=9°), max 6m, log2 output.
Origin: body XY + (-0.05, 0) offset. Geom filter: skip plane(0)/hfield(1)/mesh(7).
Output to `/mujoco_ray2d` (11 floats) + `/mujoco_qpos` (19 doubles) shm.

## Scenes

Default scene from `simulate/config.yaml` (currently `scene_terrain.xml`).
Override with `-s <scene_file>` flag. Scene files include `go2.xml` via `<include>`.
`meshdir="assets"` in go2.xml means hfield files must be in `assets/` subdirectory.

## Building

```bash
cd simulate/build2 && cmake .. -DCMAKE_BUILD_TYPE=Release && make -j$(nproc)
```

Binary: `simulate/build2/unitree_mujoco`
