# CLAUDE.md — ABS (paper source code)

**Reference implementation** of the ABS paper. All algorithms must match this code.
Parent: `/home/lidio/quadruped_robots/CLAUDE.md`

## Structure

```
ABS/
├── training/legged_gym/
│   ├── legged_gym/envs/base/
│   │   ├── legged_robot_pos.py          # Goal-reaching env (training reference)
│   │   └── legged_robot_rec.py          # Recovery policy env
│   ├── legged_gym/envs/go2/             # Go2-specific configs
│   ├── legged_gym/scripts/
│   │   ├── train.py                     # Policy training
│   │   ├── testbed.py                   # RA training + end-to-end test
│   │   └── train_depth_resnet.py        # ResNet18 ray-prediction training
│   └── legged_gym/utils/math.py         # circle_ray_query (2D raycasting)
└── deployment/src/abs_src/
    ├── depth_obstacle_depth_goal_ros.py # ** ALGORITHM REFERENCE **
    │   L475-488: RA + recovery trigger
    │   L498-525: GD twist optimization
    │   L532-536: Recovery policy inference
    │   L145-175: Goal→commands transform
    └── publisher_depthimg_linvel.py      # ZED depth + tracking
```

## Key Training Params (Go2)

Kp=30, Kd=0.65, action_scale=0.25, decimation=4, contact_threshold=1.0N
Target: forward [1.5,7.5]m, lateral [-2,2]m. Ray2d: 11 rays, [-45°,+45°], max 6m.

## Exported Models

- Agile: `training/legged_gym/logs/go2_pos_rough/exported/`
- Recovery: `training/legged_gym/logs/go2_rec_rough/exported/`
- RA: `training/legged_gym/logs/go2_pos_rough/exported/RA/`
- Ray-Pred: `training/legged_gym/legged_gym/depth_logs/.../depth_lidar_model_..._250.pt`
- ResNet18: pretrained ImageNet, fc→11. Input 160×90 depth (log2)→3ch→(3,90,160). Output 11 log2 rays.
