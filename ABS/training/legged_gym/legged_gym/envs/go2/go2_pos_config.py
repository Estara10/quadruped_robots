# SPDX-FileCopyrightText: Copyright (c) 2021 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
# list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# Copyright (c) 2021 ETH Zurich, Nikita Rudin

from legged_gym.envs.base.legged_robot_config import LeggedRobotCfgPPO, LeggedRobotCfgPPOLagrangian
from legged_gym.envs.base.legged_robot_pos_config import LeggedRobotPosCfg
import numpy as np

class Go2PosRoughCfg( LeggedRobotPosCfg ):
    class env(LeggedRobotPosCfg.env):
        num_observations = 61
        num_envs = 1280
        episode_length_s = 9

    class init_state( LeggedRobotPosCfg.init_state ):
        pos = [0.0, 0.0, 0.35] # Go2 standing height
        default_joint_angles = {
            'FL_hip_joint': 0.,
            'RL_hip_joint': 0.,
            'FR_hip_joint': 0.,
            'RR_hip_joint': 0.,

            'FL_thigh_joint': 0.8,
            'RL_thigh_joint': 0.8,
            'FR_thigh_joint': 0.8,
            'RR_thigh_joint': 0.8,

            'FL_calf_joint': -1.5,
            'RL_calf_joint': -1.5,
            'FR_calf_joint': -1.5,
            'RR_calf_joint': -1.5,
        }

    class commands:
        curriculum = False
        max_curriculum = 1.
        num_commands = 3
        class ranges:
            use_polar = False
            pos_1 = [1.5, 7.5]
            pos_2 = [-2.0, 2.0]
            heading = [-0.3, 0.3]

    class control( LeggedRobotPosCfg.control ):
        control_type = 'P'
        stiffness = {'joint': 30.}   # Go2: stiffer gains for heavier legs
        damping = {'joint': 0.65}
        action_scale = 0.25
        decimation = 4

    class asset( LeggedRobotPosCfg.asset ):
        file = '{LEGGED_GYM_ROOT_DIR}/resources/robots/go2/urdf/go2.urdf'
        name = "go2"
        foot_name = "foot"
        penalize_contacts_on = ["thigh", "calf"]
        terminate_after_contacts_on = ["base"]
        self_collisions = 0

        load_dynamic_object = True
        object_files = {
        '{LEGGED_GYM_ROOT_DIR}/resources/objects/cylindar.urdf': 0.4,
        }
        object_num = 8
        test_mode = False
        test_obj_pos = []

    class terrain( LeggedRobotPosCfg.terrain ):
        terrain_types = ['flat', 'rough', 'low_obst']
        terrain_proportions = [0.5, 0.5, 0.5]
        measure_heights = False

    class domain_rand:
        randomize_friction = True
        friction_range = [-0.2, 1.25]
        randomize_base_mass = True
        added_mass_range = [-2.0, 2.0]  # wider range for heavier Go2
        randomize_dof_bias = True
        max_dof_bias = 0.08
        randomize_timer_minus = 2.0

        push_robots = True
        push_interval_s = 2.5
        max_push_vel_xy = 0.0

        randomize_yaw = True
        init_yaw_range = [-3.14, 3.14]
        randomize_roll = False
        randomize_pitch = False
        randomize_xy = True
        init_x_range = [-0.5, 0.5]
        init_y_range = [-0.5, 0.5]
        randomize_velo = True
        init_vlinx_range = [-0.5,0.5]
        init_vliny_range = [-0.5,0.5]
        init_vlinz_range = [-0.5,0.5]
        init_vang_range = [-0.5,0.5]
        randomize_init_dof = True
        init_dof_factor=[0.5, 1.5]
        stand_bias3 = [0.0, 0.0, 0.0]

        erfi = True
        erfi_torq_lim = 7.0/9

    class sensors:
        class ray2d:
            enable = True
            log2 = True
            min_dist = 0.1
            max_dist = 6.0
            theta_start = - np.pi/4
            theta_end = np.pi/4 + 0.0001
            theta_step = np.pi/20
            x_0 = -0.05
            y_0 = 0.0
            front_rear = False
            illusion = True
            raycolor = (0,0.5,0.5)

        class depth_cam:
            enable = False
            resolution = [1280//8,720//8]
            x = 0.0
            y = 0
            z = 0.27
            far_plane = 10.0
            hfov = 102.0
            min_ = 0.1
            max_ = 6.0

    class normalization:
        class obs_scales:
            lin_vel = 1.0
            ang_vel = 1.0
            dof_pos = 1.0
            dof_vel = 0.2
            height_measurements = 2.0
            ray2d = 1.0
        clip_observations = 100.
        clip_actions = 100.

    class noise:
        add_noise = True
        noise_level = 1.0
        class noise_scales:
            dof_pos = 0.01
            dof_vel = 1.5
            lin_vel = 0.1
            ang_vel = 0.2
            gravity = 0.05
            height_measurements = 0.1
            ray2d = 0.2

    class rewards():
        class scales():
            termination = -100.
            reach_pos_target_soft = 60.0
            reach_pos_target_tight = 60.0
            reach_heading_target = 30.0
            reach_pos_target_times_heading = 0.0
            velo_dir = 10.0
            torques = -0.0005
            dof_pos_limits = -20.0
            dof_vel = -0.0005
            torque_limits = -20.0
            dof_vel_limits = -20.0
            lin_vel_z = -2.0
            ang_vel_xy = -0.05
            dof_acc = -2.0e-7
            collision = -100.
            feet_collision = -100.
            action_rate = -0.01
            stand_still_pos = -10.0
            orientation = -20.0
            fly = -20.0
            nomove = -20.0

        soft_dof_pos_limit = 0.95
        base_height_target = 0.25
        only_positive_rewards = False
        position_target_sigma_soft = 2.0
        position_target_sigma_tight = 0.5
        heading_target_sigma = 1.0
        rew_duration = 2.0
        soft_dof_vel_limit = 0.9
        soft_torque_limit = 0.85
        max_contact_force = 100.

class Go2PosRoughCfgPPO( LeggedRobotCfgPPO ):
    class algorithm( LeggedRobotCfgPPO.algorithm ):
        entropy_coef = 0.003
    class runner( LeggedRobotCfgPPO.runner ):
        run_name = ''
        experiment_name = 'go2_pos_rough'
