//
// Created by biao on 24-10-6.
//

#include "rl_quadruped_controller/FSM/StateRL.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <algorithm>
#include <rclcpp/logging.hpp>
#include <yaml-cpp/yaml.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include <cmath>
#include <limits>

template <typename T>
std::vector<T> ReadVectorFromYaml(const YAML::Node& node)
{
    std::vector<T> values;
    for (const auto& val : node)
    {
        values.push_back(val.as<T>());
    }
    return values;
}

StateRL::StateRL(CtrlInterfaces& ctrl_interfaces,
                 CtrlComponent& ctrl_component,
                 const std::vector<double>& target_pos) :
    FSMState(FSMStateName::RL, "rl", ctrl_interfaces),
    node_(ctrl_component.node_),
    ctrl_component_(ctrl_component),
    enable_estimator_(ctrl_component.enable_estimator_),
    estimator_(ctrl_component.estimator_)
{
    // Skip declare if already defined in YAML config
    if (!node_->has_parameter("robot_pkg")) {
        node_->declare_parameter("robot_pkg", robot_pkg_);
    }
    if (!node_->has_parameter("model_folder")) {
        node_->declare_parameter("model_folder", model_folder_);
    }
    if (!node_->has_parameter("use_rl_thread")) {
        node_->declare_parameter("use_rl_thread", use_rl_thread_);
    }
    robot_pkg_ = node_->get_parameter("robot_pkg").as_string();
    model_folder_ = node_->get_parameter("model_folder").as_string();
    use_rl_thread_ = node_->get_parameter("use_rl_thread").as_bool();

    RCLCPP_INFO(node_->get_logger(), "Using robot model from %s", robot_pkg_.c_str());
    const std::string package_share_directory = ament_index_cpp::get_package_share_directory(robot_pkg_);
    const std::string model_path = package_share_directory + "/config/" + model_folder_;

    for (int i = 0; i < 12; i++)
    {
        init_pos_[i] = target_pos[i];
    }

    // read params from yaml
    loadYaml(model_path);

    if (!params_.observations_history.empty())
    {
        history_obs_buf_ = std::make_shared<ObservationBuffer>(1, params_.num_observations,
                                                               params_.observations_history.size());
    }

    RCLCPP_INFO(node_->get_logger(), "Model loading: %s", params_.model_name.c_str());
    model_ = torch::jit::load(model_path + "/" + params_.model_name);

    // ===== Verification logs (remove after confirmed) =====
    RCLCPP_INFO(node_->get_logger(), "[VERIFY] num_observations=%d, num_of_dofs=%d",
                params_.num_observations, params_.num_of_dofs);
    RCLCPP_INFO(node_->get_logger(), "[VERIFY] observations order: %s",
                [&]() { std::string s; for (auto& o : params_.observations) s += o + " "; return s; }().c_str());
    RCLCPP_INFO(node_->get_logger(), "[VERIFY] rl_kp[0]=%.2f, rl_kd[0]=%.2f",
                params_.rl_kp[0][0].item<double>(), params_.rl_kd[0][0].item<double>());
    RCLCPP_INFO(node_->get_logger(), "[VERIFY] scales: action=%.2f lin_vel=%.2f ang_vel=%.2f dof_pos=%.2f dof_vel=%.2f",
                params_.action_scale, params_.lin_vel_scale, params_.ang_vel_scale,
                params_.dof_pos_scale, params_.dof_vel_scale);
    RCLCPP_INFO(node_->get_logger(), "[VERIFY] default_dof_pos: %.4f %.4f %.4f (first 3 joints = FR hip/thigh/calf)",
                params_.default_dof_pos[0][0].item<double>(), params_.default_dof_pos[0][1].item<double>(),
                params_.default_dof_pos[0][2].item<double>());
    RCLCPP_INFO(node_->get_logger(), "[VERIFY] ABS: max_ep_len=%.1fs contact_thr=%.1fN ray2d_count=%d ray2d_max_range=%.1fm",
                params_.abs_max_episode_length_s, params_.abs_contact_threshold,
                params_.abs_ray2d_count, params_.abs_ray2d_max_range);
    RCLCPP_INFO(node_->get_logger(), "[VERIFY] decimation=%d frequency=%dHz dt=%.4fs",
                params_.decimation, ctrl_interfaces_.frequency_,
                static_cast<double>(params_.decimation) / ctrl_interfaces_.frequency_);
    RCLCPP_INFO(node_->get_logger(), "[VERIFY] use_rl_thread=%s policy_joint_order=%s",
                use_rl_thread_ ? "true" : "false", params_.policy_joint_order.c_str());
    // ===== End verification logs =====

    // Load RA value network (optional — skip if model file not found)
    try
    {
        ra_model_ = torch::jit::load(model_path + "/" + params_.ra_model_name);
        ra_loaded_ = true;
        // Share RA model with CtrlComponent for recovery use
        ctrl_component_.ra_model = &ra_model_;
        ctrl_component_.ra_loaded = true;
        ctrl_component_.ra_threshold = params_.ra_threshold;
        ctrl_component_.recovery_steps = params_.recovery_steps;
        RCLCPP_INFO(node_->get_logger(), "[RA] Value network loaded: %s", params_.ra_model_name.c_str());
    }
    catch (const std::exception& e)
    {
        RCLCPP_WARN(node_->get_logger(), "[RA] Value network not found (%s), RA disabled",
                    params_.ra_model_name.c_str());
        ra_loaded_ = false;
    }

    // Load recovery policy (ROS1 lines 368-371: both models loaded at startup)
    try
    {
        const std::string rec_model_path = ament_index_cpp::get_package_share_directory(robot_pkg_)
                                           + "/config/rec/policy.pt";
        rec_model_ = torch::jit::load(rec_model_path);
        rec_loaded_ = true;
        RCLCPP_INFO(node_->get_logger(), "[REC] Recovery policy loaded: %s", rec_model_path.c_str());
    }
    catch (const std::exception& e)
    {
        RCLCPP_WARN(node_->get_logger(), "[REC] Recovery policy not found, manual RL_REC still available");
        rec_loaded_ = false;
    }

    // for (const auto &param: model_.parameters()) {
    //     std::cout << "Parameter dtype: " << param.dtype() << std::endl;
    // }


    if (use_rl_thread_)
    {
        rl_thread_ = std::thread([&]{
            while (true)
            {
                try
                {
                    executeAndSleep(
                        [&]
                        {
                            if (running_)
                            {
                                runModel();
                            }
                        },
                        ctrl_interfaces_.frequency_ / params_.decimation);
                }
                catch (const std::exception& e)
                {
                    running_ = false;
                    RCLCPP_ERROR(rclcpp::get_logger("StateRL"), "Error in RL thread: %s", e.what());
                }
            }
        });
        setThreadPriority(60, rl_thread_);
    }
}

bool StateRL::useRos1PolicyOrder() const
{
    return params_.policy_joint_order == "ros1_fl_fr_rl_rr";
}

torch::Tensor StateRL::ctrlToPolicyDofOrder(const torch::Tensor& ctrl_order) const
{
    if (!useRos1PolicyOrder())
    {
        return ctrl_order;
    }

    // Controller order is FR, FL, RR, RL. ROS1 ABS policy order is FL, FR, RL, RR.
    static const std::vector<int64_t> indices = {3, 4, 5, 0, 1, 2, 9, 10, 11, 6, 7, 8};
    const auto index = torch::tensor(indices, torch::TensorOptions().dtype(torch::kLong).device(ctrl_order.device()));
    return ctrl_order.index_select(1, index);
}

torch::Tensor StateRL::policyToCtrlDofOrder(const torch::Tensor& policy_order) const
{
    if (!useRos1PolicyOrder())
    {
        return policy_order;
    }

    // Inverse of ctrlToPolicyDofOrder.
    static const std::vector<int64_t> indices = {3, 4, 5, 0, 1, 2, 9, 10, 11, 6, 7, 8};
    const auto index = torch::tensor(indices, torch::TensorOptions().dtype(torch::kLong).device(policy_order.device()));
    return policy_order.index_select(1, index);
}

torch::Tensor StateRL::ctrlToPolicyContactOrder(const torch::Tensor& ctrl_order) const
{
    if (!useRos1PolicyOrder())
    {
        return ctrl_order;
    }

    // Controller contact order is FR, FL, RR, RL. ROS1 observation order is FL, FR, RL, RR.
    static const std::vector<int64_t> indices = {1, 0, 3, 2};
    const auto index = torch::tensor(indices, torch::TensorOptions().dtype(torch::kLong).device(ctrl_order.device()));
    return ctrl_order.index_select(1, index);
}

StateRL::~StateRL()
{
    running_ = false;
    if (rl_thread_.joinable())
    {
        rl_thread_.join();
    }
    // Cleanup ray2d shared memory
    if (ray2d_shm_ptr_ != nullptr && ray2d_shm_ptr_ != MAP_FAILED)
    {
        munmap(ray2d_shm_ptr_, 11 * sizeof(float));
    }
    if (ray2d_shm_fd_ >= 0)
    {
        close(ray2d_shm_fd_);
    }
}

void StateRL::enter()
{
    // Init observations
    obs_.lin_vel = torch::tensor({{0.0, 0.0, 0.0}});
    obs_.ang_vel = torch::tensor({{0.0, 0.0, 0.0}});
    obs_.gravity_vec = torch::tensor({{0.0, 0.0, -1.0}});
    obs_.commands = torch::tensor({{0.0, 0.0, 0.0}});
    obs_.base_quat = torch::tensor({{0.0, 0.0, 0.0, 1.0}});
    obs_.dof_pos = params_.default_dof_pos;
    obs_.dof_vel = torch::zeros({1, params_.num_of_dofs});
    obs_.actions = torch::zeros({1, params_.num_of_dofs});
    obs_.contact = torch::zeros({1, 4});
    // Init ray2d from shared memory (or fallback to constant)
    ray2d_shm_fd_ = shm_open("/mujoco_ray2d", O_RDONLY, 0666);
    if (ray2d_shm_fd_ >= 0)
    {
        ray2d_shm_ptr_ = static_cast<float*>(
            mmap(NULL, 11 * sizeof(float), PROT_READ, MAP_SHARED, ray2d_shm_fd_, 0));
        if (ray2d_shm_ptr_ == MAP_FAILED)
        {
            RCLCPP_WARN(rclcpp::get_logger("StateRL"),
                "[Ray2D] mmap failed: %s, using constant ray2d", strerror(errno));
            ray2d_shm_ptr_ = nullptr;
            close(ray2d_shm_fd_);
            ray2d_shm_fd_ = -1;
        }
        else
        {
            RCLCPP_INFO(rclcpp::get_logger("StateRL"),
                "[Ray2D] Shared memory connected: /mujoco_ray2d");
        }
    }
    else
    {
        RCLCPP_WARN(rclcpp::get_logger("StateRL"),
            "[Ray2D] shm_open failed: %s, using constant ray2d", strerror(errno));
    }

    // Always initialize obs_.ray2d (will be updated from shm in runModel if available)
    obs_.ray2d = torch::ones({1, params_.abs_ray2d_count}) * std::log2(params_.abs_ray2d_max_range);
    episode_timer_ = 0.0;
    rl_step_count_ = 0;
    sync_decimation_counter_ = 0;
    soft_start_step_ = 0;

    // Init output
    output_torques = torch::zeros({1, params_.num_of_dofs});
    output_dof_pos_ = params_.default_dof_pos;

    // Init robot_command_ to stand position (prevent sending zero commands before RL thread runs)
    for (int i = 0; i < params_.num_of_dofs; ++i)
    {
        robot_command_.motor_command.q[i] = params_.default_dof_pos[0][i].item<double>();
        robot_command_.motor_command.dq[i] = 0;
        robot_command_.motor_command.kp[i] = params_.rl_kp[0][i].item<double>();
        robot_command_.motor_command.kd[i] = params_.rl_kd[0][i].item<double>();
        robot_command_.motor_command.tau[i] = 0;
    }

    // Init control
    control_.x = 0.0;
    control_.y = 0.0;
    control_.yaw = 0.0;

    // history
    if (!params_.observations_history.empty()) {
        history_obs_buf_->clear();
    }

    running_ = true;

    // Diagnostic: confirm estimator + recovery status
    RCLCPP_INFO(rclcpp::get_logger("StateRL"),
        "[RL] enable_estimator_=%d rec_loaded=%d ra_loaded=%d ra_threshold=%.4f",
        enable_estimator_, rec_loaded_, ra_loaded_, params_.ra_threshold);

    // Confirm ray2d shared memory status on each RL entry
    if (ray2d_shm_ptr_ != nullptr) {
        RCLCPP_INFO(rclcpp::get_logger("StateRL"), "[RL] Ray2d: shm connected, policy active");
    } else {
        RCLCPP_INFO(rclcpp::get_logger("StateRL"), "[RL] Ray2d: using constant log2(%.1f)", params_.abs_ray2d_max_range);
    }
}

void StateRL::run(const rclcpp::Time&/*time*/, const rclcpp::Duration&/*period*/)
{
    getState();
    if (!use_rl_thread_)
    {
        sync_decimation_counter_++;
        if (sync_decimation_counter_ >= params_.decimation)
        {
            sync_decimation_counter_ = 0;
            runModel();
        }
    }
    setCommand();
}

void StateRL::logEvalTelemetry(double robot_wx, double robot_wy, double robot_yaw,
                               double goal_wx, double goal_wy, double dist_to_goal,
                               double body_x, double body_y, double heading_cmd,
                               bool arrived, bool in_recovery, int recovery_hold_left,
                               double recovery_vx, double recovery_vy, double recovery_wz) const
{
    if (!params_.eval_telemetry_enabled)
    {
        return;
    }

    const int interval = std::max(1, params_.eval_telemetry_interval_steps);
    if (rl_step_count_ % interval != 0)
    {
        return;
    }

    double min_ray_log = std::numeric_limits<double>::infinity();
    double max_ray_log = -std::numeric_limits<double>::infinity();
    const int ray_count = std::min(params_.abs_ray2d_count, static_cast<int>(obs_.ray2d.size(1)));
    for (int i = 0; i < ray_count; ++i)
    {
        const double ray = obs_.ray2d[0][i].item<double>();
        min_ray_log = std::min(min_ray_log, ray);
        max_ray_log = std::max(max_ray_log, ray);
    }
    const double min_ray_m = std::pow(2.0, min_ray_log);

    const double action_min = obs_.actions.min().item<double>();
    const double action_max = obs_.actions.max().item<double>();
    const double elapsed_s = rl_step_count_ * static_cast<double>(params_.decimation) / ctrl_interfaces_.frequency_;
    const double ra_value = ra_loaded_ ? ra_value_ : std::numeric_limits<double>::quiet_NaN();

    RCLCPP_INFO(rclcpp::get_logger("StateRL"),
        "[EVAL] step=%d t=%.3f robot=(%.3f,%.3f) yaw=%.3f goal=(%.3f,%.3f) dist=%.3f "
        "body=(%.3f,%.3f) heading=%.3f arrived=%d ra=%.5f entry=%.5f recovery=%d hold=%d "
        "twist=(%.3f,%.3f,%.3f) min_ray_log=%.3f max_ray_log=%.3f min_ray_m=%.3f "
        "lin_vel=(%.3f,%.3f,%.3f) ang_vel=(%.3f,%.3f,%.3f) action_range=(%.3f,%.3f) "
        "contact=(%.0f,%.0f,%.0f,%.0f)",
        rl_step_count_, elapsed_s, robot_wx, robot_wy, robot_yaw, goal_wx, goal_wy, dist_to_goal,
        body_x, body_y, heading_cmd, arrived ? 1 : 0, ra_value, params_.ra_threshold,
        in_recovery ? 1 : 0, recovery_hold_left, recovery_vx, recovery_vy, recovery_wz,
        min_ray_log, max_ray_log, min_ray_m,
        obs_.lin_vel[0][0].item<double>(), obs_.lin_vel[0][1].item<double>(), obs_.lin_vel[0][2].item<double>(),
        obs_.ang_vel[0][0].item<double>(), obs_.ang_vel[0][1].item<double>(), obs_.ang_vel[0][2].item<double>(),
        action_min, action_max,
        obs_.contact[0][0].item<double>(), obs_.contact[0][1].item<double>(),
        obs_.contact[0][2].item<double>(), obs_.contact[0][3].item<double>());
}

void StateRL::logSymmetryDebug(double robot_wx, double robot_wy, double robot_yaw,
                               double body_y, double heading_cmd, bool in_recovery,
                               const torch::Tensor& policy_actions,
                               const torch::Tensor& ctrl_actions) const
{
    if (!params_.symmetry_debug_enabled)
    {
        return;
    }

    const int interval = std::max(1, params_.eval_telemetry_interval_steps);
    if (rl_step_count_ % interval != 0)
    {
        return;
    }

    auto value = [](const torch::Tensor& tensor, int index) {
        return tensor[0][index].item<double>();
    };

    const double fl_fr_thigh_action = value(ctrl_actions, 4) - value(ctrl_actions, 1);
    const double rl_rr_thigh_action = value(ctrl_actions, 10) - value(ctrl_actions, 7);
    const double fl_fr_calf_action = value(ctrl_actions, 5) - value(ctrl_actions, 2);
    const double rl_rr_calf_action = value(ctrl_actions, 11) - value(ctrl_actions, 8);

    const double fl_fr_thigh_q = output_dof_pos_[0][4].item<double>() - output_dof_pos_[0][1].item<double>();
    const double rl_rr_thigh_q = output_dof_pos_[0][10].item<double>() - output_dof_pos_[0][7].item<double>();
    const double fl_fr_calf_q = output_dof_pos_[0][5].item<double>() - output_dof_pos_[0][2].item<double>();
    const double rl_rr_calf_q = output_dof_pos_[0][11].item<double>() - output_dof_pos_[0][8].item<double>();

    const double fl_fr_thigh_state = obs_.dof_pos[0][4].item<double>() - obs_.dof_pos[0][1].item<double>();
    const double rl_rr_thigh_state = obs_.dof_pos[0][10].item<double>() - obs_.dof_pos[0][7].item<double>();
    const double fl_fr_calf_state = obs_.dof_pos[0][5].item<double>() - obs_.dof_pos[0][2].item<double>();
    const double rl_rr_calf_state = obs_.dof_pos[0][11].item<double>() - obs_.dof_pos[0][8].item<double>();

    RCLCPP_INFO(rclcpp::get_logger("StateRL"),
        "[SYMM] step=%d robot=(%.3f,%.3f) yaw=%.3f body_y=%.3f heading=%.3f recovery=%d "
        "policy_FL=(%.3f,%.3f,%.3f) policy_FR=(%.3f,%.3f,%.3f) policy_RL=(%.3f,%.3f,%.3f) policy_RR=(%.3f,%.3f,%.3f) "
        "ctrl_FR=(%.3f,%.3f,%.3f) ctrl_FL=(%.3f,%.3f,%.3f) ctrl_RR=(%.3f,%.3f,%.3f) ctrl_RL=(%.3f,%.3f,%.3f) "
        "diff_action_thigh=(%.3f,%.3f) diff_action_calf=(%.3f,%.3f) "
        "diff_qcmd_thigh=(%.3f,%.3f) diff_qcmd_calf=(%.3f,%.3f) "
        "diff_qstate_thigh=(%.3f,%.3f) diff_qstate_calf=(%.3f,%.3f)",
        rl_step_count_, robot_wx, robot_wy, robot_yaw, body_y, heading_cmd, in_recovery ? 1 : 0,
        value(policy_actions, 0), value(policy_actions, 1), value(policy_actions, 2),
        value(policy_actions, 3), value(policy_actions, 4), value(policy_actions, 5),
        value(policy_actions, 6), value(policy_actions, 7), value(policy_actions, 8),
        value(policy_actions, 9), value(policy_actions, 10), value(policy_actions, 11),
        value(ctrl_actions, 0), value(ctrl_actions, 1), value(ctrl_actions, 2),
        value(ctrl_actions, 3), value(ctrl_actions, 4), value(ctrl_actions, 5),
        value(ctrl_actions, 6), value(ctrl_actions, 7), value(ctrl_actions, 8),
        value(ctrl_actions, 9), value(ctrl_actions, 10), value(ctrl_actions, 11),
        fl_fr_thigh_action, rl_rr_thigh_action, fl_fr_calf_action, rl_rr_calf_action,
        fl_fr_thigh_q, rl_rr_thigh_q, fl_fr_calf_q, rl_rr_calf_q,
        fl_fr_thigh_state, rl_rr_thigh_state, fl_fr_calf_state, rl_rr_calf_state);
}

void StateRL::exit()
{
    running_ = false;
    RCLCPP_INFO(rclcpp::get_logger("StateRL"), "[VERIFY-EXIT] RL steps executed: %d", rl_step_count_);
    rl_step_count_ = 0;

    // Zero PD gains so the robot goes limp when exiting to PASSIVE
    for (int i = 0; i < params_.num_of_dofs; ++i)
    {
        robot_command_.motor_command.kp[i] = 0;
        robot_command_.motor_command.kd[i] = 0;
        robot_command_.motor_command.tau[i] = 0;
    }
}

void StateRL::computeRecoveryTwist()
{
    // === Paper method: gradient descent twist optimization (ROS1 lines 498-525) ===
    // Minimize: loss = lam * max(ra + 2*eps, 0) + 0.02 * ||twist*tau - cmd||²
    // 3 iterations, lr=0.5, clipped gradient, clipped result
    using torch::indexing::Slice;

    torch::autograd::GradMode::set_enabled(true);

    // Init twist from current state (ROS1 L498-500: vx,vy from linvel, wz from obs[6]=ang_vel_z)
    torch::Tensor twist = torch::tensor(
        {{obs_.lin_vel[0][0].item<double>(),   // vx
          obs_.lin_vel[0][1].item<double>(),   // vy
          obs_.ang_vel[0][2].item<double>()}}, // wz
        torch::requires_grad(true));

    const double tau = params_.twist_tau;   // 0.05
    const double lam = params_.twist_lam;   // 10.0
    const double lr  = params_.twist_lr;    // 0.5
    const double eps = params_.twist_eps;   // 0.05

    // Bounds (ROS1: twist_min = -twist_max)
    const double vx_m = params_.twist_vx_max, vy_m = params_.twist_vy_max, wz_m = params_.twist_wz_max;

    // Target cmd for pos_dev penalty (ROS1: obs[10:12] = body-frame goal position)
    double cmd_x = obs_.commands[0][0].item<double>();
    double cmd_y = obs_.commands[0][1].item<double>();

    double ra_vals[3] = {};
    for (int iter = 0; iter < 3; iter++)
    {
        // RA observation with current twist (ROS1 L508-514):
        // [vx,vy, lin_vel_z, ang_vel_x,ang_vel_y, wz, cmd_x,cmd_y, ray2d(11)] = 19
        torch::Tensor ra_obs = torch::cat({
            twist.index({Slice(), Slice(0, 2)}),                    // twist vx, vy
            obs_.lin_vel.index({Slice(), Slice(2, 3)}),             // lin_vel_z
            obs_.ang_vel.index({Slice(), Slice(0, 2)}),             // ang_vel x, y
            twist.index({Slice(), Slice(2, 3)}),                    // twist wz
            obs_.commands.index({Slice(), Slice(0, 2)}),            // cmd x, y
            obs_.ray2d                                                // 11 rays
        }, 1);

        auto ra_val = ra_model_.forward({ra_obs}).toTensor();
        ra_vals[iter] = ra_val.item<double>();

        // Position integral (ROS1: get_pos_integral = twist * tau, L517)
        auto pos_x = twist.index({0, 0}) * tau;
        auto pos_y = twist.index({0, 1}) * tau;

        // Loss (ROS1 L518): lam * clip(ra+2*eps, min=0) + 0.02 * pos_dev²
        auto ra_penalty = lam * torch::clamp(ra_val + 2.0 * eps, 0.0);
        auto pos_penalty = 0.02 * ((pos_x - cmd_x).pow(2) + (pos_y - cmd_y).pow(2));
        auto loss = (ra_penalty + pos_penalty).sum();
        loss.backward();

        // Clip gradient then update (ROS1 L522-524)
        auto grad = torch::clamp(twist.grad(), -1.0, 1.0);
        twist.data().sub_(lr * grad);
        // Clip to bounds
        twist.data().index_put_({0, 0}, torch::clamp(twist.data().index({0, 0}), -vx_m, vx_m));
        twist.data().index_put_({0, 1}, torch::clamp(twist.data().index({0, 1}), -vy_m, vy_m));
        twist.data().index_put_({0, 2}, torch::clamp(twist.data().index({0, 2}), -wz_m, wz_m));
        twist.grad().zero_();
    }

    // Detach (ROS1 L531)
    twist = twist.detach();

    ctrl_component_.recovery_twist_vx = twist[0][0].item<double>();
    ctrl_component_.recovery_twist_vy = twist[0][1].item<double>();
    ctrl_component_.recovery_twist_wz = twist[0][2].item<double>();

    RCLCPP_INFO(rclcpp::get_logger("StateRL"),
        "[TWIST-GD] 梯度下降恢复速度优化 | 初始=[%.2f,%.2f,%.2f] 最终=[%.2f,%.2f,%.2f] 目标=(%.2f,%.2f) "
        "RA迭代=[%.4f,%.4f,%.4f] 中心射线=%.2f",
        obs_.lin_vel[0][0].item<double>(), obs_.lin_vel[0][1].item<double>(),
        obs_.ang_vel[0][2].item<double>(),
        ctrl_component_.recovery_twist_vx, ctrl_component_.recovery_twist_vy,
        ctrl_component_.recovery_twist_wz, cmd_x, cmd_y,
        ra_vals[0], ra_vals[1], ra_vals[2], obs_.ray2d[0][5].item<double>());

    torch::autograd::GradMode::set_enabled(false);
}

torch::Tensor StateRL::computeRecoveryObservation(const torch::Tensor& twist)
{
    // === ROS1 line 532: obs_rec = [obs[0:10], twist, obs[14:50]] ===
    // contact(4) + ang_vel(3) + gravity_vec(3) + twist(3) + dof_pos(12) + dof_vel(12) + actions(12) = 49
    // All dof data remapped to FL-first policy order

    auto gravity_body = quatRotateInverse(obs_.base_quat,
        torch::tensor({{0.0, 0.0, -1.0}}), params_.framework);

    auto result = torch::cat({
        ctrlToPolicyContactOrder(obs_.contact),                                              // 0:4  contact FL-first
        obs_.ang_vel * params_.ang_vel_scale,                                                // 4:7  ang_vel
        gravity_body,                                                                        // 7:10 gravity_vec body frame
        twist,                                                                               // 10:13 commands = twist [vx,vy,wz]
        (ctrlToPolicyDofOrder(obs_.dof_pos) - ctrlToPolicyDofOrder(params_.default_dof_pos)) * params_.dof_pos_scale,  // 13:25
        ctrlToPolicyDofOrder(obs_.dof_vel) * params_.dof_vel_scale,                          // 25:37
        ctrlToPolicyDofOrder(obs_.actions)                                                   // 37:49
    }, 1);

    return clamp(result, -params_.clip_obs, params_.clip_obs);
}

bool StateRL::checkBodySafety() const
{
    // Compute roll/pitch from IMU quaternion (isaacgym order: x,y,z,w)
    double qw = robot_state_.imu.quaternion[3];
    double qx = robot_state_.imu.quaternion[0];
    double qy = robot_state_.imu.quaternion[1];
    double qz = robot_state_.imu.quaternion[2];

    double roll  = std::atan2(2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx * qx + qy * qy));
    double pitch = std::asin(std::clamp(2.0 * (qw * qy - qz * qx), -1.0, 1.0));

    double limit_rad = body_tilt_limit_deg_ * M_PI / 180.0;
    if (std::abs(roll) > limit_rad || std::abs(pitch) > limit_rad)
    {
        RCLCPP_ERROR(rclcpp::get_logger("StateRL"),
            "[SAFETY] Body tilt exceeded limit: roll=%.1f° pitch=%.1f° (limit=%.1f°) → PASSIVE",
            roll * 180.0 / M_PI, pitch * 180.0 / M_PI, body_tilt_limit_deg_);
        return false;
    }
    return true;
}

bool StateRL::checkTorqueSafety() const
{
    // Matches original ABS safe.PowerProtect(cmd, low_state, 8).
    // Monitors PD-computed joint torque against YAML-configured torque_limits.
    if (!torque_monitor_enabled_) return true;

    for (int i = 0; i < params_.num_of_dofs; ++i)
    {
        double torque = output_torques[0][i].item<double>();
        double limit  = params_.torque_limits[0][i].item<double>() * torque_limit_ratio_;
        if (std::abs(torque) > limit)
        {
            RCLCPP_ERROR(rclcpp::get_logger("StateRL"),
                "[SAFETY] Joint torque exceeded limit: joint=%d torque=%.1f Nm limit=%.1f Nm → PASSIVE",
                i, torque, limit);
            return false;
        }
    }
    return true;
}

FSMStateName StateRL::checkChange()
{
    // Emergency stop — matches original ABS wireless remote B-button (L441-444).
    // In simulation triggered by control_input command=1; on real robot by Go2 remote state.
    if (emergency_stop_enabled_)
    {
        if (ctrl_interfaces_.control_inputs_.command == 1 || emergency_stop_triggered_)
        {
            RCLCPP_ERROR(rclcpp::get_logger("StateRL"),
                "[EMERGENCY] Remote stop triggered → PASSIVE");
            return FSMStateName::PASSIVE;
        }
    }

    // Independent body attitude safety (works without estimator)
    if (!checkBodySafety())
    {
        return FSMStateName::PASSIVE;
    }

    // Joint torque safety (matches original safe.PowerProtect)
    if (!checkTorqueSafety())
    {
        return FSMStateName::PASSIVE;
    }

    // Estimator safety (roll/pitch guard, requires estimator enabled)
    if (enable_estimator_ and !estimator_->safety())
    {
        return FSMStateName::PASSIVE;
    }

    // RA-based recovery is now INLINE in runModel() (matches ROS1 lines 495-538).
    // No FSM switch — recovery action replaces agile action per-timestep.
    // keep key-4 for manual RL_REC mode (testing/debugging)

    switch (ctrl_interfaces_.control_inputs_.command)
    {
    case 1:
        return FSMStateName::PASSIVE;
    case 2:
        return FSMStateName::FIXEDDOWN;
    case 4:
        // Manual recovery: optimize twist and switch to RL_REC
        if (ra_loaded_) computeRecoveryTwist();
        return FSMStateName::RL_REC;
    default:
        return FSMStateName::RL;
    }
}

torch::Tensor StateRL::computeObservation()
{
    std::vector<torch::Tensor> obs_list;

    for (const std::string& observation : params_.observations)
    {
        if (observation == "lin_vel")
        {
            obs_list.push_back(obs_.lin_vel * params_.lin_vel_scale);
        }
        else if (observation == "ang_vel")
        {
            obs_list.push_back(obs_.ang_vel * params_.ang_vel_scale);
        }
        else if (observation == "gravity_vec")
        {
            obs_list.push_back(quatRotateInverse(obs_.base_quat, obs_.gravity_vec, params_.framework));
        }
        else if (observation == "commands")
        {
            obs_list.push_back(obs_.commands * params_.commands_scale);
        }
        else if (observation == "dof_pos")
        {
            obs_list.push_back(
                (ctrlToPolicyDofOrder(obs_.dof_pos) - ctrlToPolicyDofOrder(params_.default_dof_pos)) *
                params_.dof_pos_scale);
        }
        else if (observation == "dof_vel")
        {
            obs_list.push_back(ctrlToPolicyDofOrder(obs_.dof_vel) * params_.dof_vel_scale);
        }
        else if (observation == "actions")
        {
            obs_list.push_back(ctrlToPolicyDofOrder(obs_.actions));
        }
        else if (observation == "contact")
        {
            obs_list.push_back(ctrlToPolicyContactOrder(obs_.contact));
        }
        else if (observation == "timer")
        {
            // Match the ROS1 ABS deployment, which keeps timer fixed at 0.5.
            obs_list.push_back(torch::tensor({{0.5}}));
        }
        else if (observation == "ray2d")
        {
            obs_list.push_back(obs_.ray2d);
        }
    }

    const torch::Tensor obs = cat(obs_list, 1);
    torch::Tensor clamped_obs = clamp(obs, -params_.clip_obs, params_.clip_obs);
    return clamped_obs;
}

torch::Tensor StateRL::computeRAObservation()
{
    // RA 19-dim observation (matches training and ROS1):
    //   lin_vel(3) + ang_vel(3) + commands[0:2](2) + ray2d(11) = 19
    using torch::indexing::Slice;
    auto cmds = obs_.commands.index({Slice(), Slice(0, 2)});
    return torch::cat({obs_.lin_vel, obs_.ang_vel, cmds, obs_.ray2d}, 1);
}

void StateRL::runRAModel()
{
    if (!ra_loaded_)
        return;

    torch::autograd::GradMode::set_enabled(false);
    torch::Tensor ra_obs = computeRAObservation();
    auto output = ra_model_.forward({ra_obs}).toTensor();
    ra_value_ = output.item<double>();

    static bool diag = false;
    if (!diag) {
        RCLCPP_INFO(rclcpp::get_logger("StateRL"),
            "[RA] ra=%.4f ray2d=[%.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f]",
            ra_value_,
            ra_obs[0][8].item<double>(), ra_obs[0][9].item<double>(), ra_obs[0][10].item<double>(),
            ra_obs[0][11].item<double>(), ra_obs[0][12].item<double>(), ra_obs[0][13].item<double>(),
            ra_obs[0][14].item<double>(), ra_obs[0][15].item<double>(), ra_obs[0][16].item<double>(),
            ra_obs[0][17].item<double>(), ra_obs[0][18].item<double>());
        diag = true;
    }
}

void StateRL::loadYaml(const std::string& config_path)
{
    YAML::Node config;
    try
    {
        config = YAML::LoadFile(config_path + "/config.yaml");
    }
    catch ([[maybe_unused]] YAML::BadFile& e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("StateRL"), "The file '%s' does not exist", config_path.c_str());
        return;
    }

    params_.model_name = config["model_name"].as<std::string>();

    params_.model_name = config["model_name"].as<std::string>();
    params_.framework = config["framework"].as<std::string>();
    if (config["observations_history"].IsNull())
    {
        params_.observations_history = {};
    }
    else
    {
        params_.observations_history = ReadVectorFromYaml<int>(config["observations_history"]);
    }
    params_.decimation = config["decimation"].as<int>();
    params_.num_observations = config["num_observations"].as<int>();
    params_.observations = ReadVectorFromYaml<std::string>(config["observations"]);
    params_.clip_obs = config["clip_obs"].as<double>();
    if (config["clip_actions_lower"].IsNull() && config["clip_actions_upper"].IsNull())
    {
        params_.clip_actions_upper = torch::tensor({}).view({1, -1});
        params_.clip_actions_lower = torch::tensor({}).view({1, -1});
    }
    else
    {
        params_.clip_actions_upper = torch::tensor(
            ReadVectorFromYaml<double>(config["clip_actions_upper"])).view({1, -1});
        params_.clip_actions_lower = torch::tensor(
            ReadVectorFromYaml<double>(config["clip_actions_lower"])).view({1, -1});
    }
    params_.action_scale = config["action_scale"].as<double>();
    params_.hip_scale_reduction = config["hip_scale_reduction"].as<double>();
    params_.hip_scale_reduction_indices = ReadVectorFromYaml<int>(config["hip_scale_reduction_indices"]);
    params_.num_of_dofs = config["num_of_dofs"].as<int>();
    params_.lin_vel_scale = config["lin_vel_scale"].as<double>();
    params_.ang_vel_scale = config["ang_vel_scale"].as<double>();
    params_.dof_pos_scale = config["dof_pos_scale"].as<double>();
    params_.dof_vel_scale = config["dof_vel_scale"].as<double>();
    // params_.commands_scale = torch::tensor(ReadVectorFromYaml<double>(config["commands_scale"])).view({1, -1});
    params_.commands_scale = torch::tensor({params_.lin_vel_scale, params_.lin_vel_scale, params_.ang_vel_scale});
    params_.rl_kp = torch::tensor(ReadVectorFromYaml<double>(config["rl_kp"])).view({
        1, -1
    });
    params_.rl_kd = torch::tensor(ReadVectorFromYaml<double>(config["rl_kd"])).view({
        1, -1
    });
    params_.torque_limits = torch::tensor(ReadVectorFromYaml<double>(config["torque_limits"])).view({1, -1});

    params_.default_dof_pos = torch::from_blob(init_pos_, {12}, torch::kDouble).clone().to(torch::kFloat).unsqueeze(0);

    if (config["policy_joint_order"])
    {
        params_.policy_joint_order = config["policy_joint_order"].as<std::string>();
    }

    // ABS-specific parameters
    if (config["abs"])
    {
        auto abs_node = config["abs"];
        params_.abs_max_episode_length_s = abs_node["max_episode_length_s"].as<double>();
        params_.abs_contact_threshold = abs_node["contact_threshold"].as<double>();
        params_.abs_ray2d_count = abs_node["ray2d_count"].as<int>();
        params_.abs_ray2d_max_range = abs_node["ray2d_max_range"].as<double>();
        if (abs_node["ra_model_name"])
            params_.ra_model_name = abs_node["ra_model_name"].as<std::string>();
        if (abs_node["ra_threshold"])
            params_.ra_threshold = abs_node["ra_threshold"].as<double>();
        // Recovery twist optimization params (with defaults)
        if (abs_node["twist_lam"]) params_.twist_lam = abs_node["twist_lam"].as<double>();
        if (abs_node["twist_lr"]) params_.twist_lr = abs_node["twist_lr"].as<double>();
        if (abs_node["twist_tau"]) params_.twist_tau = abs_node["twist_tau"].as<double>();
        if (abs_node["twist_eps"]) params_.twist_eps = abs_node["twist_eps"].as<double>();
        if (abs_node["twist_vx_min"]) params_.twist_vx_min = abs_node["twist_vx_min"].as<double>();
        if (abs_node["twist_vx_max"]) params_.twist_vx_max = abs_node["twist_vx_max"].as<double>();
        if (abs_node["twist_vy_min"]) params_.twist_vy_min = abs_node["twist_vy_min"].as<double>();
        if (abs_node["twist_vy_max"]) params_.twist_vy_max = abs_node["twist_vy_max"].as<double>();
        if (abs_node["twist_wz_min"]) params_.twist_wz_min = abs_node["twist_wz_min"].as<double>();
        if (abs_node["twist_wz_max"]) params_.twist_wz_max = abs_node["twist_wz_max"].as<double>();
        if (abs_node["recovery_steps"]) params_.recovery_steps = abs_node["recovery_steps"].as<int>();
        if (abs_node["soft_start_steps"]) soft_start_steps_ = abs_node["soft_start_steps"].as<int>();
        if (abs_node["recovery_hold_steps"]) recovery_hold_steps_ = abs_node["recovery_hold_steps"].as<int>();
        if (abs_node["goal_x"]) goal_x_ = abs_node["goal_x"].as<double>();
        if (abs_node["goal_y"]) goal_y_ = abs_node["goal_y"].as<double>();
        if (abs_node["resample_goal_on_arrival"])
            resample_goal_on_arrival_ = abs_node["resample_goal_on_arrival"].as<bool>();
        if (abs_node["eval_telemetry_enabled"])
            params_.eval_telemetry_enabled = abs_node["eval_telemetry_enabled"].as<bool>();
        if (abs_node["eval_telemetry_interval_steps"])
            params_.eval_telemetry_interval_steps = abs_node["eval_telemetry_interval_steps"].as<int>();
        if (abs_node["symmetry_debug_enabled"])
            params_.symmetry_debug_enabled = abs_node["symmetry_debug_enabled"].as<bool>();
        // Safety thresholds
        if (abs_node["body_tilt_limit_deg"])  body_tilt_limit_deg_  = abs_node["body_tilt_limit_deg"].as<double>();
        if (abs_node["action_output_clip"])   action_output_clip_   = abs_node["action_output_clip"].as<double>();
        // Torque monitoring (matches original safe.PowerProtect)
        if (abs_node["torque_monitor_enabled"]) torque_monitor_enabled_ = abs_node["torque_monitor_enabled"].as<bool>();
        if (abs_node["torque_limit_ratio"])     torque_limit_ratio_     = abs_node["torque_limit_ratio"].as<double>();
        // Emergency stop (matches original B-button)
        if (abs_node["emergency_stop_enabled"]) emergency_stop_enabled_ = abs_node["emergency_stop_enabled"].as<bool>();
    }
}

torch::Tensor StateRL::quatRotateInverse(const torch::Tensor& q, const torch::Tensor& v, const std::string& framework)
{
    torch::Tensor q_w;
    torch::Tensor q_vec;
    if (framework == "isaacsim")
    {
        q_w = q.index({torch::indexing::Slice(), 0});
        q_vec = q.index({torch::indexing::Slice(), torch::indexing::Slice(1, 4)});
    }
    else if (framework == "isaacgym")
    {
        q_w = q.index({torch::indexing::Slice(), 3});
        q_vec = q.index({torch::indexing::Slice(), torch::indexing::Slice(0, 3)});
    }
    const c10::IntArrayRef shape = q.sizes();

    const torch::Tensor a = v * (2.0 * torch::pow(q_w, 2) - 1.0).unsqueeze(-1);
    const torch::Tensor b = cross(q_vec, v, -1) * q_w.unsqueeze(-1) * 2.0;
    const torch::Tensor c = q_vec * bmm(q_vec.view({shape[0], 1, 3}), v.view({shape[0], 3, 1})).squeeze(-1) * 2.0;
    return a - b + c;
}

torch::Tensor StateRL::forward()
{
    torch::autograd::GradMode::set_enabled(false);
    torch::Tensor clamped_obs = computeObservation();
    torch::Tensor actions;

    if (!params_.observations_history.empty())
    {
        history_obs_buf_->insert(clamped_obs);
        history_obs_ = history_obs_buf_->getObsVec(params_.observations_history);
        actions = model_.forward({history_obs_}).toTensor();
    }
    else
    {
        actions = model_.forward({clamped_obs}).toTensor();
    }

    if (params_.clip_actions_upper.numel() != 0 && params_.clip_actions_lower.numel() != 0)
    {
        return clamp(actions, params_.clip_actions_lower, params_.clip_actions_upper);
    }
    return actions;
}

void StateRL::getState()
{
    if (params_.framework == "isaacgym")
    {
        robot_state_.imu.quaternion[3] = ctrl_interfaces_.imu_state_interface_[0].get().get_value();
        robot_state_.imu.quaternion[0] = ctrl_interfaces_.imu_state_interface_[1].get().get_value();
        robot_state_.imu.quaternion[1] = ctrl_interfaces_.imu_state_interface_[2].get().get_value();
        robot_state_.imu.quaternion[2] = ctrl_interfaces_.imu_state_interface_[3].get().get_value();
    }
    else if (params_.framework == "isaacsim")
    {
        robot_state_.imu.quaternion[0] = ctrl_interfaces_.imu_state_interface_[0].get().get_value();
        robot_state_.imu.quaternion[1] = ctrl_interfaces_.imu_state_interface_[1].get().get_value();
        robot_state_.imu.quaternion[2] = ctrl_interfaces_.imu_state_interface_[2].get().get_value();
        robot_state_.imu.quaternion[3] = ctrl_interfaces_.imu_state_interface_[3].get().get_value();
    }

    robot_state_.imu.gyroscope[0] = ctrl_interfaces_.imu_state_interface_[4].get().get_value();
    robot_state_.imu.gyroscope[1] = ctrl_interfaces_.imu_state_interface_[5].get().get_value();
    robot_state_.imu.gyroscope[2] = ctrl_interfaces_.imu_state_interface_[6].get().get_value();

    robot_state_.imu.accelerometer[0] = ctrl_interfaces_.imu_state_interface_[7].get().get_value();
    robot_state_.imu.accelerometer[1] = ctrl_interfaces_.imu_state_interface_[8].get().get_value();
    robot_state_.imu.accelerometer[2] = ctrl_interfaces_.imu_state_interface_[9].get().get_value();

    for (int i = 0; i < 12; i++)
    {
        robot_state_.motor_state.q[i] = ctrl_interfaces_.joint_position_state_interface_[i].get().get_value();
        robot_state_.motor_state.dq[i] = ctrl_interfaces_.joint_velocity_state_interface_[i].get().get_value();
        robot_state_.motor_state.tauEst[i] = ctrl_interfaces_.joint_effort_state_interface_[i].get().get_value();
    }

    control_.x = ctrl_interfaces_.control_inputs_.ly;
    control_.y = -ctrl_interfaces_.control_inputs_.lx;
    control_.yaw = -ctrl_interfaces_.control_inputs_.rx;

    updated_ = true;
}

void StateRL::runModel()
{
    obs_.ang_vel = torch::tensor(robot_state_.imu.gyroscope).unsqueeze(0);
    obs_.base_quat = torch::tensor(robot_state_.imu.quaternion).unsqueeze(0);

    torch::Tensor lin_vel_world = torch::zeros({1, 3});
    if (ctrl_interfaces_.odom_state_interface_.size() >= 6)
    {
        lin_vel_world = torch::tensor({{
            ctrl_interfaces_.odom_state_interface_[3].get().get_value(),
            ctrl_interfaces_.odom_state_interface_[4].get().get_value(),
            ctrl_interfaces_.odom_state_interface_[5].get().get_value()
        }});
    }
    else if (enable_estimator_)
    {
        lin_vel_world = torch::from_blob(estimator_->getVelocity().data(), {3}, torch::kDouble).clone().
            to(torch::kFloat).unsqueeze(0);
    }
    obs_.lin_vel = quatRotateInverse(obs_.base_quat, lin_vel_world, params_.framework);

    // === Goal-directed navigation with world-frame position ===
    // Matches paper: GOAL_XYZ in world frame, odometry from ZED/mocap → body-frame target.
    // Uses MuJoCo ground-truth position (odometer sensor) instead of gyro drift integration.

    // Get robot world-frame position from odometer sensor (MuJoCo ground truth)
    double robot_wx = 0.0, robot_wy = 0.0;
    if (ctrl_interfaces_.odom_state_interface_.size() >= 2)
    {
        robot_wx = ctrl_interfaces_.odom_state_interface_[0].get().get_value();
        robot_wy = ctrl_interfaces_.odom_state_interface_[1].get().get_value();
    }

    // Extract yaw from IMU quaternion (absolute world-frame heading)
    // Framework "isaacgym": quaternion order = [x, y, z, w]
    double qx = robot_state_.imu.quaternion[0];
    double qy = robot_state_.imu.quaternion[1];
    double qz = robot_state_.imu.quaternion[2];
    double qw = robot_state_.imu.quaternion[3];
    double robot_yaw = std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));

    // Joystick offsets for manual goal adjustment (lx=lat, ly=long)
    const double joystick_x = std::clamp(control_.x, -1.0, 1.0) * 2.0;  // ±2m fwd/bwd trim
    const double joystick_y = std::clamp(control_.y, -1.0, 1.0) * 2.0;  // ±2m lateral trim
    const double joystick_yaw = std::clamp(control_.yaw, -1.0, 1.0);

    // World-frame goal = config goal + joystick trim
    double goal_wx = goal_x_ + joystick_x;
    double goal_wy = goal_y_ + joystick_y;

    // Vector from robot to goal in world frame
    double diff_x = goal_wx - robot_wx;
    double diff_y = goal_wy - robot_wy;

    // Rotate to body frame (ROS1 transform_global_xy_to_robot_xy)
    double cos_yaw = std::cos(robot_yaw);
    double sin_yaw = std::sin(robot_yaw);
    double body_x =  diff_x * cos_yaw + diff_y * sin_yaw;
    double body_y = -diff_x * sin_yaw + diff_y * cos_yaw;

    // Distance scaling (ROS1 lines 171-174: min(1, 5/dist+0.01))
    double dist_to_goal = std::sqrt(diff_x * diff_x + diff_y * diff_y);
    double scale = std::min(1.0, 5.0 / (dist_to_goal + 0.01));
    body_x *= scale;
    body_y *= scale;

    // Heading command = direction from robot to goal
    double heading_cmd = std::atan2(body_y, body_x) + joystick_yaw * 0.3;

    // When within tight threshold (training sigma_tight=0.5m), signal "stand still"
    const double arrival_threshold = 0.5;
    static int arrived_counter = 0;
    if (dist_to_goal < arrival_threshold)
    {
        body_x = 0.0;
        body_y = 0.0;
        heading_cmd = 0.0;

        // After standing at goal for ~1.6s, optionally resample goal.
        // Default is false for reproducible one-goal simulation runs.
        if (resample_goal_on_arrival_)
        {
            const int RESAMPLE_DELAY = 200;
            arrived_counter++;
            if (arrived_counter >= RESAMPLE_DELAY)
            {
                arrived_counter = 0;
                // Sample new goal in robot's body frame, then convert to world
                // Training ranges: forward [1.5, 7.5]m, lateral [-2.0, 2.0]m
                double fwd = 1.5 + static_cast<double>(rand() % 6001) / 1000.0;   // [1.5, 7.5]
                double lat = -2.0 + static_cast<double>(rand() % 4001) / 1000.0;  // [-2.0, 2.0]
                // Convert body-frame target to world frame
                goal_x_ = robot_wx + fwd * cos_yaw - lat * sin_yaw;
                goal_y_ = robot_wy + fwd * sin_yaw + lat * cos_yaw;
                RCLCPP_INFO(rclcpp::get_logger("StateRL"),
                    "[GOAL-RESAMPLE] new world goal=(%.2f,%.2f) body_offset=(%.2f,%.2f)",
                    goal_x_, goal_y_, fwd, lat);
            }
        }
    }
    else
    {
        arrived_counter = 0;  // reset if not at goal
    }

    // Diagnostic log every 100 RL steps
    static int goal_log_counter = 0;
    if (goal_log_counter++ % 100 == 0)
    {
        RCLCPP_INFO(rclcpp::get_logger("StateRL"),
            "[GOAL] 位置=(%.2f,%.2f) 偏航=%.2f 目标=(%.2f,%.2f) 距离=%.2f 机体系=(%.2f,%.2f) 航向=%.2f%s",
            robot_wx, robot_wy, robot_yaw, goal_wx, goal_wy, dist_to_goal, body_x, body_y, heading_cmd,
            (dist_to_goal < arrival_threshold) ? " [已到达]" : "");
    }

    const bool arrived = dist_to_goal < arrival_threshold;

    obs_.commands = torch::tensor({{body_x, body_y, heading_cmd}});
    obs_.base_quat = torch::tensor(robot_state_.imu.quaternion).unsqueeze(0);
    obs_.dof_pos = torch::tensor(robot_state_.motor_state.q).narrow(0, 0, params_.num_of_dofs).unsqueeze(0);
    obs_.dof_vel = torch::tensor(robot_state_.motor_state.dq).narrow(0, 0, params_.num_of_dofs).unsqueeze(0);

    // Update episode timer (RL step = decimation / frequency seconds)
    episode_timer_ += static_cast<double>(params_.decimation) / ctrl_interfaces_.frequency_;
    if (episode_timer_ > params_.abs_max_episode_length_s)
        episode_timer_ = 0.0;

    // Update contact from foot forces in controller order (FR, FL, RR, RL).
    // computeObservation() remaps it to ROS1 policy order (FL, FR, RL, RR).
    {
        torch::Tensor contact = torch::zeros({1, 4});
        int foot_count = static_cast<int>(ctrl_interfaces_.foot_force_state_interface_.size());
        for (int i = 0; i < std::min(4, foot_count); i++)
        {
            double force = ctrl_interfaces_.foot_force_state_interface_[i].get().get_value();
            contact[0][i] = 2.0 * (force > params_.abs_contact_threshold ? 1.0 : 0.0) - 1.0;
        }
        obs_.contact = contact;
    }

    // Update ray2d from shared memory (if available), otherwise keep constant
    if (ray2d_shm_ptr_ != nullptr)
    {
        obs_.ray2d = torch::from_blob(ray2d_shm_ptr_, {1, 11}, torch::kFloat32).clone();
    }

    // RA inference FIRST (ROS1 lines 475-488: evaluate ra_value before action)
    runRAModel();

    // === ROS1 lines 495-538: RA-based recovery (inline, per-timestep) ===
    // Frequency adaptation: ROS1 inference at 12.5Hz (80ms/step), we run at 125Hz (8ms/step).
    // To match ROS1 effective recovery duration (~250ms), we enforce a minimum hold:
    //  - On ENTER: compute twist via GD, hold for rec_hold_steps RL steps
    //  - During hold: reuse cached twist, keep recovery policy active
    //  - After hold: allow exit if ra < exit_threshold
    // ROS1: 80ms * 3steps ≈ 240ms.  Ours: 8ms * 30steps ≈ 240ms.
    const int REC_HOLD_STEPS = recovery_hold_steps_;
    torch::Tensor policy_actions;
    torch::Tensor clamped_actions;
    double ra_entry_thr = params_.ra_threshold;         // -0.05 = ROS1 default
    double ra_exit_thr = params_.ra_threshold - 0.03;   // -0.08 = hysteresis margin

    // Cache last optimized twist (avoids recomputing GD every 8ms step)
    static double cached_vx = 0.0, cached_vy = 0.0, cached_wz = 0.0;
    static int rec_hold_left = 0;
    static bool in_recovery = false;

    if (ra_loaded_ && rec_loaded_)
    {
        if (!in_recovery && ra_value_ > ra_entry_thr)
        {
            // ENTER recovery (ROS1 L495-497)
            in_recovery = true;
            rec_hold_left = REC_HOLD_STEPS;
            computeRecoveryTwist();  // GD optimization (ROS1 L498-525)
            cached_vx = ctrl_component_.recovery_twist_vx;
            cached_vy = ctrl_component_.recovery_twist_vy;
            cached_wz = ctrl_component_.recovery_twist_wz;
            RCLCPP_WARN(rclcpp::get_logger("StateRL"),
                "[RA-REC] 进入恢复 | 风险值 ra=%.4f > 进入阈值=%.4f 恢复速度=[%.2f,%.2f,%.2f] 保持步数=%d",
                ra_value_, ra_entry_thr, cached_vx, cached_vy, cached_wz, REC_HOLD_STEPS);
        }
        else if (in_recovery)
        {
            rec_hold_left--;
            if (rec_hold_left <= 0 && ra_value_ < ra_exit_thr)
            {
                // EXIT recovery — hold expired and RA confirmed safe
                in_recovery = false;
                RCLCPP_INFO(rclcpp::get_logger("StateRL"),
                    "[RA-REC] 退出恢复 | 风险值 ra=%.4f < 退出阈值=%.4f, 回到敏捷策略",
                    ra_value_, ra_exit_thr);
            }
        }
    }

    if (in_recovery)
    {
        // Use cached twist (reuse across multiple RL steps to match ROS1 80ms dwell)
        ctrl_component_.recovery_twist_vx = cached_vx;
        ctrl_component_.recovery_twist_vy = cached_vy;
        ctrl_component_.recovery_twist_wz = cached_wz;

        torch::Tensor twist = torch::tensor({{cached_vx, cached_vy, cached_wz}});
        torch::Tensor rec_obs = computeRecoveryObservation(twist);  // ROS1 line 532
        policy_actions = rec_model_.forward({rec_obs}).toTensor();
        clamped_actions = policyToCtrlDofOrder(policy_actions);  // ROS1 line 536

        // NaN guard (ROS1 lines 461-466)
        if (clamped_actions.isnan().any().item<int>())
        {
            RCLCPP_ERROR(rclcpp::get_logger("StateRL"),
                "[REC-NAN] NaN in recovery action! Falling back to agile.");
            policy_actions = forward();
            clamped_actions = policyToCtrlDofOrder(policy_actions);
        }
        else
        {
            static int rec_diag_count = 0;
            if (rec_diag_count++ % 10 == 0)
            {
                RCLCPP_INFO(rclcpp::get_logger("StateRL"),
                    "[REC-DIAG] lin_vel=[%.2f,%.2f,%.2f] twist=[%.2f,%.2f,%.2f] hold=%d rec_action_range=[%.3f,%.3f]",
                    obs_.lin_vel[0][0].item<double>(), obs_.lin_vel[0][1].item<double>(),
                    obs_.lin_vel[0][2].item<double>(),
                    cached_vx, cached_vy, cached_wz, rec_hold_left,
                    policy_actions.min().item<double>(), policy_actions.max().item<double>());
            }
        }
    }
    else
    {
        policy_actions = forward();
        clamped_actions = policyToCtrlDofOrder(policy_actions);
    }

    for (const int i : params_.hip_scale_reduction_indices)
    {
        clamped_actions[0][i] *= params_.hip_scale_reduction;
    }

    // Safety: clamp action output to reasonable range before position calculation
    // ±4.0 with action_scale 0.25 → ±1 rad joint offset, covering all normal gait
    if (action_output_clip_ > 0.0)
    {
        clamped_actions = torch::clamp(clamped_actions, -action_output_clip_, action_output_clip_);
    }

    obs_.actions = clamped_actions;

    const torch::Tensor actions_scaled = clamped_actions * params_.action_scale;
    output_torques = params_.rl_kp * (actions_scaled + params_.default_dof_pos - obs_.dof_pos) - params_.rl_kd * obs_.dof_vel;

    output_dof_pos_ = actions_scaled + params_.default_dof_pos;

    // Clip target positions to Go2 joint limits (ROS1 deployment does this for Go1)
    // Go2 limits from URDF: hip [-1.0472, 1.0472], thigh [-1.5708, 3.4907], calf [-2.7227, -0.83776]
    for (int i = 0; i < params_.num_of_dofs; ++i)
    {
        double q = output_dof_pos_[0][i].item<double>();
        if (i % 3 == 0)
            q = std::clamp(q, -1.0472, 1.0472);       // hip
        else if (i % 3 == 1)
            q = std::clamp(q, -1.5708, 3.4907);       // thigh
        else
            q = std::clamp(q, -2.7227, -0.83776);     // calf
        output_dof_pos_[0][i] = q;
    }

    for (int i = 0; i < params_.num_of_dofs; ++i)
    {
        robot_command_.motor_command.q[i] = output_dof_pos_[0][i].item<double>();
        robot_command_.motor_command.dq[i] = 0;
        robot_command_.motor_command.kp[i] = params_.rl_kp[0][i].item<double>();
        robot_command_.motor_command.kd[i] = params_.rl_kd[0][i].item<double>();
        robot_command_.motor_command.tau[i] = 0;
    }

    logEvalTelemetry(robot_wx, robot_wy, robot_yaw, goal_wx, goal_wy, dist_to_goal,
                     body_x, body_y, heading_cmd, arrived, in_recovery, rec_hold_left,
                     cached_vx, cached_vy, cached_wz);
    logSymmetryDebug(robot_wx, robot_wy, robot_yaw, body_y, heading_cmd, in_recovery,
                     policy_actions, clamped_actions);

    rl_step_count_++;
}

void StateRL::setCommand() const
{
    // Soft start: ramp Kp/Kd from 0 to target over first N RL steps
    if (soft_start_step_ < soft_start_steps_)
    {
        soft_start_step_++;
    }
    const double ratio = std::min(1.0, static_cast<double>(soft_start_step_) / soft_start_steps_);

    for (int i = 0; i < 12; i++)
    {
        ctrl_interfaces_.joint_position_command_interface_[i].get().
                                                                            set_value(
                                                                                robot_command_.motor_command.q[i]);
        ctrl_interfaces_.joint_velocity_command_interface_[i].get().set_value(
            robot_command_.motor_command.dq[i]);
        ctrl_interfaces_.joint_kp_command_interface_[i].get().set_value(
            robot_command_.motor_command.kp[i] * ratio);
        ctrl_interfaces_.joint_kd_command_interface_[i].get().set_value(
            robot_command_.motor_command.kd[i] * ratio);
        ctrl_interfaces_.joint_torque_command_interface_[i].get().
                                                                          set_value(
                                                                              robot_command_.motor_command.tau[i]);
    }
}
