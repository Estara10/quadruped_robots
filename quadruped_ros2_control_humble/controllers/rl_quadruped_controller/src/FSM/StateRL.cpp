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

void StateRL::exit()
{
    running_ = false;
    RCLCPP_INFO(rclcpp::get_logger("StateRL"), "[VERIFY-EXIT] RL steps executed: %d", rl_step_count_);
    rl_step_count_ = 0;
}

void StateRL::computeRecoveryTwist()
{
    // === Pure ray2d-driven recovery twist — no gradient descent ===
    // The RA model is used only for the TRIGGER (should we activate recovery?).
    // The recovery ACTION (what twist to use) is determined directly from ray2d data.
    //
    // Rationale: RA model gradient descent proved unreliable because:
    // 1. Training data has little vy/wz variation → model is flat in those dims
    // 2. pos_dev penalty (0.02*y_iter²) pulls vy→0, undoing direction bias
    // 3. TorchScript C++ autograd status is unverified
    //
    // Ray-order: 0=-45°(right-fwd), 5=0°, 10=+45°(left-fwd)
    // Body frame: +y=left, -y=right (confirmed from go2.xml hip positions)
    using torch::indexing::Slice;
    auto ray2d = obs_.ray2d;

    // Compute side averages
    double left_avg  = (ray2d[0][6].item<double>() + ray2d[0][7].item<double>()
                      + ray2d[0][8].item<double>() + ray2d[0][9].item<double>()
                      + ray2d[0][10].item<double>()) / 5.0;
    double right_avg = (ray2d[0][0].item<double>() + ray2d[0][1].item<double>()
                      + ray2d[0][2].item<double>() + ray2d[0][3].item<double>()
                      + ray2d[0][4].item<double>()) / 5.0;
    double center_ray = ray2d[0][5].item<double>();

    // Direction: lower log2 = closer obstacle → go away from that side
    const double asym_thr = 0.15;
    int vy_sign = 0;   // -1=obstacle left→go right, +1=obstacle right→go left
    if (left_avg < right_avg - asym_thr)       vy_sign = -1;
    else if (right_avg < left_avg - asym_thr)  vy_sign = +1;

    // Forward speed: reduce when obstacles are close ahead
    // center_ray ~2.58 = clear (log2(6m)), ~1.0 = obstacle at 2m, ~0 = obstacle at 1m
    double vx = std::max(0.3, center_ray * 0.6);  // 0.3 ~ 1.5 m/s depending on clearance
    vx = std::min(vx, params_.twist_vx_max);

    // Lateral: go away from obstacle side
    double vy = 0.0;
    if (vy_sign != 0) {
        vy = vy_sign * params_.twist_vy_max;  // ±0.3
    }

    // Yaw: turn away from obstacle, proportional to asymmetry
    double wz = 0.0;
    if (vy_sign != 0) {
        // More asymmetry → stronger turn
        double asym = std::abs(right_avg - left_avg);
        wz = vy_sign * std::min(3.0, asym * 2.0);  // up to ±3.0 rad/s
    }

    const char* direction = "STRAIGHT";
    if (vy_sign < 0)      direction = "RIGHT";
    else if (vy_sign > 0) direction = "LEFT";

    ctrl_component_.recovery_twist_vx = vx;
    ctrl_component_.recovery_twist_vy = vy;
    ctrl_component_.recovery_twist_wz = wz;

    static int call_count = 0;
    if (call_count++ % 5 == 0) {  // log every 5th call to avoid spam
        RCLCPP_INFO(rclcpp::get_logger("StateRL"),
            "[TWIST-OPT] dir=%s ray2d=[L%.2f C%.2f R%.2f] asym=%.2f twist=[%.2f,%.2f,%.2f]",
            direction, left_avg, center_ray, right_avg,
            std::abs(right_avg - left_avg), vx, vy, wz);
    }
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

FSMStateName StateRL::checkChange()
{
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
        if (abs_node["rl_cooldown_steps"]) params_.rl_cooldown_steps = abs_node["rl_cooldown_steps"].as<int>();
        if (abs_node["soft_start_steps"]) soft_start_steps_ = abs_node["soft_start_steps"].as<int>();
        if (abs_node["goal_x"]) goal_x_ = abs_node["goal_x"].as<double>();
        if (abs_node["goal_y"]) goal_y_ = abs_node["goal_y"].as<double>();
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
    if (enable_estimator_)
    {
        obs_.lin_vel = torch::from_blob(estimator_->getVelocity().data(), {3}, torch::kDouble).clone().
            to(torch::kFloat).unsqueeze(0);
    }
    obs_.ang_vel = torch::tensor(robot_state_.imu.gyroscope).unsqueeze(0);

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
    // by zeroing commands. This matches training: _reward_stand_still_pos activates
    // within 0.5m, _reward_velo_dir switches to constant 1.0, removing forward incentive.
    const double arrival_threshold = 0.5;  // matches training position_target_sigma_tight
    if (dist_to_goal < arrival_threshold)
    {
        body_x = 0.0;
        body_y = 0.0;
        heading_cmd = 0.0;
    }

    // Diagnostic log every 100 RL steps
    static int goal_log_counter = 0;
    if (goal_log_counter++ % 100 == 0)
    {
        RCLCPP_INFO(rclcpp::get_logger("StateRL"),
            "[GOAL] robot=(%.2f,%.2f) yaw=%.2f goal=(%.2f,%.2f) dist=%.2f body=(%.2f,%.2f) heading=%.2f%s",
            robot_wx, robot_wy, robot_yaw, goal_wx, goal_wy, dist_to_goal, body_x, body_y, heading_cmd,
            (dist_to_goal < arrival_threshold) ? " [ARRIVED]" : "");
    }

    obs_.commands = torch::tensor({{body_x, body_y, heading_cmd}});
    obs_.base_quat = torch::tensor(robot_state_.imu.quaternion).unsqueeze(0);
    obs_.dof_pos = torch::tensor(robot_state_.motor_state.q).narrow(0, 0, params_.num_of_dofs).unsqueeze(0);
    obs_.dof_vel = torch::tensor(robot_state_.motor_state.dq).narrow(0, 0, params_.num_of_dofs).unsqueeze(0);

    // Update episode timer (RL step = decimation / frequency seconds)
    episode_timer_ += static_cast<double>(params_.decimation) / ctrl_interfaces_.frequency_;
    if (episode_timer_ > params_.abs_max_episode_length_s)
        episode_timer_ = 0.0;

    // Update contact from foot forces
    // Training order: FR, FL, RR, RL (matches IsaacGym URDF body order)
    // DDS footForce: FR, FL, RR, RL (matches MuJoCo touch sensor order)
    // ros2_control.xacro: FR, FL, RR, RL
    // YAML foot_force_interfaces: FR, FL, RR, RL
    // All aligned — no remapping needed
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
    // Hysteresis: our RA values are systematically higher than ROS1 (estimator vs ZED
    // odometry, 50Hz vs 12.5Hz). Without hysteresis, RA oscillates around threshold
    // and recovery never exits cleanly. Entry is sensitive (catch danger early),
    // exit requires clear safety margin.
    torch::Tensor clamped_actions;
    static bool in_recovery = false;
    double ra_entry_thr = params_.ra_threshold;         // -0.05 = ROS1 default
    double ra_exit_thr = params_.ra_threshold - 0.03;   // -0.08 = need slight margin

    bool should_recover = ra_loaded_ && rec_loaded_
        && (in_recovery ? (ra_value_ > ra_exit_thr) : (ra_value_ > ra_entry_thr));

    if (should_recover)
    {
        if (!in_recovery)
        {
            in_recovery = true;
            RCLCPP_WARN(rclcpp::get_logger("StateRL"),
                "[RA-REC] ENTER recovery: ra=%.4f > entry=%.4f", ra_value_, ra_entry_thr);
        }

        computeRecoveryTwist();  // ROS1 lines 498-526: gradient descent
        torch::Tensor twist = torch::tensor(
            {{ctrl_component_.recovery_twist_vx,
              ctrl_component_.recovery_twist_vy,
              ctrl_component_.recovery_twist_wz}});
        torch::Tensor rec_obs = computeRecoveryObservation(twist);  // ROS1 line 532
        auto rec_action = rec_model_.forward({rec_obs}).toTensor();
        clamped_actions = policyToCtrlDofOrder(rec_action);  // ROS1 line 536

        // ROS1 lines 461-466: NaN guard
        if (clamped_actions.isnan().any().item<int>())
        {
            RCLCPP_ERROR(rclcpp::get_logger("StateRL"),
                "[REC-NAN] NaN in recovery action! Falling back to agile policy.");
            clamped_actions = policyToCtrlDofOrder(forward());
        }
        else
        {
            static int rec_diag_count = 0;
            if (rec_diag_count++ % 10 == 0)
            {
                RCLCPP_INFO(rclcpp::get_logger("StateRL"),
                    "[REC-DIAG] lin_vel=[%.2f,%.2f,%.2f] twist=[%.2f,%.2f,%.2f] rec_action_range=[%.3f,%.3f]",
                    obs_.lin_vel[0][0].item<double>(), obs_.lin_vel[0][1].item<double>(), obs_.lin_vel[0][2].item<double>(),
                    ctrl_component_.recovery_twist_vx, ctrl_component_.recovery_twist_vy, ctrl_component_.recovery_twist_wz,
                    rec_action.min().item<double>(), rec_action.max().item<double>());
            }
        }
    }
    else
    {
        if (in_recovery)
        {
            in_recovery = false;
            RCLCPP_INFO(rclcpp::get_logger("StateRL"),
                "[RA-REC] EXIT recovery: ra=%.4f < exit=%.4f, back to agile",
                ra_value_, ra_exit_thr);
        }
        clamped_actions = policyToCtrlDofOrder(forward());
    }

    for (const int i : params_.hip_scale_reduction_indices)
    {
        clamped_actions[0][i] *= params_.hip_scale_reduction;
    }

    obs_.actions = clamped_actions;

    // Diagnostic: check left-right symmetry every 50 RL steps
    static int sym_log_counter = 0;
    if (sym_log_counter++ % 50 == 0)
    {
        auto& a = clamped_actions;  // ctrl order: FR(0-2), FL(3-5), RR(6-8), RL(9-11)
        RCLCPP_INFO(rclcpp::get_logger("StateRL"),
            "[SYM] cmd=(%.3f,%.3f,%.3f) | "
            "FR_hip=%.4f FL_hip=%.4f | FR_thigh=%.4f FL_thigh=%.4f | "
            "RR_hip=%.4f RL_hip=%.4f",
            obs_.commands[0][0].item<double>(), obs_.commands[0][1].item<double>(),
            obs_.commands[0][2].item<double>(),
            a[0][0].item<double>(), a[0][3].item<double>(),
            a[0][1].item<double>(), a[0][4].item<double>(),
            a[0][6].item<double>(), a[0][9].item<double>());
    }

    const torch::Tensor actions_scaled = clamped_actions * params_.action_scale;
    // torch::Tensor output_torques = params_.rl_kp * (actions_scaled + params_.default_dof_pos - obs_.dof_pos) - params_.rl_kd * obs_.dof_vel;
    // output_torques = clamp(output_torques, -(params_.torque_limits), params_.torque_limits);

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
