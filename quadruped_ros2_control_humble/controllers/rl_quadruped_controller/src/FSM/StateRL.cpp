//
// Created by biao on 24-10-6.
//

#include "rl_quadruped_controller/FSM/StateRL.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/logging.hpp>
#include <yaml-cpp/yaml.h>

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
    RCLCPP_INFO(node_->get_logger(), "[VERIFY] default_dof_pos: %.4f %.4f %.4f (first 3 joints = FL hip/thigh/calf)",
                params_.default_dof_pos[0][0].item<double>(), params_.default_dof_pos[0][1].item<double>(),
                params_.default_dof_pos[0][2].item<double>());
    RCLCPP_INFO(node_->get_logger(), "[VERIFY] ABS: max_ep_len=%.1fs contact_thr=%.1fN ray2d_count=%d ray2d_max_range=%.1fm",
                params_.abs_max_episode_length_s, params_.abs_contact_threshold,
                params_.abs_ray2d_count, params_.abs_ray2d_max_range);
    RCLCPP_INFO(node_->get_logger(), "[VERIFY] decimation=%d frequency=%dHz dt=%.4fs",
                params_.decimation, ctrl_interfaces_.frequency_,
                static_cast<double>(params_.decimation) / ctrl_interfaces_.frequency_);
    // ===== End verification logs =====


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
    obs_.ray2d = torch::ones({1, params_.abs_ray2d_count}) * std::log2(params_.abs_ray2d_max_range);
    episode_timer_ = 0.0;
    rl_step_count_ = 0;

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

    // ===== Verification logs (remove after confirmed) =====
    RCLCPP_INFO(rclcpp::get_logger("StateRL"), "[VERIFY-ENTER] contact shape=%ldx%ld init_values=[%.0f %.0f %.0f %.0f]",
                obs_.contact.size(0), obs_.contact.size(1),
                obs_.contact[0][0].item<double>(), obs_.contact[0][1].item<double>(),
                obs_.contact[0][2].item<double>(), obs_.contact[0][3].item<double>());
    RCLCPP_INFO(rclcpp::get_logger("StateRL"), "[VERIFY-ENTER] ray2d values (first 3): %.4f %.4f %.4f (log2(%.1f)=%.4f)",
                obs_.ray2d[0][0].item<double>(), obs_.ray2d[0][1].item<double>(),
                obs_.ray2d[0][2].item<double>(), params_.abs_ray2d_max_range,
                std::log2(params_.abs_ray2d_max_range));
    RCLCPP_INFO(rclcpp::get_logger("StateRL"), "[VERIFY-ENTER] episode_timer=%.1f, running_=%d",
                episode_timer_, running_);
    // ===== End verification logs =====
}

void StateRL::run(const rclcpp::Time&/*time*/, const rclcpp::Duration&/*period*/)
{
    getState();
    if (!use_rl_thread_)
    {
        runModel();
    }
    setCommand();
}

void StateRL::exit()
{
    running_ = false;
    RCLCPP_INFO(rclcpp::get_logger("StateRL"), "[VERIFY-EXIT] RL steps executed: %d", rl_step_count_);
    rl_step_count_ = 0;
}

FSMStateName StateRL::checkChange()
{
    if (enable_estimator_ and !estimator_->safety())
    {
        return FSMStateName::PASSIVE;
    }
    switch (ctrl_interfaces_.control_inputs_.command)
    {
    case 1:
        return FSMStateName::PASSIVE;
    case 2:
        return FSMStateName::FIXEDDOWN;
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
            obs_list.push_back((obs_.dof_pos - params_.default_dof_pos) * params_.dof_pos_scale);
        }
        else if (observation == "dof_vel")
        {
            obs_list.push_back(obs_.dof_vel * params_.dof_vel_scale);
        }
        else if (observation == "actions")
        {
            obs_list.push_back(obs_.actions);
        }
        else if (observation == "contact")
        {
            obs_list.push_back(obs_.contact);
        }
        else if (observation == "timer")
        {
            double t = std::min(episode_timer_ / params_.abs_max_episode_length_s, 1.0);
            obs_list.push_back(torch::tensor({{t}}));
        }
        else if (observation == "ray2d")
        {
            obs_list.push_back(obs_.ray2d);
        }
    }

    const torch::Tensor obs = cat(obs_list, 1);

    static bool obs_dim_printed = false;
    if (!obs_dim_printed) {
        RCLCPP_INFO(rclcpp::get_logger("StateRL"), "[VERIFY-OBS] computed observation shape: %ldx%ld (expected 1x%d)",
                    obs.size(0), obs.size(1), params_.num_observations);
        obs_dim_printed = true;
    }

    torch::Tensor clamped_obs = clamp(obs, -params_.clip_obs, params_.clip_obs);
    return clamped_obs;
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

    // ABS-specific parameters
    if (config["abs"])
    {
        auto abs_node = config["abs"];
        params_.abs_max_episode_length_s = abs_node["max_episode_length_s"].as<double>();
        params_.abs_contact_threshold = abs_node["contact_threshold"].as<double>();
        params_.abs_ray2d_count = abs_node["ray2d_count"].as<int>();
        params_.abs_ray2d_max_range = abs_node["ray2d_max_range"].as<double>();
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

    static bool action_verified = false;
    if (!action_verified) {
        RCLCPP_INFO(rclcpp::get_logger("StateRL"), "[VERIFY-ACTION] model output shape: %ldx%ld (expected 1x%d)",
                    actions.size(0), actions.size(1), params_.num_of_dofs);
        RCLCPP_INFO(rclcpp::get_logger("StateRL"), "[VERIFY-ACTION] first 3 actions: %.4f %.4f %.4f",
                    actions[0][0].item<double>(), actions[0][1].item<double>(), actions[0][2].item<double>());
        action_verified = true;
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
    obs_.commands = torch::tensor({{control_.x, control_.y, control_.yaw}});
    obs_.base_quat = torch::tensor(robot_state_.imu.quaternion).unsqueeze(0);
    obs_.dof_pos = torch::tensor(robot_state_.motor_state.q).narrow(0, 0, params_.num_of_dofs).unsqueeze(0);
    obs_.dof_vel = torch::tensor(robot_state_.motor_state.dq).narrow(0, 0, params_.num_of_dofs).unsqueeze(0);

    // Update episode timer (RL step = decimation / frequency seconds)
    episode_timer_ += static_cast<double>(params_.decimation) / ctrl_interfaces_.frequency_;
    if (episode_timer_ > params_.abs_max_episode_length_s)
        episode_timer_ = 0.0;

    // Update contact from foot forces
    {
        torch::Tensor contact = torch::zeros({1, 4});
        int foot_count = static_cast<int>(ctrl_interfaces_.foot_force_state_interface_.size());
        for (int i = 0; i < std::min(4, foot_count); i++)
        {
            double force = ctrl_interfaces_.foot_force_state_interface_[i].get().get_value();
            contact[0][i] = 2.0 * (force > params_.abs_contact_threshold ? 1.0 : 0.0) - 1.0;
        }
        obs_.contact = contact;

        static bool contact_verified = false;
        if (!contact_verified) {
            RCLCPP_INFO(rclcpp::get_logger("StateRL"),
                "[VERIFY-CONTACT] foot_count=%d forces=[%.1f %.1f %.1f %.1f] contact=[%.0f %.0f %.0f %.0f]",
                foot_count,
                (foot_count > 0 ? ctrl_interfaces_.foot_force_state_interface_[0].get().get_value() : -1.0),
                (foot_count > 1 ? ctrl_interfaces_.foot_force_state_interface_[1].get().get_value() : -1.0),
                (foot_count > 2 ? ctrl_interfaces_.foot_force_state_interface_[2].get().get_value() : -1.0),
                (foot_count > 3 ? ctrl_interfaces_.foot_force_state_interface_[3].get().get_value() : -1.0),
                contact[0][0].item<double>(), contact[0][1].item<double>(),
                contact[0][2].item<double>(), contact[0][3].item<double>());
            contact_verified = true;
        }
    }

    const torch::Tensor clamped_actions = forward();

    for (const int i : params_.hip_scale_reduction_indices)
    {
        clamped_actions[0][i] *= params_.hip_scale_reduction;
    }

    obs_.actions = clamped_actions;

    const torch::Tensor actions_scaled = clamped_actions * params_.action_scale;
    // torch::Tensor output_torques = params_.rl_kp * (actions_scaled + params_.default_dof_pos - obs_.dof_pos) - params_.rl_kd * obs_.dof_vel;
    // output_torques = clamp(output_torques, -(params_.torque_limits), params_.torque_limits);

    output_dof_pos_ = actions_scaled + params_.default_dof_pos;

    for (int i = 0; i < params_.num_of_dofs; ++i)
    {
        robot_command_.motor_command.q[i] = output_dof_pos_[0][i].item<double>();
        robot_command_.motor_command.dq[i] = 0;
        robot_command_.motor_command.kp[i] = params_.rl_kp[0][i].item<double>();
        robot_command_.motor_command.kd[i] = params_.rl_kd[0][i].item<double>();
        robot_command_.motor_command.tau[i] = 0;
    }

    static bool cmd_verified = false;
    rl_step_count_++;
    if (!cmd_verified) {
        RCLCPP_INFO(rclcpp::get_logger("StateRL"),
            "[VERIFY-CMD] FL_hip: q=%.4f kp=%.2f kd=%.2f | FL_thigh: q=%.4f kp=%.2f kd=%.2f | FL_calf: q=%.4f kp=%.2f kd=%.2f",
            robot_command_.motor_command.q[0], robot_command_.motor_command.kp[0], robot_command_.motor_command.kd[0],
            robot_command_.motor_command.q[1], robot_command_.motor_command.kp[1], robot_command_.motor_command.kd[1],
            robot_command_.motor_command.q[2], robot_command_.motor_command.kp[2], robot_command_.motor_command.kd[2]);
        cmd_verified = true;
    }
}

void StateRL::setCommand() const
{
    for (int i = 0; i < 12; i++)
    {
        ctrl_interfaces_.joint_position_command_interface_[i].get().
                                                                            set_value(
                                                                                robot_command_.motor_command.q[i]);
        ctrl_interfaces_.joint_velocity_command_interface_[i].get().set_value(
            robot_command_.motor_command.dq[i]);
        ctrl_interfaces_.joint_kp_command_interface_[i].get().set_value(
            robot_command_.motor_command.kp[i]);
        ctrl_interfaces_.joint_kd_command_interface_[i].get().set_value(
            robot_command_.motor_command.kd[i]);
        ctrl_interfaces_.joint_torque_command_interface_[i].get().
                                                                          set_value(
                                                                              robot_command_.motor_command.tau[i]);
    }
}
