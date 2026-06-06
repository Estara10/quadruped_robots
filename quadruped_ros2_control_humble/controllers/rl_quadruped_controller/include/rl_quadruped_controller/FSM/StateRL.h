//
// Created by biao on 24-10-6.
//

#ifndef STATERL_H
#define STATERL_H

#include <common/ObservationBuffer.h>
#include <rl_quadruped_controller/control/CtrlComponent.h>
#include <torch/script.h>

#include "controller_common/FSM/FSMState.h"

struct CtrlComponent;

template <typename Functor>
void executeAndSleep(Functor f, const double frequency)
{
    using clock = std::chrono::high_resolution_clock;
    const auto start = clock::now();

    // Execute wrapped function
    f();

    // Compute desired duration rounded to clock decimation
    const std::chrono::duration<double> desiredDuration(1.0 / frequency);
    const auto dt = std::chrono::duration_cast<clock::duration>(desiredDuration);

    // Sleep
    const auto sleepTill = start + dt;
    std::this_thread::sleep_until(sleepTill);
}

inline void setThreadPriority(int priority, std::thread& thread)
{
    sched_param sched{};
    sched.sched_priority = priority;

    if (priority != 0)
    {
        if (pthread_setschedparam(thread.native_handle(), SCHED_FIFO, &sched) != 0)
        {
            std::cerr << "WARNING: Failed to set threads priority (one possible reason could be "
                "that the user and the group permissions are not set properly.)"
                << std::endl;
        }
    }
}


template <typename T>
struct RobotCommand
{
    struct MotorCommand
    {
        std::vector<T> q = std::vector<T>(32, 0.0);
        std::vector<T> dq = std::vector<T>(32, 0.0);
        std::vector<T> tau = std::vector<T>(32, 0.0);
        std::vector<T> kp = std::vector<T>(32, 0.0);
        std::vector<T> kd = std::vector<T>(32, 0.0);
    } motor_command;
};

template <typename T>
struct RobotState
{
    struct IMU
    {
        std::vector<T> quaternion = {1.0, 0.0, 0.0, 0.0}; // w, x, y, z
        std::vector<T> gyroscope = {0.0, 0.0, 0.0};
        std::vector<T> accelerometer = {0.0, 0.0, 0.0};
    } imu;

    struct MotorState
    {
        std::vector<T> q = std::vector<T>(32, 0.0);
        std::vector<T> dq = std::vector<T>(32, 0.0);
        std::vector<T> ddq = std::vector<T>(32, 0.0);
        std::vector<T> tauEst = std::vector<T>(32, 0.0);
        std::vector<T> cur = std::vector<T>(32, 0.0);
    } motor_state;
};

struct Control
{
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
};

struct ModelParams
{
    std::string model_name;
    std::string framework;
    int decimation;
    int num_observations;
    std::vector<std::string> observations;
    std::vector<int> observations_history;
    double damping;
    double stiffness;
    double action_scale;
    double hip_scale_reduction;
    std::vector<int> hip_scale_reduction_indices;
    int num_of_dofs;
    double lin_vel_scale;
    double ang_vel_scale;
    double dof_pos_scale;
    double dof_vel_scale;
    double clip_obs;
    torch::Tensor clip_actions_upper;
    torch::Tensor clip_actions_lower;
    torch::Tensor torque_limits;
    torch::Tensor rl_kd;
    torch::Tensor rl_kp;
    torch::Tensor commands_scale;
    torch::Tensor default_dof_pos;
    std::string policy_joint_order = "fr_first";
    // ABS-specific params
    double abs_max_episode_length_s = 9.0;
    double abs_contact_threshold = 1.0;
    int abs_ray2d_count = 11;
    double abs_ray2d_max_range = 6.0;
    std::string ra_model_name = "ra_value.pt";
    double ra_threshold = -0.05;
    // Recovery twist optimization params
    double twist_lam = 10.0;
    double twist_lr = 0.5;
    double twist_tau = 0.05;
    double twist_eps = 0.05;
    double twist_vx_min = -1.5, twist_vx_max = 1.5;
    double twist_vy_min = -0.3, twist_vy_max = 0.3;
    double twist_wz_min = -3.0, twist_wz_max = 3.0;
    int recovery_steps = 250;
    int rl_cooldown_steps = 100;
};

struct Observations
{
    torch::Tensor lin_vel;
    torch::Tensor ang_vel;
    torch::Tensor gravity_vec;
    torch::Tensor commands;
    torch::Tensor base_quat;
    torch::Tensor dof_pos;
    torch::Tensor dof_vel;
    torch::Tensor actions;
    torch::Tensor contact;       // [1, 4] foot contact [-1, 1]
    torch::Tensor ray2d;         // [1, 11] log2 ray distances
};

class StateRL final : public FSMState
{
public:
    explicit StateRL(CtrlInterfaces& ctrl_interfaces,
                     CtrlComponent& ctrl_component,
                     const std::vector<double>& target_pos);
    ~StateRL() override;

    void enter() override;

    void run(const rclcpp::Time& time,
             const rclcpp::Duration& period) override;

    void exit() override;

    FSMStateName checkChange() override;

private:
    torch::Tensor computeObservation();
    torch::Tensor computeRAObservation();
    void runRAModel();
    // ROS1: gradient descent twist optimization + recovery action (lines 498-538)
    void computeRecoveryTwist();
    torch::Tensor computeRecoveryObservation(const torch::Tensor& twist);  // 49-dim for recovery policy

    void loadYaml(const std::string& config_path);

    static torch::Tensor quatRotateInverse(const torch::Tensor& q, const torch::Tensor& v,
                                           const std::string& framework);

    /**
    * @brief Forward the RL model to get the action
    */
    torch::Tensor forward();

    void getState();

    void runModel();

    void setCommand() const;

    std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
    CtrlComponent& ctrl_component_;
    std::string robot_pkg_ = "go2_description";
    std::string model_folder_ = "legged_gym";

    bool enable_estimator_;
    std::shared_ptr<Estimator>& estimator_;

    // Parameters
    ModelParams params_;
    Observations obs_;
    Control control_;
    double init_pos_[12] = {};

    RobotState<double> robot_state_;
    RobotCommand<double> robot_command_;

    // history buffer
    std::shared_ptr<ObservationBuffer> history_obs_buf_;
    torch::Tensor history_obs_;

    // rl module
    torch::jit::script::Module model_;
    torch::jit::script::Module ra_model_;
    torch::jit::script::Module rec_model_;    // recovery policy (ROS1: loaded at startup)
    double ra_value_ = -1.0;
    bool ra_loaded_ = false;
    bool rec_loaded_ = false;
    bool use_rl_thread_ = true;
    std::thread rl_thread_;
    bool running_ = false;
    bool updated_ = false;

    // output buffer
    torch::Tensor output_torques;
    torch::Tensor output_dof_pos_;

    // Ray2d shared memory
    float* ray2d_shm_ptr_ = nullptr;
    int ray2d_shm_fd_ = -1;

    // ABS episode timer
    double episode_timer_ = 0.0;
    int rl_step_count_ = 0;
    int sync_decimation_counter_ = 0;

    // World-frame goal position (from YAML config, matches paper GOAL_XYZ)
    double goal_x_ = 7.0;
    double goal_y_ = 0.0;

    // Inline ABS recovery state. Keep recovery active briefly after entry so
    // single-frame RA dips do not interrupt the escape maneuver.
    bool in_recovery_ = false;
    int recovery_hold_count_ = 0;
    int recovery_hold_steps_ = 25;

    // Soft start: ramp Kp/Kd from 0 to target over first N steps
    // Counter increments at controller rate (500Hz), so 250 steps ≈ 0.5s
    mutable int soft_start_step_ = 0;
    int soft_start_steps_ = 250;

    bool useRos1PolicyOrder() const;
    torch::Tensor ctrlToPolicyDofOrder(const torch::Tensor& ctrl_order) const;
    torch::Tensor policyToCtrlDofOrder(const torch::Tensor& policy_order) const;
    torch::Tensor ctrlToPolicyContactOrder(const torch::Tensor& ctrl_order) const;
};


#endif //STATERL_H
