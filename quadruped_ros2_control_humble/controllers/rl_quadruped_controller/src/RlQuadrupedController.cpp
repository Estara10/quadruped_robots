//
// Created by tlab-uav on 24-10-4.
//

#include "RlQuadrupedController.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <unordered_map>

namespace rl_quadruped_controller
{
    using config_type = controller_interface::interface_configuration_type;

    controller_interface::InterfaceConfiguration LeggedGymController::command_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration conf = {config_type::INDIVIDUAL, {}};

        conf.names.reserve(joint_names_.size() * command_interface_types_.size());
        for (const auto& joint_name : joint_names_)
        {
            for (const auto& interface_type : command_interface_types_)
            {
                if (!command_prefix_.empty())
                {
                    conf.names.push_back(command_prefix_ + "/" + joint_name + "/" += interface_type);
                }
                else
                {
                    conf.names.push_back(joint_name + "/" += interface_type);
                }
            }
        }

        return conf;
    }

    controller_interface::InterfaceConfiguration LeggedGymController::state_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration conf = {config_type::INDIVIDUAL, {}};

        conf.names.reserve(joint_names_.size() * state_interface_types_.size());
        for (const auto& joint_name : joint_names_)
        {
            for (const auto& interface_type : state_interface_types_)
            {
                conf.names.push_back(joint_name + "/" += interface_type);
            }
        }

        for (const auto& interface_type : imu_interface_types_)
        {
            conf.names.push_back(imu_name_ + "/" += interface_type);
        }

        for (const auto& interface_type : foot_force_interface_types_)
        {
            conf.names.push_back(foot_force_name_ + "/" += interface_type);
        }

        for (const auto& interface_type : odom_interface_types_)
        {
            conf.names.push_back(odom_name_ + "/" += interface_type);
        }

        return conf;
    }

    controller_interface::return_type LeggedGymController::
    update(const rclcpp::Time& time, const rclcpp::Duration& period)
    {
        // Global hard stop: handled before estimator/model/FSM logic so it works
        // from FIXEDDOWN, FIXEDSTAND, RL, and RL_REC in both simulation and real robot.
        const bool hard_stop_requested = ctrl_interfaces_.control_inputs_.command == 1 ||
                                         ctrl_interfaces_.control_inputs_.command == 9;
        if (abs_normal_shutdown::evaluate(hard_stop_requested, false, true, false,
                                          0, 0, 0, 0) == abs_normal_shutdown::Decision::ABORT)
        {
            const int stop_command = ctrl_interfaces_.control_inputs_.command;
            if (normal_shutdown_stage_ != NormalShutdownStage::NONE &&
                normal_shutdown_stage_ != NormalShutdownStage::RELEASED &&
                normal_shutdown_stage_ != NormalShutdownStage::FAILED &&
                normal_shutdown_stage_ != NormalShutdownStage::ABORTED)
            {
                const auto interrupted_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                    std::chrono::steady_clock::now().time_since_epoch()).count();
                RCLCPP_ERROR(get_node()->get_logger(),
                    "[NORMAL-SHUTDOWN] phase=ABORTED steady_ns=%lld reason=hard_stop command=%d",
                    static_cast<long long>(interrupted_ns), stop_command);
                normal_shutdown_stage_ = NormalShutdownStage::ABORTED;
            }
            bool transitioned = false;
            if (current_state_ && current_state_->state_name != FSMStateName::PASSIVE)
            {
                RCLCPP_ERROR(get_node()->get_logger(),
                             "[HARD-STOP] command=%d -> forcing PASSIVE",
                             stop_command);
                current_state_->exit();
                current_state_ = state_list_.passive;
                current_state_->enter();
                next_state_ = current_state_;
                next_state_name_ = current_state_->state_name;
                mode_ = FSMMode::NORMAL;
                transitioned = true;
            }
            const auto stop_complete_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::steady_clock::now().time_since_epoch()).count();
            RCLCPP_INFO(get_node()->get_logger(),
                        "[HARD-STOP-CONFIRMED] command=%d state=PASSIVE steady_ns=%lld changed=%d",
                        stop_command, static_cast<long long>(stop_complete_ns), transitioned ? 1 : 0);
            ctrl_interfaces_.control_inputs_.command = 0;
            return controller_interface::return_type::OK;
        }

        // The controlled path reaches PASSIVE only after controller-side
        // measured DOWN confirmation. Read back the next-cycle command values
        // before acknowledging release; this is not a MuJoCo consumption ack.
        if (normal_shutdown_stage_ == NormalShutdownStage::RELEASE_PENDING &&
            current_state_ && current_state_->state_name == FSMStateName::FIXEDDOWN)
        {
            current_state_->exit();
            current_state_ = state_list_.passive;
            current_state_->enter();
            next_state_ = current_state_;
            next_state_name_ = current_state_->state_name;
            mode_ = FSMMode::NORMAL;
            double max_kp = 0.0, max_kd = 0.0, max_tau = 0.0;
            for (int i = 0; i < 12; ++i)
            {
                max_kp = std::max(max_kp, std::abs(ctrl_interfaces_.joint_kp_command_interface_[i].get().get_value()));
                max_kd = std::max(max_kd, std::abs(ctrl_interfaces_.joint_kd_command_interface_[i].get().get_value()));
                max_tau = std::max(max_tau, std::abs(ctrl_interfaces_.joint_torque_command_interface_[i].get().get_value()));
            }
            RCLCPP_INFO(get_node()->get_logger(),
                "[NORMAL-SHUTDOWN] phase=UNLOADING steady_ns=%lld controller_state=PASSIVE "
                "down_confirmed=1 kp_max=%.6f kd_max=%.6f tau_max=%.6f",
                static_cast<long long>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                    std::chrono::steady_clock::now().time_since_epoch()).count()), max_kp, max_kd, max_tau);
            return controller_interface::return_type::OK;
        }
        if (normal_shutdown_stage_ == NormalShutdownStage::RELEASE_PENDING &&
            current_state_ && current_state_->state_name == FSMStateName::PASSIVE)
        {
            double max_kp = 0.0, max_kd = 0.0, max_tau = 0.0;
            for (int i = 0; i < 12; ++i)
            {
                max_kp = std::max(max_kp, std::abs(ctrl_interfaces_.joint_kp_command_interface_[i].get().get_value()));
                max_kd = std::max(max_kd, std::abs(ctrl_interfaces_.joint_kd_command_interface_[i].get().get_value()));
                max_tau = std::max(max_tau, std::abs(ctrl_interfaces_.joint_torque_command_interface_[i].get().get_value()));
            }
            const bool released = max_kp == 0.0 && max_kd == 0.0 && max_tau == 0.0;
            if (released)
            {
                const auto released_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                    std::chrono::steady_clock::now().time_since_epoch()).count();
                RCLCPP_INFO(get_node()->get_logger(),
                    "[NORMAL-SHUTDOWN] phase=RELEASED steady_ns=%lld controller_state=PASSIVE "
                    "kp_max=%.6f kd_max=%.6f tau_max=%.6f evidence=command_interface_readback "
                    "mujoco_consumption=UNKNOWN",
                    static_cast<long long>(released_ns), max_kp, max_kd, max_tau);
                normal_shutdown_stage_ = NormalShutdownStage::RELEASED;
                return controller_interface::return_type::OK;
            }
        }

        // ===== DDS Timeout Detection =====
        // Check if joint positions have frozen (DDS communication lost)
        // Skip in PASSIVE state — robot is not being controlled, joints naturally idle
        if (!last_joint_positions_.empty() && !dds_timeout_triggered_
            && current_state_->state_name != FSMStateName::PASSIVE)
        {
            bool frozen = true;
            for (size_t i = 0; i < last_joint_positions_.size(); i++)
            {
                double current = ctrl_interfaces_.joint_position_state_interface_[i].get().get_value();
                if (std::abs(current - last_joint_positions_[i]) > 1e-8)
                {
                    frozen = false;
                }
                last_joint_positions_[i] = current;
            }
            if (frozen)
            {
                dds_timeout_counter_++;
                if (dds_timeout_counter_ >= dds_timeout_threshold_)
                {
                    RCLCPP_FATAL(get_node()->get_logger(),
                        "[EMERGENCY] DDS timeout detected (%d steps frozen)! Forcing PASSIVE!",
                        dds_timeout_counter_);
                    dds_timeout_triggered_ = true;
                    // Force switch to PASSIVE immediately
                    if (current_state_->state_name != FSMStateName::PASSIVE)
                    {
                        current_state_->exit();
                        current_state_ = state_list_.passive;
                        current_state_->enter();
                        mode_ = FSMMode::NORMAL;
                    }
                    return controller_interface::return_type::OK;
                }
            }
            else if (dds_timeout_counter_ > 0)
            {
                // Data flowing again — reset (but keep triggered flag)
                dds_timeout_counter_ = 0;
            }
        }

        if (ctrl_component_.enable_estimator_)
        {
            if (ctrl_component_.robot_model_ == nullptr)
            {
                return controller_interface::return_type::OK;
            }

            ctrl_component_.robot_model_->update();
            ctrl_component_.estimator_->update();
        }

        if (mode_ == FSMMode::NORMAL)
        {
            current_state_->run(time, period);
            next_state_name_ = current_state_->checkChange();
            if (next_state_name_ != current_state_->state_name)
            {
                if (next_state_name_ == FSMStateName::RL_REC &&
                    !state_list_.rlRec->hasSelectedLinearVelocity())
                {
                    RCLCPP_ERROR_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                        "Recovery entry blocked: selected velocity source is unavailable or non-finite "
                        "(navigation_state_source=%s, recovery_velocity_source=%s)",
                        navigation_state_source_name_.c_str(),
                        recoveryVelocitySourceName(recovery_velocity_source_));
                    next_state_name_ = current_state_->state_name;
                    updateNormalShutdown();
                    return controller_interface::return_type::OK;
                }
                mode_ = FSMMode::CHANGE;
                next_state_ = getNextState(next_state_name_);
                RCLCPP_INFO(get_node()->get_logger(), "Switched from %s to %s",
                            current_state_->state_name_string.c_str(), next_state_->state_name_string.c_str());
            }
        }
        else if (mode_ == FSMMode::CHANGE)
        {
            const bool entering_normal_stand = current_state_ == state_list_.rl &&
                next_state_name_ == FSMStateName::FIXEDSTAND && state_list_.rl->normalShutdownReady();
            current_state_->exit();
            current_state_ = next_state_;

            current_state_->enter();
            mode_ = FSMMode::NORMAL;
            if (entering_normal_stand)
            {
                normal_shutdown_stage_ = NormalShutdownStage::STANDING;
                normal_shutdown_stage_started_ns_ = std::chrono::duration_cast<std::chrono::nanoseconds>(
                    std::chrono::steady_clock::now().time_since_epoch()).count();
                normal_shutdown_stable_since_ns_ = 0;
                RCLCPP_INFO(get_node()->get_logger(),
                    "[NORMAL-SHUTDOWN] phase=STANDING steady_ns=%lld controller_state=FIXEDSTAND support_gains_continuous=1",
                    static_cast<long long>(normal_shutdown_stage_started_ns_));
            }
        }

        updateNormalShutdown();

        return controller_interface::return_type::OK;
    }

    abs_normal_shutdown::Feedback LeggedGymController::sampleNormalShutdownFeedback(
        const std::vector<double>& target) const
    {
        abs_normal_shutdown::Feedback feedback{};
        bool valid = target.size() == 12 &&
            ctrl_interfaces_.joint_position_state_interface_.size() >= 12 &&
            ctrl_interfaces_.joint_velocity_state_interface_.size() >= 12 &&
            ctrl_interfaces_.joint_kp_command_interface_.size() >= 12 &&
            ctrl_interfaces_.joint_kd_command_interface_.size() >= 12 &&
            ctrl_interfaces_.imu_state_interface_.size() >= 7 &&
            ctrl_interfaces_.foot_force_state_interface_.size() >= 4 &&
            ctrl_interfaces_.odom_state_interface_.size() >= 6;
        if (!valid) return feedback;

        // The independent source provides no measured base height (its world Z is
        // an unvalidated VINS height, not ground clearance). The diagnostic
        // closeout therefore stays unavailable instead of reusing odometer values.
        if (ctrl_component_.navigation_state_source_ == abs_navigation_state::Source::VINS_BODY)
        {
            RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 5000,
                "[NAV-STATE] source=vins_body: odometer height/speed are not used; "
                "normal closeout confirmation stays unavailable");
            return feedback;
        }

        const double vx = ctrl_interfaces_.odom_state_interface_[3].get().get_value();
        const double vy = ctrl_interfaces_.odom_state_interface_[4].get().get_value();
        const double vz = ctrl_interfaces_.odom_state_interface_[5].get().get_value();
        feedback.horizontal_speed_mps = std::hypot(vx, vy);
        feedback.base_height_m = ctrl_interfaces_.odom_state_interface_[2].get().get_value();        const double w = ctrl_interfaces_.imu_state_interface_[0].get().get_value();
        const double x = ctrl_interfaces_.imu_state_interface_[1].get().get_value();
        const double y = ctrl_interfaces_.imu_state_interface_[2].get().get_value();
        const double z = ctrl_interfaces_.imu_state_interface_[3].get().get_value();
        const double gx = ctrl_interfaces_.imu_state_interface_[4].get().get_value();
        const double gy = ctrl_interfaces_.imu_state_interface_[5].get().get_value();
        const double gz = ctrl_interfaces_.imu_state_interface_[6].get().get_value();
        feedback.angular_speed_radps = std::hypot(std::hypot(gx, gy), gz);
        feedback.roll_rad = std::atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));
        feedback.pitch_rad = std::asin(std::clamp(2.0 * (w * y - z * x), -1.0, 1.0));
        for (int i = 0; i < 12; ++i)
        {
            const double q = ctrl_interfaces_.joint_position_state_interface_[i].get().get_value();
            const double dq = ctrl_interfaces_.joint_velocity_state_interface_[i].get().get_value();
            if (!std::isfinite(q) || !std::isfinite(dq) || !std::isfinite(target[i])) valid = false;
            feedback.max_joint_position_error_rad = std::max(feedback.max_joint_position_error_rad,
                                                               std::abs(q - target[i]));
            feedback.max_joint_speed_radps = std::max(feedback.max_joint_speed_radps, std::abs(dq));
            feedback.max_kp_command = std::max(feedback.max_kp_command,
                std::abs(ctrl_interfaces_.joint_kp_command_interface_[i].get().get_value()));
            feedback.max_kd_command = std::max(feedback.max_kd_command,
                std::abs(ctrl_interfaces_.joint_kd_command_interface_[i].get().get_value()));
        }
        for (const auto& sensor : ctrl_interfaces_.foot_force_state_interface_)
        {
            const double force = sensor.get().get_value();
            if (!std::isfinite(force)) valid = false;
            if (std::isfinite(force) && force > feet_force_threshold_) ++feedback.supported_feet;
        }
        valid = valid && std::isfinite(vx) && std::isfinite(vy) && std::isfinite(vz) &&
                std::isfinite(feedback.base_height_m) && std::isfinite(w) && std::isfinite(x) &&
                std::isfinite(y) && std::isfinite(z) && std::isfinite(gx) && std::isfinite(gy) &&
                std::isfinite(gz);
        feedback.valid = valid && abs_normal_shutdown::finite(feedback);
        return feedback;
    }

    void LeggedGymController::logNormalShutdownFeedback(
        const char* phase, const abs_normal_shutdown::Feedback& f, uint64_t stable_since_ns) const
    {
        std::ostringstream q, dq;
        q << std::fixed << std::setprecision(4);
        dq << std::fixed << std::setprecision(4);
        for (int i = 0; i < 12; ++i)
        {
            if (i) { q << ','; dq << ','; }
            q << ctrl_interfaces_.joint_position_state_interface_[i].get().get_value();
            dq << ctrl_interfaces_.joint_velocity_state_interface_[i].get().get_value();
        }
        const auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
        RCLCPP_INFO(get_node()->get_logger(),
            "[NORMAL-SHUTDOWN] phase=%s steady_ns=%lld stable_since_ns=%llu stable_ms=400 "
            "speed_xy=%.5f angular_speed=%.5f base_z=%.5f roll=%.5f pitch=%.5f "
            "max_q_error=%.5f max_qdot=%.5f supported_feet=%d max_kp=%.5f max_kd=%.5f q=[%s] dq=[%s]",
            phase, static_cast<long long>(now_ns), static_cast<unsigned long long>(stable_since_ns),
            f.horizontal_speed_mps, f.angular_speed_radps, f.base_height_m, f.roll_rad, f.pitch_rad,
            f.max_joint_position_error_rad, f.max_joint_speed_radps, f.supported_feet,
            f.max_kp_command, f.max_kd_command,
            q.str().c_str(), dq.str().c_str());
    }

    void LeggedGymController::updateNormalShutdown()
    {
        const auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
        if (normal_shutdown_stage_ == NormalShutdownStage::STANDING &&
            current_state_ == state_list_.fixedStand)
        {
            const auto feedback = sampleNormalShutdownFeedback(stand_pos_);
            const double roll_pitch = std::max(std::abs(feedback.roll_rad), std::abs(feedback.pitch_rad));
            const bool condition = abs_normal_shutdown::standingConfirmed(feedback) &&
                                  abs_normal_shutdown::standingSupportEnabled(feedback);
            if (condition)
            {
                if (normal_shutdown_stable_since_ns_ == 0) normal_shutdown_stable_since_ns_ = now_ns;
            }
            else normal_shutdown_stable_since_ns_ = 0;
            const auto decision = abs_normal_shutdown::evaluate(
                false, feedback.valid && roll_pitch >= 1.30, feedback.valid, condition,
                normal_shutdown_stable_since_ns_ == 0 ? 0 : now_ns - normal_shutdown_stable_since_ns_,
                now_ns - normal_shutdown_stage_started_ns_, 400000000ULL, 10000000000ULL);
            if (decision == abs_normal_shutdown::Decision::CONFIRM)
            {
                logNormalShutdownFeedback("STANDING_CONFIRMED", feedback, normal_shutdown_stable_since_ns_);
                const auto hold_started_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                    std::chrono::steady_clock::now().time_since_epoch()).count();
                normal_shutdown_stage_ = NormalShutdownStage::STANDING_HOLD;
                normal_shutdown_stage_started_ns_ = static_cast<uint64_t>(hold_started_ns);
                normal_shutdown_stable_since_ns_ = 0;
                RCLCPP_INFO(get_node()->get_logger(),
                    "[NORMAL-SHUTDOWN] phase=STANDING_HOLD steady_ns=%lld controller_state=FIXEDSTAND "
                    "support_gains=maintained auto_lower=0 auto_passive=0",
                    static_cast<long long>(hold_started_ns));
            }
            else if (decision == abs_normal_shutdown::Decision::FAIL)
            {
                normal_shutdown_stage_ = NormalShutdownStage::FAILED;
                RCLCPP_ERROR(get_node()->get_logger(),
                    "[NORMAL-SHUTDOWN] phase=FAILED steady_ns=%lld stage=STANDING reason=%s",
                    static_cast<long long>(now_ns), !feedback.valid ? "invalid_feedback" :
                    (roll_pitch >= 1.30 ? "abnormal_tilt" : "stage_timeout"));
            }
        }
        else if (normal_shutdown_stage_ == NormalShutdownStage::STANDING_HOLD &&
                 current_state_ == state_list_.fixedStand)
        {
            const auto feedback = sampleNormalShutdownFeedback(stand_pos_);
            const double roll_pitch = std::max(std::abs(feedback.roll_rad), std::abs(feedback.pitch_rad));
            if (!abs_normal_shutdown::standingConfirmed(feedback) ||
                !abs_normal_shutdown::standingSupportEnabled(feedback) || roll_pitch >= 1.30)
            {
                normal_shutdown_stage_ = NormalShutdownStage::FAILED;
                RCLCPP_ERROR(get_node()->get_logger(),
                    "[NORMAL-SHUTDOWN] phase=FAILED steady_ns=%lld stage=STANDING_HOLD "
                    "reason=%s controller_state=FIXEDSTAND max_kp=%.5f max_kd=%.5f",
                    static_cast<long long>(now_ns), !feedback.valid ? "invalid_feedback" :
                    ((!abs_normal_shutdown::standingSupportEnabled(feedback)) ? "support_gain_lost" : "standing_condition_lost"),
                    feedback.max_kp_command, feedback.max_kd_command);
            }
        }
        else if (normal_shutdown_stage_ == NormalShutdownStage::LOWERING &&
                 current_state_ == state_list_.fixedDown)
        {
            const auto feedback = sampleNormalShutdownFeedback(down_pos_);
            const double roll_pitch = std::max(std::abs(feedback.roll_rad), std::abs(feedback.pitch_rad));
            const bool condition = abs_normal_shutdown::downConfirmed(feedback);
            if (condition)
            {
                if (normal_shutdown_stable_since_ns_ == 0) normal_shutdown_stable_since_ns_ = now_ns;
            }
            else normal_shutdown_stable_since_ns_ = 0;
            const auto decision = abs_normal_shutdown::evaluate(
                false, feedback.valid && roll_pitch >= 1.30, feedback.valid, condition,
                normal_shutdown_stable_since_ns_ == 0 ? 0 : now_ns - normal_shutdown_stable_since_ns_,
                now_ns - normal_shutdown_stage_started_ns_, 400000000ULL, 10000000000ULL);
            if (decision == abs_normal_shutdown::Decision::CONFIRM)
            {
                logNormalShutdownFeedback("DOWN_CONFIRMED", feedback, normal_shutdown_stable_since_ns_);
                normal_shutdown_stage_ = NormalShutdownStage::RELEASE_PENDING;
                return;
            }
            else if (decision == abs_normal_shutdown::Decision::FAIL)
            {
                normal_shutdown_stage_ = NormalShutdownStage::FAILED;
                RCLCPP_ERROR(get_node()->get_logger(),
                    "[NORMAL-SHUTDOWN] phase=FAILED steady_ns=%lld stage=LOWERING reason=%s",
                    static_cast<long long>(now_ns), !feedback.valid ? "invalid_feedback" :
                    (roll_pitch >= 1.30 ? "abnormal_tilt" : "stage_timeout"));
            }
        }
    }

    controller_interface::CallbackReturn LeggedGymController::on_init()
    {
        try
        {
            joint_names_ = auto_declare<std::vector<std::string>>("joints", joint_names_);
            feet_names_ = auto_declare<std::vector<std::string>>("feet_names", feet_names_);
            command_interface_types_ =
                auto_declare<std::vector<std::string>>("command_interfaces", command_interface_types_);
            state_interface_types_ =
                auto_declare<std::vector<std::string>>("state_interfaces", state_interface_types_);

            command_prefix_ = auto_declare<std::string>("command_prefix", command_prefix_);
            base_name_ = auto_declare<std::string>("base_name", base_name_);

            // imu sensor
            imu_name_ = auto_declare<std::string>("imu_name", imu_name_);
            imu_interface_types_ = auto_declare<std::vector<std::string>>("imu_interfaces", state_interface_types_);

            // foot_force_sensor
            foot_force_name_ = auto_declare<std::string>("foot_force_name", foot_force_name_);
            foot_force_interface_types_ =
                auto_declare<std::vector<std::string>>("foot_force_interfaces", foot_force_interface_types_);
            feet_force_threshold_ = auto_declare<double>("feet_force_threshold", feet_force_threshold_);

            // odometer sensor (world-frame position + velocity from MuJoCo)
            odom_name_ = auto_declare<std::string>("odom_name", odom_name_);
            odom_interface_types_ =
                auto_declare<std::vector<std::string>>("odom_interfaces", odom_interface_types_);

            // pose parameters
            down_pos_ = auto_declare<std::vector<double>>("down_pos", down_pos_);
            stand_pos_ = auto_declare<std::vector<double>>("stand_pos", stand_pos_);
            stand_kp_ = auto_declare<double>("stand_kp", stand_kp_);
            stand_kd_ = auto_declare<double>("stand_kd", stand_kd_);
            const std::string recovery_velocity_source = auto_declare<std::string>(
                "recovery_velocity_source", "odom");
            recovery_velocity_source_ = parseRecoveryVelocitySource(recovery_velocity_source);

            // Explicit navigation/velocity state source. Default keeps the
            // existing odometer behaviour; "vins_body" is a candidate that must
            // be selected deliberately and never falls back silently.
            navigation_state_source_name_ = auto_declare<std::string>(
                "navigation_state_source", navigation_state_source_name_);
            external_state_topic_ = auto_declare<std::string>(
                "external_state_topic", external_state_topic_);
            external_state_world_frame_ = auto_declare<std::string>(
                "external_state_world_frame", external_state_world_frame_);
            external_state_body_frame_ = auto_declare<std::string>(
                "external_state_body_frame", external_state_body_frame_);
            external_state_timeout_ms_ = auto_declare<int64_t>(
                "external_state_timeout_ms", external_state_timeout_ms_);
            ctrl_component_.navigation_state_source_ =
                abs_navigation_state::parseSource(navigation_state_source_name_);
            if (ctrl_component_.navigation_state_source_ == abs_navigation_state::Source::VINS_BODY)
            {
                if (external_state_timeout_ms_ <= 0)
                {
                    throw std::invalid_argument("external_state_timeout_ms must be positive");
                }
                // Provisional value: not validated on hardware. It only turns a
                // missing/stale sample into a fail-closed stop; it is not a
                // claim about the source's real latency or accuracy.
                ctrl_component_.external_state_timeout_ns_ =
                    static_cast<uint64_t>(external_state_timeout_ms_) * 1'000'000ULL;
                ctrl_component_.external_body_state_ =
                    std::make_shared<abs_navigation_state::Mailbox>();
                external_body_state_ = ctrl_component_.external_body_state_;
                RCLCPP_WARN(get_node()->get_logger(),
                    "[NAV-STATE] source=vins_body topic=%s world_frame=%s body_frame=%s timeout_ms=%ld "
                    "(provisional, not field-validated); odometer is NOT used as fallback",
                    external_state_topic_.c_str(), external_state_world_frame_.c_str(),
                    external_state_body_frame_.c_str(), static_cast<long>(external_state_timeout_ms_));
            }
            else
            {
                RCLCPP_INFO(get_node()->get_logger(), "[NAV-STATE] source=odom (default)");
            }

            get_node()->get_parameter("update_rate", ctrl_interfaces_.frequency_);
            RCLCPP_INFO(get_node()->get_logger(), "Controller Update Rate: %d Hz", ctrl_interfaces_.frequency_);

            if (foot_force_interface_types_.size() == 4)
            {
                RCLCPP_INFO(get_node()->get_logger(), "Enable Estimator");
                ctrl_component_.enable_estimator_ = true;
                ctrl_component_.estimator_ = std::make_shared<Estimator>(ctrl_interfaces_, ctrl_component_);
            }
            ctrl_component_.node_ = get_node();
        }
        catch (const std::exception& e)
        {
            fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
            return controller_interface::CallbackReturn::ERROR;
        }

        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn LeggedGymController::on_configure(
        const rclcpp_lifecycle::State& /*previous_state*/)
    {
        robot_description_subscription_ = get_node()->create_subscription<std_msgs::msg::String>(
            "/robot_description", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local(),
            [this](const std_msgs::msg::String::SharedPtr msg)
            {
                if (ctrl_component_.enable_estimator_)
                {
                    ctrl_component_.robot_model_ = std::make_shared<QuadrupedRobot>(
                        ctrl_interfaces_, msg->data, feet_names_, base_name_);
                }
            });


        control_input_subscription_ = get_node()->create_subscription<control_input_msgs::msg::Inputs>(
            "/control_input", 10, [this](const control_input_msgs::msg::Inputs::SharedPtr msg)
            {
                // Handle message
                ctrl_interfaces_.control_inputs_.command = msg->command;
                ctrl_interfaces_.control_inputs_.lx = msg->lx;
                ctrl_interfaces_.control_inputs_.ly = msg->ly;
                ctrl_interfaces_.control_inputs_.rx = msg->rx;
                ctrl_interfaces_.control_inputs_.ry = msg->ry;
            });

        // Read-only independent navigation state. Created only when that source
        // was explicitly selected; otherwise no topic is subscribed at all.
        if (external_body_state_)
        {
            external_state_subscription_ = get_node()->create_subscription<nav_msgs::msg::Odometry>(
                external_state_topic_, rclcpp::QoS(rclcpp::KeepLast(10)),
                [this](const nav_msgs::msg::Odometry::SharedPtr msg)
                {
                    onExternalBodyState(*msg);
                });
        }

        return CallbackReturn::SUCCESS;
    }

    void LeggedGymController::onExternalBodyState(const nav_msgs::msg::Odometry& msg)
    {
        if (msg.header.frame_id != external_state_world_frame_ ||
            msg.child_frame_id != external_state_body_frame_)
        {
            RCLCPP_ERROR_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 2000,
                "[NAV-STATE] frame mismatch: got '%s'/'%s', expected '%s'/'%s'; sample rejected",
                msg.header.frame_id.c_str(), msg.child_frame_id.c_str(),
                external_state_world_frame_.c_str(), external_state_body_frame_.c_str());
            external_body_state_->invalidate();
            return;
        }

        const auto& pose = msg.pose.pose;
        const auto& twist = msg.twist.twist;
        abs_navigation_state::Sample sample;
        sample.stamp_ns = static_cast<int64_t>(msg.header.stamp.sec) * 1'000'000'000LL +
                          static_cast<int64_t>(msg.header.stamp.nanosec);
        sample.arrival_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
        sample.position = {pose.position.x, pose.position.y, pose.position.z};
        sample.quaternion = {pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w};
        sample.linear_velocity = {twist.linear.x, twist.linear.y, twist.linear.z};
        sample.angular_velocity = {twist.angular.x, twist.angular.y, twist.angular.z};

        if (!abs_navigation_state::finite(sample))
        {
            RCLCPP_ERROR_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 2000,
                "[NAV-STATE] non-finite or missing stamp; sample rejected");
            external_body_state_->invalidate();
            return;
        }
        // The twist is the base-frame velocity published by go2_body_odometry.
        // It is stored and consumed as-is; no world->body rotation is applied.
        external_body_state_->store(sample);
        if (external_body_state_->tripped())
        {
            RCLCPP_ERROR_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 2000,
                "[NAV-STATE] source time regression latched; messages stay rejected until a new "
                "RL session (regressions=%lu rejected_after_trip=%lu)",
                static_cast<unsigned long>(external_body_state_->stampRejections()),
                static_cast<unsigned long>(external_body_state_->rejectedAfterTrip()));
        }
    }

    controller_interface::CallbackReturn LeggedGymController::on_activate(
        const rclcpp_lifecycle::State& /*previous_state*/)
    {
        normal_shutdown_stage_ = NormalShutdownStage::NONE;
        normal_shutdown_stage_started_ns_ = 0;
        normal_shutdown_stable_since_ns_ = 0;
        // clear out vectors in case of restart
        ctrl_interfaces_.clear();

        // assign command interfaces
        for (auto& interface : command_interfaces_)
        {
            std::string interface_name = interface.get_interface_name();
            if (const size_t pos = interface_name.find('/'); pos != std::string::npos)
            {
                command_interface_map_[interface_name.substr(pos + 1)]->push_back(interface);
            }
            else
            {
                command_interface_map_[interface_name]->push_back(interface);
            }
        }

        // assign state interfaces
        size_t foot_force_loan_index = 0;
        for (auto& interface : state_interfaces_)
        {
            if (interface.get_prefix_name() == imu_name_)
            {
                ctrl_interfaces_.imu_state_interface_.emplace_back(interface);
            }
            else if (interface.get_prefix_name() == foot_force_name_)
            {
                RCLCPP_INFO(get_node()->get_logger(),
                            "[FOOT-LOANED] index=%zu name=%s/%s",
                            foot_force_loan_index++, interface.get_prefix_name().c_str(),
                            interface.get_interface_name().c_str());
                ctrl_interfaces_.foot_force_state_interface_.emplace_back(interface);
            }
            else if (interface.get_prefix_name() == odom_name_)
            {
                ctrl_interfaces_.odom_state_interface_.emplace_back(interface);
            }
            else
            {
                state_interface_map_[interface.get_interface_name()]->push_back(interface);
            }
        }

        std::unordered_map<std::string, size_t> joint_index;
        for (size_t i = 0; i < joint_names_.size(); ++i)
        {
            joint_index[joint_names_[i]] = i;
        }
        const auto sort_joint_interfaces = [&joint_index](auto& interfaces)
        {
            std::sort(interfaces.begin(), interfaces.end(),
                [&joint_index](const auto& lhs, const auto& rhs)
                {
                    const auto lhs_it = joint_index.find(lhs.get().get_prefix_name());
                    const auto rhs_it = joint_index.find(rhs.get().get_prefix_name());
                    const size_t lhs_idx = lhs_it == joint_index.end() ? joint_index.size() : lhs_it->second;
                    const size_t rhs_idx = rhs_it == joint_index.end() ? joint_index.size() : rhs_it->second;
                    return lhs_idx < rhs_idx;
                });
        };

        sort_joint_interfaces(ctrl_interfaces_.joint_torque_command_interface_);
        sort_joint_interfaces(ctrl_interfaces_.joint_position_command_interface_);
        sort_joint_interfaces(ctrl_interfaces_.joint_velocity_command_interface_);
        sort_joint_interfaces(ctrl_interfaces_.joint_kp_command_interface_);
        sort_joint_interfaces(ctrl_interfaces_.joint_kd_command_interface_);
        sort_joint_interfaces(ctrl_interfaces_.joint_effort_state_interface_);
        sort_joint_interfaces(ctrl_interfaces_.joint_position_state_interface_);
        sort_joint_interfaces(ctrl_interfaces_.joint_velocity_state_interface_);

        if (ctrl_interfaces_.joint_position_state_interface_.size() == joint_names_.size())
        {
            RCLCPP_INFO(get_node()->get_logger(),
                        "[VERIFY] joint interface order: %s %s %s ... %s",
                        ctrl_interfaces_.joint_position_state_interface_[0].get().get_prefix_name().c_str(),
                        ctrl_interfaces_.joint_position_state_interface_[1].get().get_prefix_name().c_str(),
                        ctrl_interfaces_.joint_position_state_interface_[2].get().get_prefix_name().c_str(),
                        ctrl_interfaces_.joint_position_state_interface_.back().get().get_prefix_name().c_str());
        }

        // Create FSM List
        state_list_.passive = std::make_shared<StatePassive>(ctrl_interfaces_);
        state_list_.fixedDown = std::make_shared<StateFixedDown>(ctrl_interfaces_, down_pos_, stand_kp_, stand_kd_);
        state_list_.fixedStand = std::make_shared<StateFixedStand>(ctrl_interfaces_, stand_pos_, stand_kp_, stand_kd_);
        state_list_.rl = std::make_shared<StateRL>(ctrl_interfaces_, ctrl_component_, stand_pos_);
        state_list_.rlRec = std::make_shared<StateRLRec>(
            ctrl_interfaces_, ctrl_component_, stand_pos_, recovery_velocity_source_);

        // Initialize FSM
        current_state_ = state_list_.passive;
        current_state_->enter();
        next_state_ = current_state_;
        next_state_name_ = current_state_->state_name;
        mode_ = FSMMode::NORMAL;

        // Init DDS timeout detection
        last_joint_positions_.resize(12, 0.0);
        dds_timeout_counter_ = 0;
        dds_timeout_triggered_ = false;
        // Seed with current positions after first read
        for (int i = 0; i < 12; i++)
        {
            last_joint_positions_[i] = ctrl_interfaces_.joint_position_state_interface_[i].get().get_value();
        }

        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn LeggedGymController::on_deactivate(
        const rclcpp_lifecycle::State& /*previous_state*/)
    {
        // Stop inference before clearing commands. Hardware stays active when
        // only this controller is deactivated: this unloads, not native takeover.
        if (current_state_)
        {
            current_state_->exit();
            current_state_ = state_list_.passive;
            current_state_->enter();
        }
        release_interfaces();
        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn
    LeggedGymController::on_cleanup(const rclcpp_lifecycle::State& previous_state)
    {
        return ControllerInterface::on_cleanup(previous_state);
    }

    controller_interface::CallbackReturn
    LeggedGymController::on_shutdown(const rclcpp_lifecycle::State& previous_state)
    {
        return ControllerInterface::on_shutdown(previous_state);
    }

    controller_interface::CallbackReturn LeggedGymController::on_error(const rclcpp_lifecycle::State& previous_state)
    {
        return ControllerInterface::on_error(previous_state);
    }

    std::shared_ptr<FSMState> LeggedGymController::getNextState(const FSMStateName stateName) const
    {
        switch (stateName)
        {
        case FSMStateName::INVALID:
            return state_list_.invalid;
        case FSMStateName::PASSIVE:
            return state_list_.passive;
        case FSMStateName::FIXEDDOWN:
            return state_list_.fixedDown;
        case FSMStateName::FIXEDSTAND:
            return state_list_.fixedStand;
        case FSMStateName::RL:
            return state_list_.rl;
        case FSMStateName::RL_REC:
            return state_list_.rlRec;
        default:
            return state_list_.invalid;
        }
    }
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(rl_quadruped_controller::LeggedGymController, controller_interface::ControllerInterface);
