//
// Created by biao on 24-9-9.
//

#include "hardware_unitree_mujoco/HardwareUnitree.h"

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

#include <cmath>
#include <algorithm>
#include <chrono>
#include <limits>
#include <unistd.h>

#include "crc32.h"

#define TOPIC_LOWCMD "rt/lowcmd"
#define TOPIC_LOWSTATE "rt/lowstate"
#define TOPIC_HIGHSTATE "rt/sportmodestate"

namespace
{
constexpr float PosStopF = 2.146E+9f;
constexpr float VelStopF = 16000.0f;
constexpr double CommandEps = 1e-9;

uint64_t monotonicNowNs()
{
    return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count());
}
}

using namespace unitree::robot;
using hardware_interface::return_type;

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn HardwareUnitree::on_init(
    const hardware_interface::HardwareInfo& info)
{
    if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
    {
        return CallbackReturn::ERROR;
    }

    joint_torque_command_.assign(12, 0);
    joint_position_command_.assign(12, 0);
    joint_velocities_command_.assign(12, 0);
    joint_kp_command_.assign(12, 0);
    joint_kd_command_.assign(12, 0);

    joint_position_.assign(12, 0);
    joint_velocities_.assign(12, 0);
    joint_effort_.assign(12, 0);

    imu_states_.assign(10, 0);
    foot_force_.assign(4, 0);
    high_states_.assign(6, 0);

    for (const auto& joint : info_.joints)
    {
        for (const auto& interface : joint.state_interfaces)
        {
            joint_interfaces[interface.name].push_back(joint.name);
        }
    }


    if (const auto network_interface_param = info.hardware_parameters.find("network_interface"); network_interface_param
        != info.hardware_parameters.end())
    {
        network_interface_ = network_interface_param->second;
    }
    if (const auto domain_param = info.hardware_parameters.find("domain"); domain_param != info.hardware_parameters.
        end())
    {
        domain_ = std::stoi(domain_param->second);
    }
    if (const auto show_foot_force_param = info.hardware_parameters.find("show_foot_force"); show_foot_force_param !=
        info.hardware_parameters.end())
    {
        show_foot_force_ = show_foot_force_param->second == "true";
    }
    if (const auto freshness_param = info.hardware_parameters.find("state_freshness_timeout_ms");
        freshness_param != info.hardware_parameters.end())
    {
        const long timeout_ms = std::stol(freshness_param->second);
        if (timeout_ms <= 0) return CallbackReturn::ERROR;
        state_freshness_timeout_ns_ = static_cast<uint64_t>(timeout_ms) * 1'000'000ULL;
    }

    RCLCPP_INFO(rclcpp::get_logger("unitree_hardware"), " network_interface: %s, domain: %d", network_interface_.c_str(), domain_);
    ChannelFactory::Instance()->Init(domain_, network_interface_);

    lows_tate_subscriber_ =
        std::make_shared<ChannelSubscriber<unitree_go::msg::dds_::LowState_>>(
            TOPIC_LOWSTATE);
    lows_tate_subscriber_->InitChannel(
        [this](auto&& PH1)
        {
            lowStateMessageHandle(std::forward<decltype(PH1)>(PH1));
        },
        1);
    initLowCmd();
    logMotorIndexMap();

    high_state_subscriber_ =
        std::make_shared<ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>>(
            TOPIC_HIGHSTATE);
    high_state_subscriber_->InitChannel(
        [this](auto&& PH1)
        {
            highStateMessageHandle(std::forward<decltype(PH1)>(PH1));
        },
        1);

    return CallbackReturn::SUCCESS;
}

HardwareUnitree::~HardwareUnitree()
{
    // Destruction must not let a transport exception terminate the process.
    try
    {
        stopAndRestoreMotionMode();
    }
    catch (const std::exception& error)
    {
        RCLCPP_ERROR(rclcpp::get_logger("unitree_hardware"),
                     "Shutdown failed: %s; native control takeover is NOT confirmed", error.what());
    }
}

HardwareUnitree::CallbackReturn HardwareUnitree::on_activate(const rclcpp_lifecycle::State&)
{
    std::lock_guard<std::mutex> lock(command_mutex_);
    if (network_interface_ != "lo")
    {
        if (restore_required_ || !releaseMotionMode())
        {
            restoreMotionMode();
            return CallbackReturn::ERROR;
        }
    }
    std::fill(joint_kp_command_.begin(), joint_kp_command_.end(), 0.0);
    std::fill(joint_kd_command_.begin(), joint_kd_command_.end(), 0.0);
    std::fill(joint_torque_command_.begin(), joint_torque_command_.end(), 0.0);
    initLowCmd();
    low_cmd_publisher_ = std::make_shared<ChannelPublisher<unitree_go::msg::dds_::LowCmd_>>(TOPIC_LOWCMD);
    try
    {
        low_cmd_publisher_->InitChannel();
    }
    catch (const std::exception& error)
    {
        low_cmd_publisher_->CloseChannel();
        low_cmd_publisher_.reset();
        RCLCPP_ERROR(rclcpp::get_logger("unitree_hardware"),
                     "LowCmd publisher activation failed: %s", error.what());
        restoreMotionMode();
        return CallbackReturn::ERROR;
    }
    return CallbackReturn::SUCCESS;
}

HardwareUnitree::CallbackReturn HardwareUnitree::on_deactivate(const rclcpp_lifecycle::State&)
{
    return stopAndRestoreMotionMode() ? CallbackReturn::SUCCESS : CallbackReturn::ERROR;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn HardwareUnitree::on_shutdown(
    const rclcpp_lifecycle::State& /* previous_state */)
{
    return stopAndRestoreMotionMode() ? CallbackReturn::SUCCESS : CallbackReturn::ERROR;
}

std::vector<hardware_interface::StateInterface> HardwareUnitree::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;

    int ind = 0;
    for (const auto& joint_name : joint_interfaces["position"])
    {
        state_interfaces.emplace_back(joint_name, "position", &joint_position_[ind++]);
    }

    ind = 0;
    for (const auto& joint_name : joint_interfaces["velocity"])
    {
        state_interfaces.emplace_back(joint_name, "velocity", &joint_velocities_[ind++]);
    }

    ind = 0;
    for (const auto& joint_name : joint_interfaces["effort"])
    {
        state_interfaces.emplace_back(joint_name, "effort", &joint_effort_[ind++]);
    }

    // export imu sensor state interface
    for (uint i = 0; i < info_.sensors[0].state_interfaces.size(); i++)
    {
        state_interfaces.emplace_back(
            info_.sensors[0].name, info_.sensors[0].state_interfaces[i].name, &imu_states_[i]);
    }

    // export foot force sensor state interface
    if (info_.sensors.size() > 1)
    {
        for (uint i = 0; i < info_.sensors[1].state_interfaces.size(); i++)
        {
            state_interfaces.emplace_back(
                info_.sensors[1].name, info_.sensors[1].state_interfaces[i].name, &foot_force_[i]);
        }
    }

    // export odometer state interface
    if (info_.sensors.size() > 2)
    {
        // export high state interface
        for (uint i = 0; i < info_.sensors[2].state_interfaces.size(); i++)
        {
            state_interfaces.emplace_back(
                info_.sensors[2].name, info_.sensors[2].state_interfaces[i].name, &high_states_[i]);
        }
    }


    return
        state_interfaces;
}

std::vector<hardware_interface::CommandInterface> HardwareUnitree::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    int ind = 0;
    for (const auto& joint_name : joint_interfaces["position"])
    {
        command_interfaces.emplace_back(joint_name, "position", &joint_position_command_[ind++]);
    }

    ind = 0;
    for (const auto& joint_name : joint_interfaces["velocity"])
    {
        command_interfaces.emplace_back(joint_name, "velocity", &joint_velocities_command_[ind++]);
    }

    ind = 0;
    for (const auto& joint_name : joint_interfaces["effort"])
    {
        command_interfaces.emplace_back(joint_name, "effort", &joint_torque_command_[ind]);
        command_interfaces.emplace_back(joint_name, "kp", &joint_kp_command_[ind]);
        command_interfaces.emplace_back(joint_name, "kd", &joint_kd_command_[ind]);
        ind++;
    }
    return command_interfaces;
}

return_type HardwareUnitree::read(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
    unitree_go::msg::dds_::LowState_ low_state;
    unitree_go::msg::dds_::SportModeState_ high_state;
    uint64_t low_received_ns = 0;
    uint64_t sport_received_ns = 0;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        low_state = low_state_;
        high_state = high_state_;
        low_received_ns = low_state_received_ns_;
        sport_received_ns = sport_state_received_ns_;
    }
    const uint64_t now_ns = monotonicNowNs();
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const bool low_fresh = low_received_ns != 0 && now_ns >= low_received_ns &&
                           now_ns - low_received_ns <= state_freshness_timeout_ns_;
    const bool sport_fresh = sport_received_ns != 0 && now_ns >= sport_received_ns &&
                             now_ns - sport_received_ns <= state_freshness_timeout_ns_;

    // joint states
    for (int i(0); i < 12; ++i)
    {
        const int motor_idx = motor_index_map_[i];
        joint_position_[i] = low_fresh ? low_state.motor_state()[motor_idx].q() : nan;
        joint_velocities_[i] = low_fresh ? low_state.motor_state()[motor_idx].dq() : nan;
        joint_effort_[i] = low_fresh ? low_state.motor_state()[motor_idx].tau_est() : nan;
    }

    // imu states
    imu_states_[0] = low_fresh ? low_state.imu_state().quaternion()[0] : nan; // w
    imu_states_[1] = low_fresh ? low_state.imu_state().quaternion()[1] : nan; // x
    imu_states_[2] = low_fresh ? low_state.imu_state().quaternion()[2] : nan; // y
    imu_states_[3] = low_fresh ? low_state.imu_state().quaternion()[3] : nan; // z
    imu_states_[4] = low_fresh ? low_state.imu_state().gyroscope()[0] : nan;
    imu_states_[5] = low_fresh ? low_state.imu_state().gyroscope()[1] : nan;
    imu_states_[6] = low_fresh ? low_state.imu_state().gyroscope()[2] : nan;
    imu_states_[7] = low_fresh ? low_state.imu_state().accelerometer()[0] : nan;
    imu_states_[8] = low_fresh ? low_state.imu_state().accelerometer()[1] : nan;
    imu_states_[9] = low_fresh ? low_state.imu_state().accelerometer()[2] : nan;

    // contact states
    foot_force_[0] = low_fresh ? low_state.foot_force()[0] : nan;
    foot_force_[1] = low_fresh ? low_state.foot_force()[1] : nan;
    foot_force_[2] = low_fresh ? low_state.foot_force()[2] : nan;
    foot_force_[3] = low_fresh ? low_state.foot_force()[3] : nan;

    if (show_foot_force_)
    {
        RCLCPP_INFO(rclcpp::get_logger("unitree_hardware"), "foot_force(): %f, %f, %f, %f", foot_force_[0], foot_force_[1], foot_force_[2],
                    foot_force_[3]);
    }

    // high states
    high_states_[0] = sport_fresh ? high_state.position()[0] : nan;
    high_states_[1] = sport_fresh ? high_state.position()[1] : nan;
    high_states_[2] = sport_fresh ? high_state.position()[2] : nan;
    high_states_[3] = sport_fresh ? high_state.velocity()[0] : nan;
    high_states_[4] = sport_fresh ? high_state.velocity()[1] : nan;
    high_states_[5] = sport_fresh ? high_state.velocity()[2] : nan;

    // RCLCPP_INFO(get_logger(), "high state: %f %f %f %f %f %f", high_states_[0], high_states_[1], high_states_[2],
    //             high_states_[3], high_states_[4], high_states_[5]);

    return return_type::OK;
}

return_type HardwareUnitree::write(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
    std::lock_guard<std::mutex> lock(command_mutex_);
    if (!low_cmd_publisher_) return return_type::OK;
    // Always start from the Unitree stop sentinel for all 20 motor slots.
    // This matches Unitree Go2 low-level examples and prevents stale commands
    // on unused motor slots from carrying over between control modes.
    for (int i = 0; i < 20; ++i)
    {
        stopMotorCmd(low_cmd_.motor_cmd()[i]);
    }

    // send command
    for (int i(0); i < 12; ++i)
    {
        const int motor_idx = motor_index_map_[i];
        auto& motor_cmd = low_cmd_.motor_cmd()[motor_idx];
        const bool passive_command =
            std::abs(joint_kp_command_[i]) < CommandEps &&
            std::abs(joint_kd_command_[i]) < CommandEps &&
            std::abs(joint_torque_command_[i]) < CommandEps;

        motor_cmd.mode() = 0x01;
        if (passive_command)
        {
            stopMotorCmd(motor_cmd);
            continue;
        }

        motor_cmd.q() = static_cast<float>(joint_position_command_[i]);
        motor_cmd.dq() = static_cast<float>(joint_velocities_command_[i]);
        motor_cmd.kp() = static_cast<float>(joint_kp_command_[i]);
        motor_cmd.kd() = static_cast<float>(joint_kd_command_[i]);
        motor_cmd.tau() = static_cast<float>(joint_torque_command_[i]);
    }

    low_cmd_.crc() = crc32_core(reinterpret_cast<uint32_t*>(&low_cmd_),
                                (sizeof(unitree_go::msg::dds_::LowCmd_) >> 2) - 1);
    low_cmd_publisher_->Write(low_cmd_);
    return return_type::OK;
}

void HardwareUnitree::initLowCmd()
{
    low_cmd_.head()[0] = 0xFE;
    low_cmd_.head()[1] = 0xEF;
    low_cmd_.level_flag() = 0xFF;
    low_cmd_.gpio() = 0;

    for (int i = 0; i < 20; i++)
    {
        stopMotorCmd(low_cmd_.motor_cmd()[i]);
    }
}

void HardwareUnitree::logMotorIndexMap() const
{
    const auto joint_it = joint_interfaces.find("position");
    if (joint_it == joint_interfaces.end() || joint_it->second.size() < motor_index_map_.size())
    {
        RCLCPP_WARN(rclcpp::get_logger("unitree_hardware"),
                    "[MOTOR-MAP] Could not print full motor index map: joint list is incomplete");
        return;
    }

    for (size_t i = 0; i < motor_index_map_.size(); ++i)
    {
        RCLCPP_INFO(rclcpp::get_logger("unitree_hardware"),
                    "[MOTOR-MAP] controller[%zu] %s -> Unitree motor[%d]",
                    i, joint_it->second[i].c_str(), motor_index_map_[i]);
    }
}

void HardwareUnitree::stopMotorCmd(unitree_go::msg::dds_::MotorCmd_& motor_cmd) const
{
    motor_cmd.mode() = 0x01; // motor switch to servo (PMSM) mode
    motor_cmd.q() = PosStopF;
    motor_cmd.kp() = 0;
    motor_cmd.dq() = VelStopF;
    motor_cmd.kd() = 0;
    motor_cmd.tau() = 0;
}

bool HardwareUnitree::releaseMotionMode()
{
    motion_switcher_ = std::make_unique<unitree::robot::b2::MotionSwitcherClient>();
    motion_switcher_->SetTimeout(10.0f);
    motion_switcher_->Init();
    if (motion_switcher_->CheckMode(original_form_, original_mode_) != 0)
    {
        RCLCPP_ERROR(rclcpp::get_logger("unitree_hardware"),
                     "Cannot query native motion mode; refusing LowCmd activation");
        return false;
    }
    if (original_mode_.empty())
    {
        RCLCPP_WARN(rclcpp::get_logger("unitree_hardware"),
                    "No native motion mode was active; shutdown will not select a replacement mode");
        return true;
    }

    // A request may succeed even if its response is lost. Keep the original
    // mode until restoration has been confirmed, including activation failure.
    restore_required_ = true;
    for (int attempt = 0; attempt < 5; ++attempt)
    {
        const int32_t ret = motion_switcher_->ReleaseMode();
        std::string form, mode;
        sleep(1);
        const int32_t checked = motion_switcher_->CheckMode(form, mode);
        if (checked == 0 && mode.empty())
        {
            RCLCPP_INFO(rclcpp::get_logger("unitree_hardware"),
                        "Native mode '%s' released and confirmed", original_mode_.c_str());
            return true;
        }
        RCLCPP_WARN(rclcpp::get_logger("unitree_hardware"),
                    "Release not confirmed (%d/5): request=%d query=%d mode='%s'",
                    attempt + 1, ret, checked, mode.c_str());
    }
    RCLCPP_ERROR(rclcpp::get_logger("unitree_hardware"),
                 "Native mode release NOT confirmed; refusing LowCmd activation");
    return false;
}

bool HardwareUnitree::restoreMotionMode()
{
    if (!restore_required_) return true;
    std::string form, mode;
    int32_t checked = motion_switcher_->CheckMode(form, mode);
    if (checked != 0 || form != original_form_ || mode != original_mode_)
    {
        const int32_t ret = motion_switcher_->SelectMode(original_mode_);
        // Check actual mode even if the request response reported an error.
        checked = motion_switcher_->CheckMode(form, mode);
        if (checked != 0 || form != original_form_ || mode != original_mode_)
        {
            RCLCPP_ERROR(rclcpp::get_logger("unitree_hardware"),
                         "Native mode '%s' restoration NOT confirmed: request=%d query=%d mode='%s'. "
                         "LowCmd publication is stopped; keep robot supported and use the site manual recovery procedure",
                         original_mode_.c_str(), ret, checked, mode.c_str());
            return false;
        }
    }
    restore_required_ = false;
    RCLCPP_INFO(rclcpp::get_logger("unitree_hardware"),
                "Native mode '%s' restored and queried; physical remote takeover still requires onsite confirmation",
                original_mode_.c_str());
    return true;
}

bool HardwareUnitree::stopAndRestoreMotionMode()
{
    // Serialize with write(): no LowCmd can follow the native mode selection.
    std::lock_guard<std::mutex> lock(command_mutex_);
    bool sent = true;
    if (low_cmd_publisher_)
    {
        initLowCmd();
        low_cmd_.crc() = crc32_core(reinterpret_cast<uint32_t*>(&low_cmd_),
                                    (sizeof(unitree_go::msg::dds_::LowCmd_) >> 2) - 1);
        try
        {
            sent = low_cmd_publisher_->Write(low_cmd_);
        }
        catch (const std::exception& error)
        {
            sent = false;
            RCLCPP_ERROR(rclcpp::get_logger("unitree_hardware"),
                         "Final stop sentinel transport failed: %s", error.what());
        }
        low_cmd_publisher_->CloseChannel();
        low_cmd_publisher_.reset();
        if (!sent)
            RCLCPP_ERROR(rclcpp::get_logger("unitree_hardware"),
                         "Final stop sentinel send failed; publication closed, motor receipt NOT confirmed");
    }
    const bool restored = restoreMotionMode();
    return sent && restored;
}

void HardwareUnitree::lowStateMessageHandle(const void* messages)
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    low_state_ = *static_cast<const unitree_go::msg::dds_::LowState_*>(messages);
    low_state_received_ns_ = monotonicNowNs();
}

void HardwareUnitree::highStateMessageHandle(const void* messages)
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    high_state_ = *static_cast<const unitree_go::msg::dds_::SportModeState_*>(messages);
    sport_state_received_ns_ = monotonicNowNs();
}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    HardwareUnitree, hardware_interface::SystemInterface)
