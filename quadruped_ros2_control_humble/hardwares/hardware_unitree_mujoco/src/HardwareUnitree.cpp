//
// Created by biao on 24-9-9.
//

#include "hardware_unitree_mujoco/HardwareUnitree.h"

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

#include <cmath>
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

    RCLCPP_INFO(rclcpp::get_logger("unitree_hardware"), " network_interface: %s, domain: %d", network_interface_.c_str(), domain_);
    ChannelFactory::Instance()->Init(domain_, network_interface_);

    low_cmd_publisher_ =
        std::make_shared<ChannelPublisher<unitree_go::msg::dds_::LowCmd_>>(
            TOPIC_LOWCMD);
    low_cmd_publisher_->InitChannel();

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

    if (network_interface_ != "lo")
    {
        releaseMotionMode();
    }


    return SystemInterface::on_init(info);
}

HardwareUnitree::~HardwareUnitree()
{
    if (network_interface_ != "lo" && motion_switcher_)
    {
        restoreMotionMode();
    }
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn HardwareUnitree::on_shutdown(
    const rclcpp_lifecycle::State& /* previous_state */)
{
    if (network_interface_ != "lo" && motion_switcher_)
    {
        restoreMotionMode();
    }
    return CallbackReturn::SUCCESS;
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
    // joint states
    for (int i(0); i < 12; ++i)
    {
        const int motor_idx = motor_index_map_[i];
        joint_position_[i] = low_state_.motor_state()[motor_idx].q();
        joint_velocities_[i] = low_state_.motor_state()[motor_idx].dq();
        joint_effort_[i] = low_state_.motor_state()[motor_idx].tau_est();
    }

    // imu states
    imu_states_[0] = low_state_.imu_state().quaternion()[0]; // w
    imu_states_[1] = low_state_.imu_state().quaternion()[1]; // x
    imu_states_[2] = low_state_.imu_state().quaternion()[2]; // y
    imu_states_[3] = low_state_.imu_state().quaternion()[3]; // z
    imu_states_[4] = low_state_.imu_state().gyroscope()[0];
    imu_states_[5] = low_state_.imu_state().gyroscope()[1];
    imu_states_[6] = low_state_.imu_state().gyroscope()[2];
    imu_states_[7] = low_state_.imu_state().accelerometer()[0];
    imu_states_[8] = low_state_.imu_state().accelerometer()[1];
    imu_states_[9] = low_state_.imu_state().accelerometer()[2];

    // contact states
    foot_force_[0] = low_state_.foot_force()[0];
    foot_force_[1] = low_state_.foot_force()[1];
    foot_force_[2] = low_state_.foot_force()[2];
    foot_force_[3] = low_state_.foot_force()[3];

    if (show_foot_force_)
    {
        RCLCPP_INFO(rclcpp::get_logger("unitree_hardware"), "foot_force(): %f, %f, %f, %f", foot_force_[0], foot_force_[1], foot_force_[2],
                    foot_force_[3]);
    }

    // high states
    high_states_[0] = high_state_.position()[0];
    high_states_[1] = high_state_.position()[1];
    high_states_[2] = high_state_.position()[2];
    high_states_[3] = high_state_.velocity()[0];
    high_states_[4] = high_state_.velocity()[1];
    high_states_[5] = high_state_.velocity()[2];

    // RCLCPP_INFO(get_logger(), "high state: %f %f %f %f %f %f", high_states_[0], high_states_[1], high_states_[2],
    //             high_states_[3], high_states_[4], high_states_[5]);

    return return_type::OK;
}

return_type HardwareUnitree::write(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
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

void HardwareUnitree::releaseMotionMode()
{
    motion_switcher_ = std::make_unique<unitree::robot::b2::MotionSwitcherClient>();
    motion_switcher_->SetTimeout(10.0f);
    motion_switcher_->Init();

    for (int attempt = 0; attempt < 5; ++attempt)
    {
        std::string service_name;
        const int motion_status = queryMotionStatus(service_name);
        if (motion_status == 0)
        {
            RCLCPP_INFO(rclcpp::get_logger("unitree_hardware"),
                        "Motion service is already deactivated");
            return;
        }
        if (motion_status < 0)
        {
            RCLCPP_WARN(rclcpp::get_logger("unitree_hardware"),
                        "Could not query motion service status; continuing with LowCmd setup");
            return;
        }

        RCLCPP_WARN(rclcpp::get_logger("unitree_hardware"),
                    "Motion service '%s' is active; releasing before LowCmd control (attempt %d/5)",
                    service_name.c_str(), attempt + 1);
        const int32_t ret = motion_switcher_->ReleaseMode();
        if (ret != 0)
        {
            RCLCPP_WARN(rclcpp::get_logger("unitree_hardware"),
                        "ReleaseMode failed with code %d", ret);
        }
        sleep(1);
    }

    std::string service_name;
    if (queryMotionStatus(service_name) > 0)
    {
        RCLCPP_ERROR(rclcpp::get_logger("unitree_hardware"),
                     "Motion service '%s' is still active after ReleaseMode attempts; LowCmd may fight sport_mode",
                     service_name.c_str());
    }
}

void HardwareUnitree::restoreMotionMode()
{
    if (!motion_switcher_)
    {
        return;
    }

    // Re-enable sport_mode so the remote controller can take over after our
    // LowCmd control is stopped.
    RCLCPP_INFO(rclcpp::get_logger("unitree_hardware"),
                "Restoring native sport_mode for remote-controller takeover");
    const int32_t ret = motion_switcher_->SelectMode("normal");
    if (ret != 0)
    {
        RCLCPP_WARN(rclcpp::get_logger("unitree_hardware"),
                    "SelectMode(\"normal\") failed with code %d; the remote controller may not work until the robot is restarted",
                    ret);
    }
}

int HardwareUnitree::queryMotionStatus(std::string& service_name)
{
    if (!motion_switcher_)
    {
        RCLCPP_WARN(rclcpp::get_logger("unitree_hardware"),
                    "Motion switcher is not initialized");
        return -1;
    }

    std::string robot_form;
    std::string motion_name;
    const int32_t ret = motion_switcher_->CheckMode(robot_form, motion_name);
    if (ret != 0)
    {
        RCLCPP_WARN(rclcpp::get_logger("unitree_hardware"),
                    "CheckMode failed with code %d", ret);
        return -1;
    }

    if (motion_name.empty())
    {
        service_name.clear();
        return 0;
    }

    service_name = queryServiceName(robot_form, motion_name);
    if (service_name.empty())
    {
        service_name = robot_form + ":" + motion_name;
    }
    return 1;
}

std::string HardwareUnitree::queryServiceName(const std::string& form, const std::string& name) const
{
    if (form == "0")
    {
        if (name == "normal") return "sport_mode";
        if (name == "ai") return "ai_sport";
        if (name == "advanced") return "advanced_sport";
    }
    else
    {
        if (name == "ai-w") return "wheeled_sport(go2W)";
        if (name == "normal-w") return "wheeled_sport(b2W)";
    }
    return "";
}

void HardwareUnitree::lowStateMessageHandle(const void* messages)
{
    low_state_ = *static_cast<const unitree_go::msg::dds_::LowState_*>(messages);
}

void HardwareUnitree::highStateMessageHandle(const void* messages)
{
    high_state_ = *static_cast<const unitree_go::msg::dds_::SportModeState_*>(messages);
}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    HardwareUnitree, hardware_interface::SystemInterface)
