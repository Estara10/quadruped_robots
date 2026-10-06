//
// Created by biao on 24-9-9.
//


#ifndef HARDWAREUNITREE_H
#define HARDWAREUNITREE_H

#include "hardware_interface/system_interface.hpp"

#include <array>
#include <memory>

#include <unitree/idl/go2/WirelessController_.hpp>
#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/idl/go2/LowCmd_.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/robot/b2/motion_switcher/motion_switcher_client.hpp>

class HardwareUnitree final : public hardware_interface::SystemInterface {
public:
    HardwareUnitree() = default;
    ~HardwareUnitree() override;

    CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;

    CallbackReturn on_shutdown(const rclcpp_lifecycle::State &previous_state) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;

    hardware_interface::return_type write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override;

protected:
    std::vector<double> joint_torque_command_;
    std::vector<double> joint_position_command_;
    std::vector<double> joint_velocities_command_;
    std::vector<double> joint_kp_command_;
    std::vector<double> joint_kd_command_;

    std::vector<double> joint_position_;
    std::vector<double> joint_velocities_;
    std::vector<double> joint_effort_;

    std::vector<double> imu_states_;
    std::vector<double> foot_force_;
    std::vector<double> high_states_;

    // Controller joint order: FR, FL, RR, RL; each leg: hip, thigh, calf.
    // Unitree Go2 LowState/LowCmd motor index order is expected to match this.
    std::array<int, 12> motor_index_map_{0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};

    std::unordered_map<std::string, std::vector<std::string> > joint_interfaces = {
        {"position", {}},
        {"velocity", {}},
        {"effort", {}}
    };


    void initLowCmd();

    void logMotorIndexMap() const;

    void stopMotorCmd(unitree_go::msg::dds_::MotorCmd_& motor_cmd) const;

    void releaseMotionMode();
    void restoreMotionMode();

    int queryMotionStatus(std::string& service_name);

    std::string queryServiceName(const std::string& form, const std::string& name) const;

    void lowStateMessageHandle(const void *messages);

    void highStateMessageHandle(const void *messages);

    void remoteWirelessHandle(const void *messages);

    unitree_go::msg::dds_::LowCmd_ low_cmd_{}; // default init
    unitree_go::msg::dds_::LowState_ low_state_{}; // default init
    unitree_go::msg::dds_::SportModeState_ high_state_{}; // default init
    std::unique_ptr<unitree::robot::b2::MotionSwitcherClient> motion_switcher_;

    std::string network_interface_ = "lo";
    int domain_ = 1;
    bool show_foot_force_ = false;

    /*publisher*/
    unitree::robot::ChannelPublisherPtr<unitree_go::msg::dds_::LowCmd_> low_cmd_publisher_;
    /*subscriber*/
    unitree::robot::ChannelSubscriberPtr<unitree_go::msg::dds_::LowState_> lows_tate_subscriber_;
    unitree::robot::ChannelSubscriberPtr<unitree_go::msg::dds_::SportModeState_> high_state_subscriber_;
};

#endif //HARDWAREUNITREE_H
