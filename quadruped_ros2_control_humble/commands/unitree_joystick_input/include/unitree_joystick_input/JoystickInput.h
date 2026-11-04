//
// Created by tlab-uav on 24-9-13.
//
#pragma once

#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <unitree/idl/go2/WirelessController_.hpp>
#include <unitree/idl/go2/LowState_.hpp>
#include <control_input_msgs/msg/inputs.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>


typedef union
{
    struct
    {
        uint8_t R1 : 1;
        uint8_t L1 : 1;
        uint8_t start : 1;
        uint8_t select : 1;
        uint8_t R2 : 1;
        uint8_t L2 : 1;
        uint8_t F1 : 1;
        uint8_t F2 : 1;
        uint8_t A : 1;
        uint8_t B : 1;
        uint8_t X : 1;
        uint8_t Y : 1;
        uint8_t up : 1;
        uint8_t right : 1;
        uint8_t down : 1;
        uint8_t left : 1;
    } components;
    uint16_t value;
} xKeySwitchUnion;

class JoystickInput final : public rclcpp::Node {
public:
    JoystickInput();

    ~JoystickInput() override;

private:
    void remoteWirelessHandle(const void *messages);
    void lowStateRemoteHandle(const void *messages);

    std::string network_interface_ = "lo";
    int domain_ = 0;
    bool read_only_ = false;
    bool stop_only_ = false;
    std::string state_source_ = "wirelesscontroller";
    uint16_t last_logged_keys_ = 0;
    uint64_t receive_count_ = 0;
    std::chrono::steady_clock::time_point last_log_{};

    unitree_go::msg::dds_::WirelessController_ wireless_controller_{};
    xKeySwitchUnion xKeySwitchUnion_{};

    control_input_msgs::msg::Inputs inputs_;
    rclcpp::Publisher<control_input_msgs::msg::Inputs>::SharedPtr publisher_;
    unitree::robot::ChannelSubscriberPtr<unitree_go::msg::dds_::WirelessController_> subscriber_;
    unitree::robot::ChannelSubscriberPtr<unitree_go::msg::dds_::LowState_> low_state_subscriber_;
};
