//
// Created by tlab-uav on 24-9-13.
//

#include "unitree_joystick_input/JoystickInput.h"
#include <unitree/dds_wrapper/common/unitree_joystick.hpp>
#include <cstring>
#include <stdexcept>
#define TOPIC_JOYSTICK "rt/wirelesscontroller"

using std::placeholders::_1;

using namespace unitree::robot;

JoystickInput::JoystickInput() : Node("unitree_joysick_node")
{
    declare_parameter("network_interface", network_interface_);
    declare_parameter("domain", domain_);
    declare_parameter("read_only", read_only_);
    declare_parameter("stop_only", stop_only_);
    declare_parameter("state_source", state_source_);

    network_interface_ = get_parameter("network_interface").as_string();
    domain_ = get_parameter("domain").as_int();
    read_only_ = get_parameter("read_only").as_bool();
    stop_only_ = get_parameter("stop_only").as_bool();
    state_source_ = get_parameter("state_source").as_string();
    if (state_source_ != "lowstate" && state_source_ != "wirelesscontroller")
        throw std::invalid_argument("state_source must be lowstate or wirelesscontroller");
    if (!read_only_)
        publisher_ = create_publisher<control_input_msgs::msg::Inputs>("control_input", 10);
    RCLCPP_INFO(get_logger(), " network_interface: %s, domain: %d", network_interface_.c_str(), domain_);
    RCLCPP_INFO(get_logger(), "read_only=%d stop_only=%d state_source=%s; read_only never publishes control_input",
                read_only_, stop_only_, state_source_.c_str());
    ChannelFactory::Instance()->Init(domain_, network_interface_);

    if (state_source_ == "lowstate")
    {
        low_state_subscriber_ = std::make_shared<ChannelSubscriber<unitree_go::msg::dds_::LowState_>>("rt/lowstate");
        low_state_subscriber_->InitChannel([this](const void* message) { lowStateRemoteHandle(message); }, 1);
    }
    else
    {
        subscriber_ = std::make_shared<ChannelSubscriber<unitree_go::msg::dds_::WirelessController_>>(
            TOPIC_JOYSTICK);
        subscriber_->InitChannel(
            [this](auto&& PH1)
            {
                remoteWirelessHandle(std::forward<decltype(PH1)>(PH1));
            },
            1);
    }
}

JoystickInput::~JoystickInput()
{
    if (subscriber_) subscriber_->CloseChannel();
    if (low_state_subscriber_) low_state_subscriber_->CloseChannel();
    ChannelFactory::Instance()->Release();
}

void JoystickInput::lowStateRemoteHandle(const void* messages)
{
    const auto& raw = static_cast<const unitree_go::msg::dds_::LowState_*>(messages)->wireless_remote();
    unitree::common::REMOTE_DATA_RX remote{};
    static_assert(sizeof(remote.buff) == 40, "Go2 remote payload must be 40 bytes");
    std::memcpy(remote.buff, raw.data(), raw.size());
    unitree_go::msg::dds_::WirelessController_ decoded;
    decoded.keys(remote.RF_RX.btn.value);
    decoded.lx(remote.RF_RX.lx);
    decoded.ly(remote.RF_RX.ly);
    decoded.rx(remote.RF_RX.rx);
    decoded.ry(remote.RF_RX.ry);
    remoteWirelessHandle(&decoded);
}

void JoystickInput::remoteWirelessHandle(const void* messages)
{
    wireless_controller_ = *static_cast<const unitree_go::msg::dds_::WirelessController_*>(messages);
    xKeySwitchUnion_.value = wireless_controller_.keys();
    // Clear old axes on every packet, especially all stop/mode commands.
    inputs_ = control_input_msgs::msg::Inputs{};

    if (xKeySwitchUnion_.components.L2 && xKeySwitchUnion_.components.B)
    {
        inputs_.command = 9;
    }
    else if (xKeySwitchUnion_.components.select)
    {
        inputs_.command = 1;
    }
    else if (xKeySwitchUnion_.components.left && xKeySwitchUnion_.components.X)
    {
        inputs_.command = 9;
    }
    else if (stop_only_)
    {
        // Real commissioning: no stand/RL commands or joystick axes yet.
        inputs_.command = 0;
    }
    else if (xKeySwitchUnion_.components.start)
    {
        inputs_.command = 2;
    }
    else if (xKeySwitchUnion_.components.right and xKeySwitchUnion_.components.B)
    {
        inputs_.command = 3;
    }
    else if (xKeySwitchUnion_.components.right and xKeySwitchUnion_.components.A)
    {
        inputs_.command = 4;
    }
    else if (xKeySwitchUnion_.components.right and xKeySwitchUnion_.components.X)
    {
        inputs_.command = 5;
    }
    else if (xKeySwitchUnion_.components.right and xKeySwitchUnion_.components.Y)
    {
        inputs_.command = 6;
    }
    else if (xKeySwitchUnion_.components.left and xKeySwitchUnion_.components.B)
    {
        inputs_.command = 7;
    }
    else if (xKeySwitchUnion_.components.left and xKeySwitchUnion_.components.A)
    {
        inputs_.command = 8;
    }
    else if (xKeySwitchUnion_.components.left and xKeySwitchUnion_.components.Y)
    {
        inputs_.command = 10;
    }
    else
    {
        inputs_.command = 0;
        inputs_.lx = wireless_controller_.lx();
        inputs_.ly = wireless_controller_.ly();
        inputs_.rx = wireless_controller_.rx();
        inputs_.ry = wireless_controller_.ry();
    }
    if (read_only_)
    {
        const auto now = std::chrono::steady_clock::now();
        ++receive_count_;
        if (receive_count_ == 1 || wireless_controller_.keys() != last_logged_keys_ ||
            now - last_log_ >= std::chrono::seconds(1))
        {
            RCLCPP_INFO(get_logger(),
                        "[REMOTE-READONLY] source=%s count=%lu keys=0x%04x L2=%u B=%u select=%u start=%u "
                        "mapped_command=%d publish=0 axes=[%.3f,%.3f,%.3f,%.3f]",
                        state_source_.c_str(), static_cast<unsigned long>(receive_count_),
                        static_cast<unsigned>(wireless_controller_.keys()),
                        static_cast<unsigned>(xKeySwitchUnion_.components.L2),
                        static_cast<unsigned>(xKeySwitchUnion_.components.B),
                        static_cast<unsigned>(xKeySwitchUnion_.components.select),
                        static_cast<unsigned>(xKeySwitchUnion_.components.start),
                        inputs_.command, wireless_controller_.lx(), wireless_controller_.ly(),
                        wireless_controller_.rx(), wireless_controller_.ry());
            last_logged_keys_ = wireless_controller_.keys();
            last_log_ = now;
        }
        return;
    }
    publisher_->publish(inputs_);
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JoystickInput>();
    spin(node);
    rclcpp::shutdown();
    return 0;
}
