//
// Created by tlab-uav on 24-10-4.
//

#include "RlQuadrupedController.h"

#include <algorithm>
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
        if (ctrl_interfaces_.control_inputs_.command == 1 || ctrl_interfaces_.control_inputs_.command == 9)
        {
            if (current_state_ && current_state_->state_name != FSMStateName::PASSIVE)
            {
                RCLCPP_ERROR(get_node()->get_logger(),
                             "[HARD-STOP] command=%d -> forcing PASSIVE",
                             ctrl_interfaces_.control_inputs_.command);
                current_state_->exit();
                current_state_ = state_list_.passive;
                current_state_->enter();
                next_state_ = current_state_;
                next_state_name_ = current_state_->state_name;
                mode_ = FSMMode::NORMAL;
            }
            ctrl_interfaces_.control_inputs_.command = 0;
            return controller_interface::return_type::OK;
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
                mode_ = FSMMode::CHANGE;
                next_state_ = getNextState(next_state_name_);
                RCLCPP_INFO(get_node()->get_logger(), "Switched from %s to %s",
                            current_state_->state_name_string.c_str(), next_state_->state_name_string.c_str());
            }
        }
        else if (mode_ == FSMMode::CHANGE)
        {
            current_state_->exit();
            current_state_ = next_state_;

            current_state_->enter();
            mode_ = FSMMode::NORMAL;
        }

        return controller_interface::return_type::OK;
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

        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn LeggedGymController::on_activate(
        const rclcpp_lifecycle::State& /*previous_state*/)
    {
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
        for (auto& interface : state_interfaces_)
        {
            if (interface.get_prefix_name() == imu_name_)
            {
                ctrl_interfaces_.imu_state_interface_.emplace_back(interface);
            }
            else if (interface.get_prefix_name() == foot_force_name_)
            {
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
        state_list_.rlRec = std::make_shared<StateRLRec>(ctrl_interfaces_, ctrl_component_, stand_pos_);

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
