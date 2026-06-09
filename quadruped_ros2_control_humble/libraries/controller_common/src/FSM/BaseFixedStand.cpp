//
// Created by biao on 24-9-10.
//

#include "controller_common/FSM/BaseFixedStand.h"

#include <cmath>
#include <rclcpp/logging.hpp>

BaseFixedStand::BaseFixedStand(CtrlInterfaces& ctrl_interfaces, const std::vector<double>& target_pos,
                               const double kp,
                               const double kd)
    : FSMState(FSMStateName::FIXEDSTAND, "fixed stand", ctrl_interfaces),
      kp_(kp), kd_(kd)
{
    duration_ = ctrl_interfaces_.frequency_ * 1.2;
    for (int i = 0; i < 12; i++)
    {
        target_pos_[i] = target_pos[i];
    }
}

void BaseFixedStand::enter()
{
    for (int i = 0; i < 12; i++)
    {
        start_pos_[i] = ctrl_interfaces_.joint_position_state_interface_[i].get().get_value();
    }
    for (int i = 0; i < 12; i++)
    {
        ctrl_interfaces_.joint_position_command_interface_[i].get().set_value(start_pos_[i]);
        ctrl_interfaces_.joint_velocity_command_interface_[i].get().set_value(0);
        ctrl_interfaces_.joint_torque_command_interface_[i].get().set_value(0);
        ctrl_interfaces_.joint_kp_command_interface_[i].get().set_value(kp_);
        ctrl_interfaces_.joint_kd_command_interface_[i].get().set_value(kd_);
    }
    ctrl_interfaces_.control_inputs_.command = 0;
}

void BaseFixedStand::run(const rclcpp::Time&/*time*/, const rclcpp::Duration&/*period*/)
{
    percent_ += 1 / duration_;
    phase = std::tanh(percent_);
    for (int i = 0; i < 12; i++)
    {
        ctrl_interfaces_.joint_position_command_interface_[i].get().set_value(
            phase * target_pos_[i] + (1 - phase) * start_pos_[i]);
    }

    static int stand_symm_count = 0;
    if (phase < 1.0 && stand_symm_count++ % 100 == 0)
    {
        const double fr_thigh_q = ctrl_interfaces_.joint_position_state_interface_[1].get().get_value();
        const double fl_thigh_q = ctrl_interfaces_.joint_position_state_interface_[4].get().get_value();
        const double rr_thigh_q = ctrl_interfaces_.joint_position_state_interface_[7].get().get_value();
        const double rl_thigh_q = ctrl_interfaces_.joint_position_state_interface_[10].get().get_value();
        const double fr_calf_q = ctrl_interfaces_.joint_position_state_interface_[2].get().get_value();
        const double fl_calf_q = ctrl_interfaces_.joint_position_state_interface_[5].get().get_value();
        const double rr_calf_q = ctrl_interfaces_.joint_position_state_interface_[8].get().get_value();
        const double rl_calf_q = ctrl_interfaces_.joint_position_state_interface_[11].get().get_value();

        RCLCPP_INFO(rclcpp::get_logger("BaseFixedStand"),
            "[STAND-SYMM] phase=%.3f q_thigh FR=%.3f FL=%.3f RR=%.3f RL=%.3f diff_FLFR=%.3f diff_RLRR=%.3f "
            "q_calf FR=%.3f FL=%.3f RR=%.3f RL=%.3f diff_FLFR=%.3f diff_RLRR=%.3f",
            phase,
            fr_thigh_q, fl_thigh_q, rr_thigh_q, rl_thigh_q, fl_thigh_q - fr_thigh_q, rl_thigh_q - rr_thigh_q,
            fr_calf_q, fl_calf_q, rr_calf_q, rl_calf_q, fl_calf_q - fr_calf_q, rl_calf_q - rr_calf_q);
    }
}

void BaseFixedStand::exit()
{
    percent_ = 0;
}

FSMStateName BaseFixedStand::checkChange()
{
    if (percent_ < 1.5)
    {
        return FSMStateName::FIXEDSTAND;
    }
    switch (ctrl_interfaces_.control_inputs_.command)
    {
    case 1:
        return FSMStateName::PASSIVE;
    case 2:
        return FSMStateName::FIXEDDOWN;
    default:
        return FSMStateName::FIXEDSTAND;
    }
}
