//
// Created by biao on 24-9-10.
//

#ifndef CtrlComponent_H
#define CtrlComponent_H

#include "Estimator.h"
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <torch/script.h>

struct CtrlComponent {

    bool enable_estimator_ = false;
    std::shared_ptr<QuadrupedRobot> robot_model_;
    std::shared_ptr<Estimator> estimator_;
    std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;

    // Shared RA model for recovery twist optimization + safe-return check
    torch::jit::script::Module* ra_model = nullptr;
    bool ra_loaded = false;
    double ra_threshold = -0.05;

    // Optimized recovery twist
    double recovery_twist_vx = 0.0;
    double recovery_twist_vy = 0.0;
    double recovery_twist_wz = 0.0;
    int recovery_steps = 250;  // auto-return after this many steps

    CtrlComponent() = default;
};

#endif //CtrlComponent_H
