#pragma once

#include <torch/script.h>

#include <cstdint>
#include <string>
#include <vector>

// Shared by the deployed ABS states and the P1-01 test adapter.  This is an
// interface contract only: it does not choose goals, contacts, rays or safety
// fallbacks.  All inputs must already be in their documented runtime frame.
namespace abs_observation {

struct Input {
  torch::Tensor lin_vel;          // body frame m/s (RA only)
  torch::Tensor contact;          // controller order, encoded {-1,+1}
  torch::Tensor ang_vel;          // body frame rad/s
  torch::Tensor gravity_body;     // world gravity expressed in body frame
  torch::Tensor commands;         // deployment command representation
  torch::Tensor timer;            // normalized deployment timer
  torch::Tensor dof_pos;          // controller order
  torch::Tensor default_dof_pos;  // controller order
  torch::Tensor dof_bias;         // policy order; zero when unavailable
  torch::Tensor dof_vel;          // controller order
  torch::Tensor actions;          // controller order
  torch::Tensor ray2d;            // 11 log2-distance values
};

struct Scale { double ang_vel{1.0}; double dof_pos{1.0}; double dof_vel{0.2}; double clip{100.0}; };

torch::Tensor controllerToPolicyDof(const torch::Tensor& value, bool ros1_policy_order);
torch::Tensor controllerToPolicyContact(const torch::Tensor& value, bool ros1_policy_order);
torch::Tensor policyToControllerDof(const torch::Tensor& value, bool ros1_policy_order);
torch::Tensor agile(const Input& in, const Scale& scale, bool ros1_policy_order);
torch::Tensor recovery(const Input& in, const Scale& scale, bool ros1_policy_order);
torch::Tensor ra(const Input& in);
bool finite(const torch::Tensor& value);
double rollingTimeLeftNormalized(double elapsed_s, double horizon_s);
torch::Tensor temporalContact(const torch::Tensor& current, torch::Tensor& previous);
bool rayFrameValid(const float* rays, int count, uint64_t stamp_magic, uint64_t stamp_ns,
                   uint64_t now_ns, uint64_t timeout_ns);

}  // namespace abs_observation
