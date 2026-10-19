#include "rl_quadruped_controller/FSM/AbsObservationContract.h"

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
torch::Tensor row(const std::vector<float>& values) {
  return torch::tensor(values).reshape({1, static_cast<int64_t>(values.size())});
}

void expect(const std::string& label, const torch::Tensor& actual,
            const std::vector<float>& expected) {
  const auto values = actual.flatten().contiguous();
  if (values.numel() != static_cast<int64_t>(expected.size()))
    throw std::runtime_error(label + ": dimension mismatch");
  for (int64_t i = 0; i < values.numel(); ++i) {
    const float value = values[i].item<float>();
    if (std::abs(value - expected[static_cast<size_t>(i)]) > 1e-5f)
      throw std::runtime_error(label + ": mismatch at slot " + std::to_string(i));
  }
  std::cout << label << " PASS\n";
}

std::vector<float> range(float base, int n) {
  std::vector<float> values;
  for (int i = 0; i < n; ++i) values.push_back(base + static_cast<float>(i));
  return values;
}

std::vector<float> permute(const std::vector<float>& values,
                           const std::vector<int>& indices) {
  std::vector<float> result;
  for (int i : indices) result.push_back(values.at(static_cast<size_t>(i)));
  return result;
}

void append(std::vector<float>& into, const std::vector<float>& values) {
  into.insert(into.end(), values.begin(), values.end());
}
}  // namespace

int main() {
  // Controller vector slots are FR, FL, RR, RL; each leg hip, thigh, calf.
  const std::vector<int> dof_indices{3,4,5,0,1,2,9,10,11,6,7,8};
  const std::vector<int> contact_indices{1,0,3,2};
  const auto q = range(100, 12);
  const auto q0 = range(200, 12);
  const auto bias = range(300, 12);
  const auto dq = range(400, 12);
  const auto prior_action = range(500, 12);
  const auto policy_action = range(700, 12);
  const auto contacts = range(800, 4);

  const auto q_policy = permute(q, dof_indices);
  const auto q0_policy = permute(q0, dof_indices);
  const auto bias_policy = permute(bias, dof_indices);
  const auto dq_policy = permute(dq, dof_indices);
  const auto prior_policy = permute(prior_action, dof_indices);
  const auto contact_policy = permute(contacts, contact_indices);

  expect("controllerToPolicyDof q", abs_observation::controllerToPolicyDof(row(q), true), q_policy);
  expect("controllerToPolicyDof default", abs_observation::controllerToPolicyDof(row(q0), true), q0_policy);
  expect("controllerToPolicyDof bias", abs_observation::controllerToPolicyDof(row(bias), true), bias_policy);
  expect("controllerToPolicyDof dq", abs_observation::controllerToPolicyDof(row(dq), true), dq_policy);
  expect("controllerToPolicyDof previous action", abs_observation::controllerToPolicyDof(row(prior_action), true), prior_policy);
  expect("controllerToPolicyContact", abs_observation::controllerToPolicyContact(row(contacts), true), contact_policy);

  const auto ctrl_action = permute(policy_action, dof_indices);
  expect("policyToControllerDof Agile output", abs_observation::policyToControllerDof(row(policy_action), true), ctrl_action);
  expect("policyToControllerDof Recovery output", abs_observation::policyToControllerDof(row(policy_action), true), ctrl_action);
  expect("previous action round trip",
         abs_observation::policyToControllerDof(abs_observation::controllerToPolicyDof(row(prior_action), true), true),
         prior_action);

  abs_observation::Input input{
      row({1,2,3}), row(contacts), row({4,5,6}), row({7,8,9}), row({10,11,12}), row({13}),
      row(q), row(q0), row(bias_policy), row(dq), row(prior_action), row({14,15})};
  const abs_observation::Scale scale{1.0, 1.0, 0.2, 1e6};
  std::vector<float> expected_agile;
  std::vector<float> dq_scaled;
  for (float value : dq_policy) dq_scaled.push_back(value * 0.2f);
  append(expected_agile, contact_policy); append(expected_agile, {4,5,6});
  append(expected_agile, {7,8,9}); append(expected_agile, {10,11,12}); append(expected_agile, {13});
  std::vector<float> q_delta;
  for (size_t i = 0; i < q_policy.size(); ++i)
    q_delta.push_back(q_policy[i] - q0_policy[i] - bias_policy[i]);
  append(expected_agile, q_delta); append(expected_agile, dq_scaled);
  append(expected_agile, prior_policy); append(expected_agile, {14,15});

  auto expected_recovery = expected_agile;
  // Recovery omits Agile timer and ray fields.
  expected_recovery.clear();
  append(expected_recovery, contact_policy); append(expected_recovery, {4,5,6});
  append(expected_recovery, {7,8,9}); append(expected_recovery, {10,11,12});
  append(expected_recovery, q_delta); append(expected_recovery, dq_scaled);
  append(expected_recovery, prior_policy);

  expect("Agile observation field layout", abs_observation::agile(input, scale, true), expected_agile);
  expect("inline Recovery observation field layout", abs_observation::recovery(input, scale, true), expected_recovery);

  std::cout << "policy FL/FR/RL/RR channel i -> controller slot "
            << "3/4/5,0/1/2,9/10/11,6/7/8 PASS\n";
  std::cout << "foot contact controller FR/FL/RR/RL -> policy FL/FR/RL/RR = 1/0/3/2 PASS\n";
}
