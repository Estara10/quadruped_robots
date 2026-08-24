#include "rl_quadruped_controller/FSM/AbsObservationContract.h"

namespace abs_observation {
namespace {
torch::Tensor select(const torch::Tensor& value, const std::vector<int64_t>& indices, bool enabled) {
  if (!enabled) return value;
  return value.index_select(1, torch::tensor(indices, torch::TensorOptions().dtype(torch::kLong).device(value.device())));
}
torch::Tensor joints(const Input& in, const Scale& s, bool order) {
  return (controllerToPolicyDof(in.dof_pos, order) - controllerToPolicyDof(in.default_dof_pos, order) - in.dof_bias) * s.dof_pos;
}
torch::Tensor clipped(const std::vector<torch::Tensor>& fields, double clip) {
  return torch::clamp(torch::cat(fields, 1), -clip, clip);
}
}
torch::Tensor controllerToPolicyDof(const torch::Tensor& v, bool e) { return select(v, {3,4,5,0,1,2,9,10,11,6,7,8}, e); }
torch::Tensor policyToControllerDof(const torch::Tensor& v, bool e) { return controllerToPolicyDof(v, e); }
torch::Tensor controllerToPolicyContact(const torch::Tensor& v, bool e) { return select(v, {1,0,3,2}, e); }
torch::Tensor agile(const Input& in, const Scale& s, bool order) {
  return clipped({controllerToPolicyContact(in.contact, order), in.ang_vel*s.ang_vel, in.gravity_body, in.commands, in.timer,
                  joints(in,s,order), controllerToPolicyDof(in.dof_vel,order)*s.dof_vel, controllerToPolicyDof(in.actions,order), in.ray2d}, s.clip);
}
torch::Tensor recovery(const Input& in, const Scale& s, bool order) {
  return clipped({controllerToPolicyContact(in.contact, order), in.ang_vel*s.ang_vel, in.gravity_body, in.commands,
                  joints(in,s,order), controllerToPolicyDof(in.dof_vel,order)*s.dof_vel, controllerToPolicyDof(in.actions,order)}, s.clip);
}
torch::Tensor ra(const Input& in) {
  return torch::cat({in.lin_vel, in.ang_vel,
                     in.commands.index({torch::indexing::Slice(), torch::indexing::Slice(0,2)}), in.ray2d}, 1);
}
bool finite(const torch::Tensor& v) { return torch::isfinite(v).all().item<bool>(); }
}  // namespace abs_observation
