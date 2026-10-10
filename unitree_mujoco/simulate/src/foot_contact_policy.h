#pragma once

#include <mujoco/mujoco.h>

#include <array>
#include <cmath>
#include <cstring>
#include <string>

namespace abs_foot {

constexpr std::array<const char*, 4> kLegs{{"FL", "FR", "RL", "RR"}};
constexpr double kImpactVerticalMultiplier = 2.0;
constexpr double kImpactOffsetN = 10.0;

struct FootGeomSet {
  std::array<int, 4> geom_ids{{-1, -1, -1, -1}};
  bool valid = false;
};

inline bool resolveFootGeoms(const mjModel* model, FootGeomSet* result,
                             std::string* error = nullptr) {
  if (result == nullptr) return false;
  *result = FootGeomSet{};
  auto fail = [&](const std::string& why) {
    if (error != nullptr) *error = why;
    result->valid = false;
    return false;
  };
  if (model == nullptr) return fail("null MuJoCo model");
  for (std::size_t i = 0; i < kLegs.size(); ++i) {
    const int geom = mj_name2id(model, mjOBJ_GEOM, kLegs[i]);
    if (geom < 0) return fail(std::string("missing named foot geom: ") + kLegs[i]);
    const char* actual_geom_name = mj_id2name(model, mjOBJ_GEOM, geom);
    const char* actual_body_name = mj_id2name(model, mjOBJ_BODY, model->geom_bodyid[geom]);
    if (actual_geom_name == nullptr || std::strcmp(actual_geom_name, kLegs[i]) != 0 ||
        model->geom_bodyid[geom] < 0 || model->geom_bodyid[geom] >= model->nbody ||
        actual_body_name == nullptr ||
        model->geom_type[geom] != mjGEOM_SPHERE || model->geom_group[geom] != 3) {
      return fail(std::string("foot geom is not the named group-3 sphere with a valid owning body: ") + kLegs[i]);
    }
    result->geom_ids[i] = geom;
  }
  result->valid = true;
  return true;
}

inline int footIndex(const FootGeomSet& feet, int geom_id) {
  for (std::size_t i = 0; i < feet.geom_ids.size(); ++i) {
    if (feet.geom_ids[i] == geom_id) return static_cast<int>(i);
  }
  return -1;
}

inline bool isTerminalRobotObstacleContact(bool is_exact_foot_geom) {
  return !is_exact_foot_geom;
}

inline bool accumulateWorldForce(std::array<double, 3>* sum,
                                 const std::array<double, 3>& force) {
  if (sum == nullptr) return false;
  for (int axis = 0; axis < 3; ++axis) {
    if (!std::isfinite(force[axis]) || !std::isfinite((*sum)[axis] + force[axis])) return false;
  }
  for (int axis = 0; axis < 3; ++axis) (*sum)[axis] += force[axis];
  return true;
}

// mj_contactForce returns the force on geom[1], in contact coordinates.
// mjContact.frame stores the contact axes in rows; the normal (row 0) points
// from geom[0] to geom[1]. Return the force on the foot in world coordinates.
inline bool contactForceOnFootWorld(const mjContact& contact,
                                    const mjtNum force_torque[6],
                                    bool foot_is_geom1,
                                    std::array<double, 3>* world_force) {
  if (world_force == nullptr || contact.efc_address < 0) return false;
  const double sign = foot_is_geom1 ? -1.0 : 1.0;
  std::array<double, 3> value{};
  for (int axis = 0; axis < 3; ++axis) {
    value[axis] = sign * (static_cast<double>(force_torque[0]) * contact.frame[axis] +
                          static_cast<double>(force_torque[1]) * contact.frame[3 + axis] +
                          static_cast<double>(force_torque[2]) * contact.frame[6 + axis]);
    if (!std::isfinite(value[axis])) return false;
  }
  *world_force = value;
  return true;
}

struct ImpactDecision {
  bool known = false;
  bool impact = false;
  double fxy = 0.0;
  double abs_fz = 0.0;
  double threshold = 0.0;
};

inline ImpactDecision classifyImpact(const std::array<double, 3>& world_force,
                                     bool has_foot_obstacle_contact,
                                     bool all_external_forces_known) {
  ImpactDecision out{};
  if (!has_foot_obstacle_contact || !all_external_forces_known) return out;
  if (!std::isfinite(world_force[0]) || !std::isfinite(world_force[1]) ||
      !std::isfinite(world_force[2])) return out;
  out.fxy = std::hypot(world_force[0], world_force[1]);
  out.abs_fz = std::fabs(world_force[2]);
  out.threshold = kImpactVerticalMultiplier * out.abs_fz + kImpactOffsetN;
  if (!std::isfinite(out.fxy) || !std::isfinite(out.abs_fz) ||
      !std::isfinite(out.threshold)) return ImpactDecision{};
  out.known = true;
  out.impact = out.fxy > out.threshold;
  return out;
}

}  // namespace abs_foot
