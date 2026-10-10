#pragma once

#include <array>
#include <cmath>
#include <cstring>
#include <string>
#include <vector>

#include <mujoco/mujoco.h>

namespace abs_scene {

enum class ObstacleKind { kLegacySignatures, kNoObstacles, kNamedPptObstacles };

struct SceneSpec {
  const char* filename;
  const char* id;
  int obstacle_count;
  ObstacleKind kind;
  const char* root_sha256;
  const char* closure_sha256;
  const char* model_fingerprint;
};

inline constexpr std::array<SceneSpec, 10> kScenes = {{
#define kLegacySignatures ObstacleKind::kLegacySignatures
#define kNoObstacles ObstacleKind::kNoObstacles
#define kNamedPptObstacles ObstacleKind::kNamedPptObstacles
#define ABS_SCENE(file, id, count, kind, root, closure, fingerprint) \
  {file, id, count, kind, root, closure, fingerprint},
#include "abs_scene_catalog.def"
#undef ABS_SCENE
#undef kLegacySignatures
#undef kNoObstacles
#undef kNamedPptObstacles
}};

struct Signature { int type; double pos[3]; double size[3]; };
inline constexpr std::array<Signature, 7> kLegacyObstacleSignatures = {{
    {mjGEOM_BOX, {4.77821, 1.39196, 0.319396}, {0.178009, 0.177997, 0.0656475}},
    {mjGEOM_BOX, {4.39779, 0.60669, 0.368633}, {0.110292, 0.584955, 0.315243}},
    {mjGEOM_BOX, {1.45553, -1.90905, 0.132532}, {0.252121, 0.362378, 0.0856493}},
    {mjGEOM_BOX, {1.81053, 0.671117, 0.112772}, {0.246072, 0.283181, 0.0786285}},
    {mjGEOM_BOX, {4.03329, -1.80196, 0.281405}, {0.396207, 0.123225, 0.190589}},
    {mjGEOM_BOX, {1.26736, -2.60969, 0.476998}, {0.582816, 0.504199, 0.18007}},
    {mjGEOM_CYLINDER, {1.69268, -1.29487, 0.609958}, {0.252499, 0.609958, 0.609958}},
}};

inline const SceneSpec* byId(const std::string& id) {
  for (const auto& scene : kScenes) if (id == scene.id) return &scene;
  return nullptr;
}

inline const SceneSpec* byFilename(const std::string& filename) {
  for (const auto& scene : kScenes) if (filename == scene.filename) return &scene;
  return nullptr;
}

inline bool isFloor(const mjModel* model, int geom_id) {
  const char* name = (model && geom_id >= 0 && geom_id < model->ngeom)
      ? mj_id2name(model, mjOBJ_GEOM, geom_id) : nullptr;
  return name != nullptr && std::strcmp(name, "floor") == 0;
}

inline bool isRobotGeom(const mjModel* model, int geom_id) {
  return model && geom_id >= 0 && geom_id < model->ngeom &&
         (model->geom_group[geom_id] == 2 || model->geom_group[geom_id] == 3);
}

inline bool nearlyEqual(double a, double b) {
  return std::isfinite(a) && std::fabs(a - b) <= 1e-12;
}

inline bool matchesLegacySignature(const mjModel* model, int geom_id) {
  if (!model || geom_id < 0 || geom_id >= model->ngeom || model->geom_bodyid[geom_id] != 0) return false;
  for (const auto& signature : kLegacyObstacleSignatures) {
    if (model->geom_type[geom_id] != signature.type) continue;
    bool match = true;
    for (int axis = 0; axis < 3; ++axis) {
      match = match && nearlyEqual(model->geom_pos[geom_id * 3 + axis], signature.pos[axis]);
      match = match && nearlyEqual(model->geom_size[geom_id * 3 + axis], signature.size[axis]);
    }
    if (match) return true;
  }
  return false;
}

inline bool selectObstacleGeoms(const mjModel* model, const SceneSpec* scene,
                                std::vector<int>& ids, std::string* error = nullptr) {
  ids.clear();
  if (!model || !scene) { if (error) *error = "missing_model_or_scene"; return false; }
  for (int geom_id = 0; geom_id < model->ngeom; ++geom_id) {
    if (isRobotGeom(model, geom_id) || isFloor(model, geom_id)) continue;
    const int type = model->geom_type[geom_id];
    if (type == mjGEOM_PLANE || type == mjGEOM_HFIELD || type == mjGEOM_MESH) continue;
    const char* name = mj_id2name(model, mjOBJ_GEOM, geom_id);
    const bool eligible_static = model->geom_bodyid[geom_id] == 0 &&
        (model->geom_contype[geom_id] != 0 || model->geom_conaffinity[geom_id] != 0);
    if (!eligible_static) continue;
    bool selected = false;
    switch (scene->kind) {
      case ObstacleKind::kLegacySignatures:
        selected = matchesLegacySignature(model, geom_id);
        break;
      case ObstacleKind::kNamedPptObstacles:
        selected = name != nullptr && std::strncmp(name, "ppt_obstacle_", 13) == 0;
        break;
      case ObstacleKind::kNoObstacles:
        selected = false;
        break;
    }
    if (selected) ids.push_back(geom_id);
    else {
      if (error) *error = std::string("unrecognized_world_collision_geom:") +
          (name == nullptr ? "<unnamed>" : name);
      ids.clear();
      return false;
    }
  }
  if (ids.size() != static_cast<std::size_t>(scene->obstacle_count)) {
    if (error) *error = "obstacle_count_mismatch";
    ids.clear();
    return false;
  }
  return true;
}

}  // namespace abs_scene
