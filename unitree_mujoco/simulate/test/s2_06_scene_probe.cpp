#include <mujoco/mujoco.h>

#include <algorithm>
#include <iostream>
#include <string>
#include <vector>

#include "abs_collision_model_fingerprint.h"
#include "abs_scene_catalog.h"

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "usage: s2_06_scene_probe <go2-scene-directory>\n";
    return 2;
  }
  const std::string scene_dir = argv[1];
  for (const auto& scene : abs_scene::kScenes) {
    if (std::string(scene.id).rfind("random_", 0) != 0) continue;
    const std::string path = scene_dir + "/" + scene.filename;
    char error[1024] = {};
    mjModel* model = mj_loadXML(path.c_str(), nullptr, error, sizeof(error));
    if (!model) {
      std::cerr << "model_load_failed scene=" << scene.filename << " error=" << error << "\n";
      return 3;
    }
    std::string fingerprint, fingerprint_error, geometry_error;
    std::vector<int> obstacle_ids;
    const bool fingerprint_ok = abs_collision_model::compute(model, &fingerprint, &fingerprint_error);
    const bool geometry_ok = abs_scene::selectObstacleGeoms(model, &scene, obstacle_ids, &geometry_error);
    if (!fingerprint_ok || !geometry_ok || obstacle_ids.size() != static_cast<std::size_t>(scene.obstacle_count)) {
      std::cerr << "preflight_failed scene=" << scene.filename << " fingerprint=" << fingerprint
                << " fingerprint_error=" << fingerprint_error << " geometry_error=" << geometry_error << "\n";
      mj_deleteModel(model);
      return 4;
    }
    if (fingerprint != scene.model_fingerprint) {
      std::cerr << "catalog_fingerprint_mismatch scene=" << scene.filename
                << " actual=" << fingerprint << " expected=" << scene.model_fingerprint << "\n";
      mj_deleteModel(model);
      return 7;
    }
    std::cout << "SCENE filename=" << scene.filename << " id=" << scene.id
              << " expected_count=" << scene.obstacle_count
              << " loaded_count=" << obstacle_ids.size() << " fingerprint=" << fingerprint << "\n";
    for (int id : obstacle_ids) {
      const char* name = mj_id2name(model, mjOBJ_GEOM, id);
      std::cout << "GEOM scene=" << scene.id << " name=" << (name ? name : "<unnamed>")
                << " type=" << model->geom_type[id]
                << " pos=" << model->geom_pos[3 * id] << "," << model->geom_pos[3 * id + 1]
                << "," << model->geom_pos[3 * id + 2]
                << " size=" << model->geom_size[3 * id] << "," << model->geom_size[3 * id + 1]
                << "," << model->geom_size[3 * id + 2] << "\n";
      if (abs_scene::isFloor(model, id) || abs_scene::isRobotGeom(model, id)) {
        mj_deleteModel(model);
        return 5;
      }
    }
    for (int id = 0; id < model->ngeom; ++id) {
      const char* name = mj_id2name(model, mjOBJ_GEOM, id);
      if (name && std::string(name) == "floor" &&
          std::find(obstacle_ids.begin(), obstacle_ids.end(), id) != obstacle_ids.end()) {
        mj_deleteModel(model);
        return 6;
      }
    }
    mj_deleteModel(model);
  }
  return 0;
}
