#include <mujoco/mujoco.h>

#include <cstdlib>
#include <iostream>
#include <string>
#include <sys/mman.h>

#include "abs_collision_model_fingerprint.h"
#include "abs_scene_catalog.h"
#include "obstacle_collision_authority.h"

int main(int argc, char** argv) {
  if (argc != 4) {
    std::cerr << "usage: b1_base_bounds_probe <scene-dir> <scene-xml> <collision-shm-name>\n";
    return 2;
  }
  const auto* spec = abs_scene::byFilename(argv[2]);
  if (!spec) { std::cerr << "scene_not_registered\n"; return 3; }
  char error[1024] = {};
  mjModel* model = mj_loadXML((std::string(argv[1]) + "/" + argv[2]).c_str(),
                              nullptr, error, sizeof(error));
  if (!model) { std::cerr << error << "\n"; return 4; }
  mjData* data = mj_makeData(model);
  if (!data) { mj_deleteModel(model); return 5; }
  std::string fingerprint, fingerprint_error;
  if (!abs_collision_model::compute(model, &fingerprint, &fingerprint_error)) {
    std::cerr << fingerprint_error << "\n";
    mj_deleteData(data); mj_deleteModel(model); return 6;
  }
  int base_joint = -1;
  for (int id = 0; id < model->njnt; ++id) {
    if (model->jnt_type[id] == mjJNT_FREE) { base_joint = id; break; }
  }
  if (base_joint < 0) { std::cerr << "free_base_joint_missing\n"; return 7; }
  const int qadr = model->jnt_qposadr[base_joint];
  const std::string collision_name = argv[3];
  {
    ObstacleCollisionAuthority authority(collision_name.c_str());
    if (!authority.available()) {
      std::cerr << "collision_authority_unavailable\n";
      mj_deleteData(data); mj_deleteModel(model); return 8;
    }
    mj_resetData(model, data);
    mj_forward(model, data);
    mj_step(model, data);
    authority.publish(model, data, 1);
    // Force one controlled base-link sample outside the declared x bound
    // between reader polls, then return inside before the next sample.
    data->qpos[qadr] = 8.2;
    mj_forward(model, data);
    mj_step(model, data);
    authority.publish(model, data, 2);
    data->qpos[qadr] = 0.0;
    mj_forward(model, data);
    mj_step(model, data);
    authority.publish(model, data, 3);
    std::cout << "PUBLISHED scene_id=" << spec->id << " fingerprint=" << fingerprint
              << " final_physics_step=3\n" << std::flush;
  }
  shm_unlink(collision_name.c_str());
  mj_deleteData(data);
  mj_deleteModel(model);
  return 0;
}
