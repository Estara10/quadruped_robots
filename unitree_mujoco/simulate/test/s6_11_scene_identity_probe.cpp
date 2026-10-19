#include <mujoco/mujoco.h>

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>
#include <sys/mman.h>
#include <unistd.h>
#include <vector>

#include "abs_collision_model_fingerprint.h"
#include "abs_scene_catalog.h"
#include "obstacle_collision_authority.h"

namespace {
bool loadScene(const std::string& scene_dir, const abs_scene::SceneSpec* spec,
               mjModel** model, std::string* fingerprint, std::string* error) {
  if (spec == nullptr) { *error = "scene_not_in_catalog"; return false; }
  const std::string path = scene_dir + "/" + spec->filename;
  char load_error[1024] = {};
  *model = mj_loadXML(path.c_str(), nullptr, load_error, sizeof(load_error));
  if (*model == nullptr) { *error = load_error; return false; }
  std::string fingerprint_error;
  if (!abs_collision_model::compute(*model, fingerprint, &fingerprint_error)) {
    *error = "fingerprint:" + fingerprint_error;
    mj_deleteModel(*model); *model = nullptr; return false;
  }
  std::vector<int> obstacles;
  if (!abs_scene::selectObstacleGeoms(*model, spec, obstacles, error) ||
      obstacles.size() != static_cast<std::size_t>(spec->obstacle_count)) {
    if (error->empty()) *error = "selected_obstacle_count_mismatch";
    mj_deleteModel(*model); *model = nullptr; return false;
  }
  for (int id : obstacles) {
    if (abs_scene::isFloor(*model, id) || abs_scene::isRobotGeom(*model, id)) {
      *error = "floor_or_robot_geom_selected";
      mj_deleteModel(*model); *model = nullptr; return false;
    }
  }
  return true;
}

int fingerprintMode(int argc, char** argv) {
  if (argc < 4) {
    std::cerr << "usage: s6_11_scene_identity_probe --fingerprints <scene-dir> <xml>...\n";
    return 2;
  }
  const std::string scene_dir = argv[2];
  for (int i = 3; i < argc; ++i) {
    const std::string filename = argv[i];
    const auto* spec = abs_scene::byFilename(filename);
    mjModel* model = nullptr;
    std::string fingerprint, error;
    if (!loadScene(scene_dir, spec, &model, &fingerprint, &error)) {
      std::cerr << "FAILED filename=" << filename << " error=" << error << "\n";
      return 3;
    }
    std::cout << "SCENE filename=" << filename << " id=" << spec->id
              << " obstacle_count=" << spec->obstacle_count
              << " fingerprint=" << fingerprint
              << " mujoco_version=" << mj_versionString() << "\n";
    mj_deleteModel(model);
  }
  return 0;
}

int publishMode(int argc, char** argv) {
  if (argc != 5) {
    std::cerr << "usage: s6_11_scene_identity_probe --publish <scene-dir> <xml> <shm-name>\n";
    return 2;
  }
  const std::string filename = argv[3];
  const std::string shm_name = argv[4];
  const auto* spec = abs_scene::byFilename(filename);
  mjModel* model = nullptr;
  std::string fingerprint, error;
  if (!loadScene(argv[2], spec, &model, &fingerprint, &error)) {
    std::cerr << "FAILED filename=" << filename << " error=" << error << "\n";
    return 3;
  }
  mjData* data = mj_makeData(model);
  if (data == nullptr) { mj_deleteModel(model); return 4; }
  {
    ObstacleCollisionAuthority authority(shm_name.c_str());
    if (!authority.available()) {
      mj_deleteData(data); mj_deleteModel(model); return 5;
    }
    mj_resetData(model, data);
    mj_step(model, data);
    authority.publish(model, data, 1);
    std::cout << "READY filename=" << filename << " id=" << spec->id
              << " obstacle_count=" << spec->obstacle_count
              << " fingerprint=" << fingerprint
              << " mujoco_version=" << mj_versionString() << " shm=" << shm_name << "\n";
    std::cout.flush();
    char ack = 0;
    if (!std::cin.get(ack) || ack != '\n') {
      mj_deleteData(data); mj_deleteModel(model); return 6;
    }
  }
  shm_unlink(shm_name.c_str());
  mj_deleteData(data);
  mj_deleteModel(model);
  std::cout << "CLEANED shm=" << shm_name << "\n";
  return 0;
}
}

int main(int argc, char** argv) {
  if (argc >= 2 && std::strcmp(argv[1], "--fingerprints") == 0) return fingerprintMode(argc, argv);
  if (argc >= 2 && std::strcmp(argv[1], "--publish") == 0) return publishMode(argc, argv);
  std::cerr << "expected --fingerprints or --publish\n";
  return 2;
}
