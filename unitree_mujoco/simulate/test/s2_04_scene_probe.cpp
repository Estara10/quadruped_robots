#include <mujoco/mujoco.h>

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <string>
#include <sys/mman.h>
#include <unistd.h>
#include <vector>

#include "abs_collision_contract.h"
#include "abs_collision_model_fingerprint.h"
#include "abs_scene_catalog.h"
#include "obstacle_collision_authority.h"

namespace {
abs_collision::Snapshot readSnapshot(const std::string& name) {
  const int fd = shm_open(name.c_str(), O_RDONLY, 0);
  if (fd < 0) std::abort();
  auto* view = static_cast<abs_collision::Snapshot*>(mmap(
      nullptr, sizeof(abs_collision::Snapshot), PROT_READ, MAP_SHARED, fd, 0));
  if (view == MAP_FAILED) std::abort();
  abs_collision::Snapshot result{};
  std::memcpy(&result, view, sizeof(result));
  munmap(view, sizeof(abs_collision::Snapshot));
  close(fd);
  return result;
}
}

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "usage: s2_04_scene_probe <go2-scene-directory>\n";
    return 2;
  }
  const std::string scene_dir = argv[1];
  const std::string capture_id = "p1-10-capture-aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa";
  for (const auto& scene : abs_scene::kScenes) {
    const std::string scene_path = scene_dir + "/" + scene.filename;
    char error[1024] = {};
    mjModel* model = mj_loadXML(scene_path.c_str(), nullptr, error, sizeof(error));
    if (!model) {
      std::cerr << "model_load_failed scene=" << scene.filename << " error=" << error << "\n";
      return 3;
    }
    std::string fingerprint, fingerprint_error;
    std::vector<int> obstacle_ids;
    std::string identity_error;
    if (!abs_collision_model::compute(model, &fingerprint, &fingerprint_error) ||
        fingerprint != scene.model_fingerprint ||
        !abs_scene::selectObstacleGeoms(model, &scene, obstacle_ids, &identity_error) ||
        obstacle_ids.size() != static_cast<std::size_t>(scene.obstacle_count)) {
      std::cerr << "scene_identity_failed scene=" << scene.filename
                << " fingerprint=" << fingerprint << " fingerprint_error=" << fingerprint_error
                << " geometry_error=" << identity_error << "\n";
      mj_deleteModel(model);
      return 4;
    }
    for (int id : obstacle_ids) {
      if (abs_scene::isFloor(model, id) || abs_scene::isRobotGeom(model, id)) {
        std::cerr << "excluded_geom_selected scene=" << scene.filename << " geom=" << id << "\n";
        mj_deleteModel(model);
        return 5;
      }
    }
    for (int id = 0; id < model->ngeom; ++id) {
      if (abs_scene::isFloor(model, id)) {
        for (int selected : obstacle_ids) if (selected == id) return 6;
      }
    }

    const std::string shm_name = "/abs_s204_" + std::to_string(getpid()) + "_" + scene.id;
    setenv("ABS_P1_10_SCENARIO_ID", scene.id, 1);
    setenv("ABS_P1_10_ROOT_XML_SHA256", scene.root_sha256, 1);
    setenv("ABS_P1_10_MODEL_CLOSURE_SHA256", scene.closure_sha256, 1);
    setenv("ABS_P1_10_CAPTURE_ID", capture_id.c_str(), 1);
    setenv("ABS_P1_10_EXPECTED_MODEL_FINGERPRINT", scene.model_fingerprint, 1);
    mjData* data = mj_makeData(model);
    if (!data) return 7;
    {
      ObstacleCollisionAuthority authority(shm_name.c_str());
      if (!authority.available()) return 8;
      mj_resetData(model, data);
      mj_step(model, data);
      authority.publish(model, data, 1);
      mj_step(model, data);
      authority.publish(model, data, 2);
      const auto snapshot = readSnapshot(shm_name);
      if (snapshot.authoritative != 1 || snapshot.robot_obstacle_contacts != 0 ||
          snapshot.current_collision != 0 || snapshot.scenario_id[0] == '\0') {
        std::cerr << "authority_publish_failed scene=" << scene.filename
                  << " authoritative=" << snapshot.authoritative
                  << " obstacle_contacts=" << snapshot.robot_obstacle_contacts << "\n";
        return 9;
      }
      if (scene.obstacle_count == 0 && snapshot.ground_contacts == 0) {
        std::cerr << "flat_floor_contact_not_observed\n";
        return 10;
      }
      std::cout << "scene=" << scene.filename << " id=" << scene.id
                << " obstacle_count=" << obstacle_ids.size()
                << " ground_contacts=" << snapshot.ground_contacts
                << " robot_obstacle_contacts=" << snapshot.robot_obstacle_contacts
                << " authoritative=1 fingerprint=" << fingerprint << "\n";
    }
    shm_unlink(shm_name.c_str());
    mj_deleteData(data);
    mj_deleteModel(model);
  }
  return 0;
}
