#include <mujoco/mujoco.h>

#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <fstream>
#include <iostream>
#include <limits>
#include <string>
#include <sys/mman.h>
#include <unistd.h>

#include "abs_collision_contract.h"
#include "abs_collision_model_fingerprint.h"
#include "abs_scene_catalog.h"
#include "foot_contact_policy.h"
#include "obstacle_collision_authority.h"

namespace {
abs_collision::Snapshot readSnapshot(const std::string& name) {
  const int fd = shm_open(name.c_str(), O_RDONLY, 0);
  assert(fd >= 0);
  auto* ptr = static_cast<abs_collision::Snapshot*>(mmap(
      nullptr, sizeof(abs_collision::Snapshot), PROT_READ, MAP_SHARED, fd, 0));
  assert(ptr != MAP_FAILED);
  abs_collision::Snapshot result{};
  std::memcpy(&result, ptr, sizeof(result));
  munmap(ptr, sizeof(abs_collision::Snapshot));
  close(fd);
  return result;
}

void saveSnapshot(const std::string& path, const abs_collision::Snapshot& value) {
  std::ofstream out(path, std::ios::binary | std::ios::trunc);
  out.write(reinterpret_cast<const char*>(&value), sizeof(value));
  assert(out.good());
}

void setBasePose(const mjModel* model, mjData* data, double x, double y, double z) {
  const int base = mj_name2id(model, mjOBJ_BODY, "base_link");
  assert(base >= 0);
  int free_joint = -1;
  for (int joint = 0; joint < model->njnt; ++joint) {
    if (model->jnt_bodyid[joint] == base && model->jnt_type[joint] == mjJNT_FREE) {
      free_joint = joint;
      break;
    }
  }
  assert(free_joint >= 0);
  const int address = model->jnt_qposadr[free_joint];
  data->qpos[address] = x; data->qpos[address + 1] = y; data->qpos[address + 2] = z;
  data->qpos[address + 3] = 1.0; data->qpos[address + 4] = 0.0;
  data->qpos[address + 5] = 0.0; data->qpos[address + 6] = 0.0;
}
}

int main(int argc, char** argv) {
  if (argc != 4) {
    std::fprintf(stderr, "usage: s2_05_contact_probe <sparse-scene.xml> <unique-shm-name> <snapshot.bin>\n");
    return 2;
  }
  const auto* scene = abs_scene::byId("ppt_sparse");
  assert(scene != nullptr);
  setenv("ABS_P1_10_SCENARIO_ID", scene->id, 1);
  setenv("ABS_P1_10_ROOT_XML_SHA256", scene->root_sha256, 1);
  setenv("ABS_P1_10_MODEL_CLOSURE_SHA256", scene->closure_sha256, 1);
  setenv("ABS_P1_10_CAPTURE_ID", "p1-10-capture-aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa", 1);
  setenv("ABS_P1_10_EXPECTED_MODEL_FINGERPRINT", scene->model_fingerprint, 1);

  const auto below = abs_foot::classifyImpact({{29.999, 0.0, 10.0}}, true, true);
  const auto equal = abs_foot::classifyImpact({{30.0, 0.0, 10.0}}, true, true);
  const auto above = abs_foot::classifyImpact({{30.001, 0.0, 10.0}}, true, true);
  const auto unknown = abs_foot::classifyImpact(
      {{std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0}}, true, true);
  assert(below.known && !below.impact);
  assert(equal.known && !equal.impact);
  assert(above.known && above.impact);
  assert(above.impact && !abs_foot::isTerminalRobotObstacleContact(true));
  assert(!unknown.known);
  assert(!abs_foot::classifyImpact({{100.0, 0.0, 0.0}}, true, false).known);
  assert(!abs_foot::isTerminalRobotObstacleContact(true));
  assert(abs_foot::isTerminalRobotObstacleContact(false));
  std::array<double, 3> aggregate{};
  assert(abs_foot::accumulateWorldForce(&aggregate, {{20.0, 0.0, 0.0}}));
  assert(abs_foot::accumulateWorldForce(&aggregate, {{10.001, 0.0, 10.0}}));
  assert(abs_foot::classifyImpact(aggregate, true, true).impact);
  assert(!abs_foot::accumulateWorldForce(
      &aggregate, {{std::numeric_limits<double>::infinity(), 0.0, 0.0}}));
  mjContact orientation_contact{};
  orientation_contact.efc_address = 0;
  orientation_contact.frame[0] = 1.0;
  orientation_contact.frame[4] = 1.0;
  orientation_contact.frame[8] = 1.0;
  mjtNum orientation_wrench[6] = {12.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  std::array<double, 3> geom1_force{}, geom2_force{};
  assert(abs_foot::contactForceOnFootWorld(orientation_contact, orientation_wrench, true, &geom1_force));
  assert(abs_foot::contactForceOnFootWorld(orientation_contact, orientation_wrench, false, &geom2_force));
  assert(geom1_force[0] == -12.0 && geom2_force[0] == 12.0);

  char error[1024]{};
  mjModel* model = mj_loadXML(argv[1], nullptr, error, sizeof(error));
  if (model == nullptr) {
    std::fprintf(stderr, "model_load_failed=%s\n", error);
    return 3;
  }
  abs_foot::FootGeomSet feet;
  std::string foot_error;
  assert(abs_foot::resolveFootGeoms(model, &feet, &foot_error) && feet.valid);
  for (std::size_t i = 0; i < abs_foot::kLegs.size(); ++i) {
    const int id = feet.geom_ids[i];
    assert(mj_id2name(model, mjOBJ_GEOM, id) != nullptr);
    assert(std::strcmp(mj_id2name(model, mjOBJ_GEOM, id), abs_foot::kLegs[i]) == 0);
    assert(std::strcmp(mj_id2name(model, mjOBJ_BODY, model->geom_bodyid[id]),
                       (std::string(abs_foot::kLegs[i]) + "_calf").c_str()) == 0);
    assert(model->geom_type[id] == mjGEOM_SPHERE && model->geom_group[id] == 3);
  }
  std::cout << "foot_geom_owners=FL:FL_calf,FR:FR_calf,RL:RL_calf,RR:RR_calf\n";
  std::string fingerprint, fingerprint_error;
  assert(abs_collision_model::compute(model, &fingerprint, &fingerprint_error));
  assert(fingerprint == scene->model_fingerprint);
  int floor_id = mj_name2id(model, mjOBJ_GEOM, "floor");
  assert(floor_id >= 0);
  int obstacle_id = mj_name2id(model, mjOBJ_GEOM, "ppt_obstacle_01");
  assert(obstacle_id >= 0);
  int nonfoot_id = -1;
  for (int id = 0; id < model->ngeom; ++id) {
    if (model->geom_group[id] != 3 || abs_foot::footIndex(feet, id) >= 0) continue;
    const int body = model->geom_bodyid[id];
    const char* body_name = mj_id2name(model, mjOBJ_BODY, body);
    if (body_name != nullptr &&
        (std::strstr(body_name, "thigh") != nullptr || std::strstr(body_name, "calf") != nullptr)) {
      nonfoot_id = id;
      break;
    }
  }
  assert(nonfoot_id >= 0);
  const char* nonfoot_geom_name = mj_id2name(model, mjOBJ_GEOM, nonfoot_id);
  const char* nonfoot_body_name = mj_id2name(model, mjOBJ_BODY, model->geom_bodyid[nonfoot_id]);
  std::cout << "nonfoot_identity geom=" << (nonfoot_geom_name ? nonfoot_geom_name : "<unnamed>")
            << " body=" << (nonfoot_body_name ? nonfoot_body_name : "<unnamed>") << "\n";

  mjData* data = mj_makeData(model);
  assert(data != nullptr);
  std::string shm_name = argv[2];
  int actual_foot_floor_samples = 0;
  bool actual_force_sample = false;
  std::array<double, 3> real_floor_force{};
  std::string real_floor_foot;
  {
    ObstacleCollisionAuthority authority(shm_name.c_str());
    assert(authority.available());
    mj_resetData(model, data);
    setBasePose(model, data, 0.0, 0.0, 0.445);
    mj_forward(model, data);
    for (uint64_t step = 1; step <= 2000; ++step) {
      mj_step(model, data);
      authority.publish(model, data, step);
      const int ncon = data->ncon;
      for (int i = 0; i < ncon; ++i) {
        const auto& contact = data->contact[i];
        const int foot1 = abs_foot::footIndex(feet, contact.geom1);
        const int foot2 = abs_foot::footIndex(feet, contact.geom2);
        if ((foot1 >= 0 && contact.geom2 == floor_id) ||
            (foot2 >= 0 && contact.geom1 == floor_id)) {
          ++actual_foot_floor_samples;
          if (contact.efc_address >= 0) {
            mjtNum wrench[6]{};
            std::array<double, 3> world{};
            mj_contactForce(model, data, i, wrench);
            if (abs_foot::contactForceOnFootWorld(contact, wrench, foot1 >= 0, &world)) {
              assert(world[2] > 0.0);
              actual_force_sample = true;
              real_floor_force = world;
              real_floor_foot = abs_foot::kLegs[foot1 >= 0 ? foot1 : foot2];
            }
          }
        }
      }
      const auto current = readSnapshot(shm_name);
      if (current.authoritative == 1 && actual_force_sample) break;
    }
    assert(actual_foot_floor_samples > 0 && actual_force_sample);
    auto snapshot = readSnapshot(shm_name);
    assert(snapshot.authoritative == 1 && snapshot.version == 5);
    assert(snapshot.v5.foot_identity_valid == 1);
    assert(snapshot.foot_ground_contacts > 0);
    assert(snapshot.current_collision == 0);
    assert(snapshot.v5.nonfoot_obstacle_contacts == 0);
    assert(snapshot.v5.foot_obstacle_contacts == 0);
    saveSnapshot(std::string(argv[3]) + ".floor.bin", snapshot);

    // Classification-path injection uses one real active MuJoCo foot/floor
    // contact wrench and frame, changing only the paired geometry identity.
    // It checks policy routing, not physical obstacle impact magnitude.
    int selected = -1;
    bool foot_is_geom1 = false;
    for (int i = 0; i < data->ncon; ++i) {
      const auto& contact = data->contact[i];
      if (contact.efc_address < 0) continue;
      if (abs_foot::footIndex(feet, contact.geom1) >= 0 && contact.geom2 == floor_id) {
        selected = i; foot_is_geom1 = true; break;
      }
      if (abs_foot::footIndex(feet, contact.geom2) >= 0 && contact.geom1 == floor_id) {
        selected = i; foot_is_geom1 = false; break;
      }
    }
    assert(selected >= 0);
    mj_step(model, data);
    selected = -1;
    for (int i = 0; i < data->ncon; ++i) {
      const auto& contact = data->contact[i];
      if (contact.efc_address < 0) continue;
      if (abs_foot::footIndex(feet, contact.geom1) >= 0 && contact.geom2 == floor_id) {
        selected = i; foot_is_geom1 = true; break;
      }
      if (abs_foot::footIndex(feet, contact.geom2) >= 0 && contact.geom1 == floor_id) {
        selected = i; foot_is_geom1 = false; break;
      }
    }
    assert(selected >= 0);
    if (foot_is_geom1) data->contact[selected].geom2 = obstacle_id;
    else data->contact[selected].geom1 = obstacle_id;
    authority.publish(model, data, snapshot.physics_step + 1);
    auto foot_contact = readSnapshot(shm_name);
    assert(foot_contact.authoritative == 1 && foot_contact.current_collision == 0);
    assert(foot_contact.v5.foot_obstacle_contacts > 0);
    assert(foot_contact.v5.nonfoot_obstacle_contacts == 0);
    saveSnapshot(std::string(argv[3]) + ".foot.bin", foot_contact);

    mj_step(model, data);
    selected = -1;
    for (int i = 0; i < data->ncon; ++i) {
      const auto& contact = data->contact[i];
      if (contact.efc_address < 0) continue;
      if (abs_foot::footIndex(feet, contact.geom1) >= 0 && contact.geom2 == floor_id) {
        selected = i; foot_is_geom1 = true; break;
      }
      if (abs_foot::footIndex(feet, contact.geom2) >= 0 && contact.geom1 == floor_id) {
        selected = i; foot_is_geom1 = false; break;
      }
    }
    assert(selected >= 0);
    if (foot_is_geom1) {
      data->contact[selected].geom1 = nonfoot_id;
      data->contact[selected].geom2 = obstacle_id;
    } else {
      data->contact[selected].geom1 = obstacle_id;
      data->contact[selected].geom2 = nonfoot_id;
    }
    authority.publish(model, data, snapshot.physics_step + 2);
    auto nonfoot_contact = readSnapshot(shm_name);
    assert(nonfoot_contact.authoritative == 1 && nonfoot_contact.current_collision == 1);
    assert(nonfoot_contact.v5.nonfoot_obstacle_contacts > 0);
    saveSnapshot(std::string(argv[3]) + ".nonfoot.bin", nonfoot_contact);
    std::cout << "version=" << snapshot.version << " bytes=" << sizeof(snapshot)
              << " foot_floor_samples=" << actual_foot_floor_samples
              << " foot_ground_contacts=" << snapshot.foot_ground_contacts
              << " real_mj_contact_force_world_transform=PASS"
              << " real_floor_foot=" << real_floor_foot
              << " real_floor_force_world_N=" << real_floor_force[0] << ","
              << real_floor_force[1] << "," << real_floor_force[2]
              << " injected_force_impact_mask=" << foot_contact.v5.foot_impact_mask
              << " injected_foot_obstacle_terminal=0"
              << " injected_nonfoot_obstacle_terminal=1\n";
  }
  shm_unlink(shm_name.c_str());
  mj_deleteData(data);
  mj_deleteModel(model);
  return 0;
}
