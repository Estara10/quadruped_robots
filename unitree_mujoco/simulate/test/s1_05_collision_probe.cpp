#include <mujoco/mujoco.h>

#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <limits>
#include <fcntl.h>
#include <string>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include "abs_collision_contract.h"
#include "abs_collision_model_fingerprint.h"
#include "obstacle_collision_authority.h"

namespace {
constexpr const char* kCapture = "p1-10-capture-aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa";
constexpr const char* kFingerprint = "3e4d82cf204d5929ff98c11fed5b3918ba3b92f0d698215860097d734d239694";

void setBasePose(const mjModel* model, mjData* data, double x, double y, double z,
                 double angle_rad = 0.0) {
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
  data->qpos[address + 0] = x;
  data->qpos[address + 1] = y;
  data->qpos[address + 2] = z;
  data->qpos[address + 3] = std::cos(angle_rad / 2.0);
  data->qpos[address + 4] = std::sin(angle_rad / 2.0);
  data->qpos[address + 5] = 0.0;
  data->qpos[address + 6] = 0.0;
}

abs_collision::Snapshot readSnapshot(const char* shm_name) {
  const int fd = shm_open(shm_name, O_RDONLY, 0);
  assert(fd >= 0);
  auto* snapshot = static_cast<abs_collision::Snapshot*>(mmap(
      nullptr, sizeof(abs_collision::Snapshot), PROT_READ, MAP_SHARED, fd, 0));
  assert(snapshot != MAP_FAILED);
  abs_collision::Snapshot copy{};
  std::memcpy(&copy, snapshot, sizeof(copy));
  munmap(snapshot, sizeof(abs_collision::Snapshot));
  close(fd);
  return copy;
}

void writeSnapshot(const std::string& directory, const char* name,
                   const abs_collision::Snapshot& snapshot) {
  std::ofstream out(directory + "/" + name, std::ios::binary | std::ios::trunc);
  out.write(reinterpret_cast<const char*>(&snapshot), sizeof(snapshot));
  if (!out) std::abort();
}
}

int main(int argc, char** argv) {
  if (argc != 4) return 2;
  setenv("ABS_P1_10_SCENARIO_ID", "obstacle_test1", 1);
  setenv("ABS_P1_10_ROOT_XML_SHA256",
         "e12a69fa5463e723d115696b8872c27c71b03a9d029a9ef933343ae93ba6dd5e", 1);
  setenv("ABS_P1_10_MODEL_CLOSURE_SHA256",
         "6ca5da14be6909815ac9c41bf6db0f8108e07082aea5aba22c91e833e6181746", 1);
  setenv("ABS_P1_10_CAPTURE_ID", kCapture, 1);
  setenv("ABS_P1_10_EXPECTED_MODEL_FINGERPRINT", kFingerprint, 1);

  char error[1024] = {};
  mjModel* model = mj_loadXML(argv[1], nullptr, error, sizeof(error));
  if (model == nullptr) {
    std::fprintf(stderr, "model_load_error=%s\n", error);
    return 3;
  }
  std::string actual_fingerprint;
  std::string fingerprint_error;
  if (!abs_collision_model::compute(model, &actual_fingerprint, &fingerprint_error) ||
      actual_fingerprint != kFingerprint) {
    std::fprintf(stderr, "fingerprint_mismatch=%s actual=%s\n",
                 fingerprint_error.c_str(), actual_fingerprint.c_str());
    mj_deleteModel(model);
    return 4;
  }
  mjData* data = mj_makeData(model);
  if (data == nullptr) {
    mj_deleteModel(model);
    return 5;
  }
  {
    ObstacleCollisionAuthority authority(argv[2]);
    if (!authority.available()) return 6;
    mj_resetData(model, data);
    setBasePose(model, data, 0.0, 0.0, 0.445);
    mj_forward(model, data);
    for (uint64_t step = 1; step <= 5; ++step) {
      mj_step(model, data);
      authority.publish(model, data, step);
    }
    writeSnapshot(argv[3], "initial.bin", readSnapshot(argv[2]));

    // Controlled contact: pin the base above one of the bound obstacle boxes
    // for several physics steps. This is a source-path diagnostic, not policy data.
    for (uint64_t step = 6; step <= 10; ++step) {
      mj_step(model, data);
      setBasePose(model, data, 1.45553, -1.90905, 0.48);
      mj_forward(model, data);
      authority.publish(model, data, step);
    }
    auto contact = readSnapshot(argv[2]);
    if (contact.authoritative != 1 || contact.collision_episode_count == 0 ||
        contact.collision_history_count == 0 ||
        contact.physics_coverage_complete != 1) {
      std::fprintf(stderr, "controlled_contact_not_recorded auth=%u episodes=%llu history=%u coverage=%u\n",
                   contact.authoritative,
                   static_cast<unsigned long long>(contact.collision_episode_count),
                   contact.collision_history_count, contact.physics_coverage_complete);
      return 7;
    }

    // Create a one-step unclassified contact between observations, then restore
    // the real contact before any later MuJoCo step.
    mj_step(model, data);
    if (data->ncon > 0) {
      const int saved_geom = data->contact[0].geom1;
      data->contact[0].geom1 = model->ngeom + 10;
      authority.publish(model, data, 11);
      data->contact[0].geom1 = saved_geom;
      writeSnapshot(argv[3], "unknown_contact.bin", readSnapshot(argv[2]));
    }
    mj_step(model, data);
    const int base_id = mj_name2id(model, mjOBJ_BODY, "base_link");
    const double saved_height = data->xpos[3 * base_id + 2];
    data->xpos[3 * base_id + 2] = std::numeric_limits<double>::quiet_NaN();
    authority.publish(model, data, 12);
    data->xpos[3 * base_id + 2] = saved_height;
    writeSnapshot(argv[3], "invalid_posture.bin", readSnapshot(argv[2]));

    // Startup fall episode, recovery, then a second episode. The v4 ring must
    // retain both even though a policy reader would sample far less often.
    for (uint64_t step = 11; step <= 180; ++step) {
      mj_step(model, data);
      setBasePose(model, data, 0.0, 0.0, step < 162 ? 0.10 : 0.445);
      mj_forward(model, data);
      authority.publish(model, data, step + 2);
      if (step == 161) writeSnapshot(argv[3], "startup_confirmed.bin", readSnapshot(argv[2]));
    }
    writeSnapshot(argv[3], "recovered.bin", readSnapshot(argv[2]));
    for (uint64_t step = 183; step <= 360; ++step) {
      mj_step(model, data);
      setBasePose(model, data, 0.0, 0.0, 0.10);
      mj_forward(model, data);
      authority.publish(model, data, step);
    }
    const auto final = readSnapshot(argv[2]);
    writeSnapshot(argv[3], "final.bin", final);
    const auto& event = final.collision_history[0];
    std::printf("authoritative=%u physics_coverage=%u episodes=%llu contact_steps=%llu "
                "event=%s/%s fall_candidate=%u fall_confirmed=%u fall_start_step=%llu "
                "fall_confirmed_step=%llu fall_events=%u unknown_contact_steps=%llu foot_floor=%u nonfoot_floor=%u\n",
                final.authoritative, final.physics_coverage_complete,
                static_cast<unsigned long long>(final.collision_episode_count),
                static_cast<unsigned long long>(final.collision_contact_steps),
                event.robot_geom_name, event.obstacle_geom_name, final.fall_candidate,
                final.fall_confirmed,
                static_cast<unsigned long long>(final.fall_start_physics_step),
                static_cast<unsigned long long>(final.fall_confirmed_physics_step),
                final.fall_history_count,
                static_cast<unsigned long long>(final.unknown_contact_steps),
                final.foot_ground_contacts, final.nonfoot_ground_contacts);
    if (final.fall_confirmed != 1 || final.fall_confirmed_physics_step == 0 ||
        final.physics_coverage_complete != 1 || final.version != 4 ||
        sizeof(abs_collision::Snapshot) != 5760 || final.unknown_contact_steps == 0 ||
        final.fall_history_count < 2 || final.ground_contacts !=
          final.foot_ground_contacts + final.nonfoot_ground_contacts) return 8;
  }
  mj_deleteData(data);
  mj_deleteModel(model);
  // The Python harness owns cleanup of this unique test-only object.
  return 0;
}
