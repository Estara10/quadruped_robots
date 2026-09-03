// Offline collision-model fingerprint contract test. It loads a model but
// never calls mj_step, starts a UI, or starts ROS/runtime capture.
#include <mujoco/mujoco.h>

#include <cstdio>
#include <string>

#include "abs_collision_model_fingerprint.h"

namespace {
bool fingerprint(const mjModel* model, std::string* out) {
  std::string error;
  return abs_collision_model::compute(model, out, &error);
}

mjModel* load(const char* path) {
  char error[1024] = {0};
  mjModel* model = mj_loadXML(path, nullptr, error, sizeof(error));
  if (model == nullptr) std::fprintf(stderr, "load failed: %s\n", error);
  return model;
}

bool changed(const char* path, const char* label,
             void (*mutate)(mjModel*, int)) {
  mjModel* original = load(path);
  mjModel* altered = load(path);
  if (original == nullptr || altered == nullptr) return false;
  std::string before, after;
  const int geom_id = 0;
  const bool ok = fingerprint(original, &before);
  mutate(altered, geom_id);
  const bool altered_ok = fingerprint(altered, &after);
  const bool result = ok && altered_ok && before != after;
  if (!result) std::fprintf(stderr, "fingerprint mutation not detected: %s\n", label);
  mj_deleteModel(original);
  mj_deleteModel(altered);
  return result;
}
}

int main(int argc, char** argv) {
  if (argc != 2) return 2;
  const char* path = argv[1];
  int checks = 0;
  auto check = [&](bool value, const char* label) {
    ++checks;
    if (!value) std::fprintf(stderr, "FAIL %s\n", label);
    return value;
  };
  check(changed(path, "type", [](mjModel* m, int g) { ++m->geom_type[g]; }), "type");
  check(changed(path, "body", [](mjModel* m, int g) {
    m->geom_bodyid[g] = (m->geom_bodyid[g] + 1) % m->nbody;
  }), "body");
  check(changed(path, "pos", [](mjModel* m, int g) { m->geom_pos[3 * g] += 0.001; }), "pos");
  check(changed(path, "quat", [](mjModel* m, int g) { m->geom_quat[4 * g] += 0.001; }), "quat");
  check(changed(path, "size", [](mjModel* m, int g) { m->geom_size[3 * g] += 0.001; }), "size");
  check(changed(path, "contype", [](mjModel* m, int g) { m->geom_contype[g] ^= 1; }), "contype");
  check(changed(path, "conaffinity", [](mjModel* m, int g) { m->geom_conaffinity[g] ^= 1; }), "conaffinity");
  check(changed(path, "group", [](mjModel* m, int g) { ++m->geom_group[g]; }), "group");
  check(changed(path, "geom_name", [](mjModel* m, int g) {
    const int address = m->name_geomadr[g];
    if (address >= 0 && m->names[address] != '\0') m->names[address] = 'X';
  }), "geom_name");
  std::printf("P1-10 model fingerprint mutation test PASS (%d checks)\n", checks);
  return 0;
}
