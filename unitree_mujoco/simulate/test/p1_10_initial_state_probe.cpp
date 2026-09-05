// P1-10 offline initial-state probe.
//
// This loads the same scene XML and executes only the model construction path
// used by PhysicsThread: mj_loadXML -> mj_makeData -> mj_forward.  It does not
// call mj_step, start the simulator UI, or start ROS2.  The output is consumed
// by the P1-10 Python binding validator.

#include <mujoco/mujoco.h>

#include "abs_collision_model_fingerprint.h"

#include <cstdio>
#include <cmath>

static void print_vector(const char* name, const mjtNum* values, int count)
{
    std::printf("%s=[", name);
    for (int i = 0; i < count; ++i)
    {
        if (i) std::printf(",");
        std::printf("%.17g", static_cast<double>(values[i]));
    }
    std::printf("]\n");
}

static double yaw_from_quat(const mjtNum* q)
{
    // MuJoCo free-joint quaternion order is w,x,y,z.
    const double w = q[0];
    const double x = q[1];
    const double y = q[2];
    const double z = q[3];
    return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}

int main(int argc, char** argv)
{
    if (argc != 2)
    {
        std::fprintf(stderr, "usage: %s <scene.xml>\n", argv[0]);
        return 2;
    }

    char error[1024] = {0};
    mjModel* model = mj_loadXML(argv[1], nullptr, error, sizeof(error));
    if (!model)
    {
        std::fprintf(stderr, "mj_loadXML failed: %s\n", error);
        return 1;
    }
    mjData* data = mj_makeData(model);
    if (!data)
    {
        std::fprintf(stderr, "mj_makeData failed\n");
        mj_deleteModel(model);
        return 1;
    }

    std::printf("scene=%s\n", argv[1]);
    std::printf("mujoco_version=%s\n", mj_versionString());
    std::printf("nq=%d\n", model->nq);
    std::printf("startup_path=mj_loadXML->mj_makeData->mj_forward\n");
    print_vector("qpos_before_forward", data->qpos, model->nq);
    print_vector("qpos0", model->qpos0, model->nq);

    if (model->nkey > 0)
    {
        const char* key_name = model->names + model->name_keyadr[0];
        std::printf("keyframe0_name=%s\n", key_name);
        print_vector("keyframe0_qpos", model->key_qpos, model->nq);
    }
    else
    {
        std::printf("keyframe0_name=NONE\n");
    }

    mj_forward(model, data);
    print_vector("qpos_after_forward", data->qpos, model->nq);
    std::string fingerprint;
    std::string fingerprint_error;
    if (!abs_collision_model::compute(model, &fingerprint, &fingerprint_error))
    {
        std::fprintf(stderr, "collision fingerprint failed: %s\n", fingerprint_error.c_str());
        mj_deleteData(data);
        mj_deleteModel(model);
        return 1;
    }
    std::printf("collision_model_fingerprint=%s\n", fingerprint.c_str());
    std::printf("collision_model_fingerprint_schema=%s\n", abs_collision_model::kFingerprintSchema);
    if (model->nq >= 7)
    {
        std::printf("base_pose_world_m=[%.17g,%.17g,%.17g]\n",
                    static_cast<double>(data->qpos[0]),
                    static_cast<double>(data->qpos[1]),
                    static_cast<double>(data->qpos[2]));
        std::printf("base_quat_wxyz=[%.17g,%.17g,%.17g,%.17g]\n",
                    static_cast<double>(data->qpos[3]),
                    static_cast<double>(data->qpos[4]),
                    static_cast<double>(data->qpos[5]),
                    static_cast<double>(data->qpos[6]));
        std::printf("base_yaw_rad=%.17g\n", yaw_from_quat(data->qpos + 3));
    }

    mj_deleteData(data);
    mj_deleteModel(model);
    return 0;
}
