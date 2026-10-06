//
// P1-08 — offline MuJoCo model probe.
//
// Loads the launched scene XML with libmujoco and prints the AUTHORITATIVE
// effective static model facts (mjModel.opt.*, dimensions/counts, actuator and
// joint ranges) that feed the P1-08 baseline manifest. It also steps the model
// a few times to confirm d->time advances by opt.timestep (independent
// arithmetic check of the observed physics timestep).
//
// Usage: p1_08_model_probe <scene.xml>
//

#include <mujoco/mujoco.h>

#include <cstdio>
#include <cstring>
#include <string>

static const char* integratorName(int id)
{
    switch (id)
    {
        case mjINT_EULER: return "Euler";
        case mjINT_RK4: return "RK4";
        case mjINT_IMPLICIT: return "Implicit";
        case mjINT_IMPLICITFAST: return "ImplicitFast";
        default: return "unknown";
    }
}

static const char* coneName(int id)
{
    switch (id)
    {
        case mjCONE_PYRAMIDAL: return "pyramidal";
        case mjCONE_ELLIPTIC: return "elliptic";
        default: return "unknown";
    }
}

static const char* solverName(int id)
{
    switch (id)
    {
        case mjSOL_PGS: return "PGS";
        case mjSOL_CG: return "CG";
        case mjSOL_NEWTON: return "Newton";
        default: return "unknown";
    }
}

int main(int argc, char** argv)
{
    if (argc < 2)
    {
        std::fprintf(stderr, "usage: %s <scene.xml>\n", argv[0]);
        return 2;
    }

    char loadError[1024] = {0};
    mjModel* m = mj_loadXML(argv[1], nullptr, loadError, 1024);
    if (!m)
    {
        std::fprintf(stderr, "mj_loadXML failed: %s\n", loadError);
        return 1;
    }
    mjData* d = mj_makeData(m);
    if (!d)
    {
        std::fprintf(stderr, "mj_makeData failed\n");
        mj_deleteModel(m);
        return 1;
    }

    std::printf("scene=%s\n", argv[1]);
    std::printf("mujoco_version=%s\n", mj_versionString());

    // --- dimensions / counts ---
    std::printf("dims.nq=%d\n", m->nq);
    std::printf("dims.nv=%d\n", m->nv);
    std::printf("dims.nu=%d\n", m->nu);
    std::printf("dims.njnt=%d\n", m->njnt);
    std::printf("dims.nbody=%d\n", m->nbody);
    std::printf("dims.ngeom=%d\n", m->ngeom);
    std::printf("dims.nsite=%d\n", m->nsite);
    std::printf("dims.nsensor=%d\n", m->nsensor);
    std::printf("dims.nmesh=%d\n", m->nmesh);
    std::printf("dims.nflex=%d\n", m->nflex);
    std::printf("dims.ncam=%d\n", m->ncam);

    // --- mjOption effective values ---
    std::printf("opt.timestep=%.9f\n", m->opt.timestep);
    std::printf("opt.apirate=%.6f\n", m->opt.apirate);
    std::printf("opt.integrator=%s\n", integratorName(m->opt.integrator));
    std::printf("opt.cone=%s\n", coneName(m->opt.cone));
    std::printf("opt.jacobian=%d\n", m->opt.jacobian);
    std::printf("opt.solver=%s\n", solverName(m->opt.solver));
    std::printf("opt.iterations=%d\n", m->opt.iterations);
    std::printf("opt.ls_iterations=%d\n", m->opt.ls_iterations);
    std::printf("opt.noslip_iterations=%d\n", m->opt.noslip_iterations);
    std::printf("opt.ccd_iterations=%d\n", m->opt.ccd_iterations);
    std::printf("opt.tolerance=%.3e\n", m->opt.tolerance);
    std::printf("opt.ls_tolerance=%.3e\n", m->opt.ls_tolerance);
    std::printf("opt.noslip_tolerance=%.3e\n", m->opt.noslip_tolerance);
    std::printf("opt.ccd_tolerance=%.3e\n", m->opt.ccd_tolerance);
    std::printf("opt.gravity=%.6f,%.6f,%.6f\n", m->opt.gravity[0], m->opt.gravity[1], m->opt.gravity[2]);
    std::printf("opt.density=%.6f\n", m->opt.density);
    std::printf("opt.viscosity=%.6f\n", m->opt.viscosity);
    std::printf("opt.impratio=%.6f\n", m->opt.impratio);
    std::printf("opt.disableflags=0x%x\n", m->opt.disableflags);
    std::printf("opt.enableflags=0x%x\n", m->opt.enableflags);
    std::printf("opt.o_margin=%.6f\n", m->opt.o_margin);
    std::printf("opt.o_solref=%.6f,%.6f\n", m->opt.o_solref[0], m->opt.o_solref[1]);
    std::printf("opt.o_solimp=%.6f,%.6f,%.6f,%.6f,%.6f\n",
                m->opt.o_solimp[0], m->opt.o_solimp[1], m->opt.o_solimp[2],
                m->opt.o_solimp[3], m->opt.o_solimp[4]);

    // --- actuator control ranges (nu x 2) ---
    std::printf("actuator.nu=%d\n", m->nu);
    for (int i = 0; i < m->nu; i++)
    {
        std::printf("actuator.%d.ctrlrange=%.6f,%.6f\n", i,
                    m->actuator_ctrlrange[2 * i], m->actuator_ctrlrange[2 * i + 1]);
        std::printf("actuator.%d.forcerange=%.6f,%.6f\n", i,
                    m->actuator_forcerange[2 * i], m->actuator_forcerange[2 * i + 1]);
    }

    // --- joint ranges ---
    for (int i = 0; i < m->njnt; i++)
    {
        if (m->jnt_type[i] == mjJNT_HINGE || m->jnt_type[i] == mjJNT_SLIDE)
        {
            std::printf("joint.%d.range=%.6f,%.6f\n", i,
                        m->jnt_range[2 * i], m->jnt_range[2 * i + 1]);
        }
    }

    // --- physics stepping arithmetic check ---
    const mjtNum dt = m->opt.timestep;
    const double t0 = d->time;
    for (int i = 0; i < 10; i++) mj_step(m, d);
    const double advanced = d->time - t0;
    std::printf("step.10steps_advance=%.9f (10 * timestep = %.9f)\n", advanced, 10.0 * dt);
    std::printf("step.advance_matches= %s\n",
                (advanced > 0 && advanced > 10.0 * dt - 1e-9 && advanced < 10.0 * dt + 1e-9) ? "true" : "false");

    mj_deleteData(d);
    mj_deleteModel(m);
    return 0;
}
