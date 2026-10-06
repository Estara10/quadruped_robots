//
// P1-07 — RA-to-Recovery switching truth-table tests.
//
// Offline, deterministic. Exercises the pure helper
// (RASwitchingLogic.hpp) for BOTH modes with the SAME RA sequences and at all
// threshold equalities. No libtorch, no ROS, no MuJoCo: the helper is a
// dependency-free state-machine boundary, so this executable links nothing but
// the C++ standard library.
//
// Reference mirror: `reference_stabilized` below is the literal ENTER/EXIT
// logic that lived in StateRL.cpp before P1-07. The stabilized regression
// asserts that the helper reproduces that exact old state sequence.
//

#include "rl_quadruped_controller/FSM/RASwitchingLogic.hpp"

#include <cmath>
#include <cstdio>
#include <limits>
#include <string>

namespace abs_switching {
// (helper functions are inline in the header — nothing to link here)
}

using abs_switching::SwitchMode;
using abs_switching::SwitchState;
using abs_switching::stepSwitching;

// Deployment thresholds/config (abs/config.yaml).
static const double ENTRY = -0.05;
static const double EXIT = -0.08;
static const int HOLD = 30;

static int g_checks = 0;
static bool g_fail = false;

#define CHECK(cond)                                                     \
    do {                                                                \
        ++g_checks;                                                     \
        if (!(cond)) {                                                  \
            g_fail = true;                                              \
            std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); \
        }                                                               \
    } while (0)

// Literal mirror of the pre-P1-07 deployed switching block
// (StateRL.cpp ENTER/EXIT): strict > ENTRY, unconditional hold decrement,
// exit only after hold expiry AND RA < EXIT.
static void reference_stabilized(bool& in_recovery, int& hold_left, double ra)
{
    if (!in_recovery && ra > ENTRY)
    {
        in_recovery = true;
        hold_left = HOLD;
    }
    else if (in_recovery)
    {
        hold_left--;
        if (hold_left <= 0 && ra < EXIT)
        {
            in_recovery = false;
        }
    }
}

int main()
{
    // ------------------------------------------------------------------
    // A. paper_faithful_switch truth table
    // ------------------------------------------------------------------
    {
        // A1: RA == -0.05 enters Recovery (paper equality -> Recovery).
        auto d = stepSwitching(SwitchMode::paper_faithful_switch, {false, 0},
                               -0.05, ENTRY, EXIT, HOLD);
        CHECK(d.state.in_recovery == true);
        CHECK(d.enter_edge == true);
        CHECK(d.invalid == false);

        // A2: RA > -0.05 enters Recovery.
        d = stepSwitching(SwitchMode::paper_faithful_switch, {false, 0},
                          0.0, ENTRY, EXIT, HOLD);
        CHECK(d.state.in_recovery == true);
        CHECK(d.enter_edge == true);

        // A3: RA < -0.05 selects Agile immediately (from agile).
        d = stepSwitching(SwitchMode::paper_faithful_switch, {false, 0},
                          -0.10, ENTRY, EXIT, HOLD);
        CHECK(d.state.in_recovery == false);
        CHECK(d.enter_edge == false);

        // A3b: RA < -0.05 exits immediately even while in Recovery with an
        // active hold counter (no forced hold, no hysteresis).
        d = stepSwitching(SwitchMode::paper_faithful_switch, {true, HOLD},
                          -0.10, ENTRY, EXIT, HOLD);
        CHECK(d.state.in_recovery == false);
        CHECK(d.state.hold_left == 0);

        // A4: no hold state can delay exit — in Recovery with full hold,
        // a sub-threshold RA leaves Recovery on the same step.
        d = stepSwitching(SwitchMode::paper_faithful_switch, {true, HOLD},
                          -0.06, ENTRY, EXIT, HOLD);
        CHECK(d.state.in_recovery == false);

        // A5: stay in Recovery while RA >= threshold; no repeated entry edge.
        d = stepSwitching(SwitchMode::paper_faithful_switch, {false, 0},
                          -0.05, ENTRY, EXIT, HOLD);
        CHECK(d.state.in_recovery == true && d.enter_edge == true);
        d = stepSwitching(SwitchMode::paper_faithful_switch, d.state,
                          -0.05, ENTRY, EXIT, HOLD);
        CHECK(d.state.in_recovery == true && d.enter_edge == false);

        // A6: recovery is stateless per step — exit then immediate re-entry.
        d = stepSwitching(SwitchMode::paper_faithful_switch, {false, 0},
                          -0.05, ENTRY, EXIT, HOLD);           // enter
        d = stepSwitching(SwitchMode::paper_faithful_switch, d.state,
                          -0.06, ENTRY, EXIT, HOLD);           // exit
        CHECK(d.state.in_recovery == false);
        d = stepSwitching(SwitchMode::paper_faithful_switch, d.state,
                          0.02, ENTRY, EXIT, HOLD);            // re-enter
        CHECK(d.state.in_recovery == true && d.enter_edge == true);
    }

    // ------------------------------------------------------------------
    // B. stabilized_switch truth table
    // ------------------------------------------------------------------
    {
        // B1: RA == -0.05 does NOT enter (strict >).
        auto d = stepSwitching(SwitchMode::stabilized_switch, {false, 0},
                               -0.05, ENTRY, EXIT, HOLD);
        CHECK(d.state.in_recovery == false);
        CHECK(d.enter_edge == false);

        // B2: RA > -0.05 enters and arms the forced hold.
        d = stepSwitching(SwitchMode::stabilized_switch, {false, 0},
                          0.0, ENTRY, EXIT, HOLD);
        CHECK(d.state.in_recovery == true);
        CHECK(d.enter_edge == true);
        CHECK(d.state.hold_left == HOLD);

        // B3: before hold expiry, an arbitrarily safe RA cannot exit.
        d = stepSwitching(SwitchMode::stabilized_switch, d.state,
                          -0.9, ENTRY, EXIT, HOLD);
        CHECK(d.state.in_recovery == true);
        CHECK(d.state.hold_left == HOLD - 1);

        // B4: after hold expiry, RA == -0.08 does NOT exit (strict <).
        d = stepSwitching(SwitchMode::stabilized_switch, {true, 1},
                          -0.08, ENTRY, EXIT, HOLD);
        CHECK(d.state.hold_left == 0);
        CHECK(d.state.in_recovery == true);

        // B5: after hold expiry, RA < -0.08 exits.
        d = stepSwitching(SwitchMode::stabilized_switch, {true, 1},
                          -0.09, ENTRY, EXIT, HOLD);
        CHECK(d.state.in_recovery == false);

        // B6: -0.05 < RA < -0.08 boundary sequences match current code.
        //   agile + -0.06  -> stays agile (not > ENTRY).
        d = stepSwitching(SwitchMode::stabilized_switch, {false, 0},
                          -0.06, ENTRY, EXIT, HOLD);
        CHECK(d.state.in_recovery == false);
        //   recovery (hold expired) + -0.06 -> stays in recovery (not < EXIT).
        d = stepSwitching(SwitchMode::stabilized_switch, {true, 0},
                          -0.06, ENTRY, EXIT, HOLD);
        CHECK(d.state.in_recovery == true);
        //   recovery (hold expired) + -0.06 -> hold counter keeps drifting
        //   negative exactly like current code.
        CHECK(d.state.hold_left == -1);
        //   already in recovery + RA > ENTRY does not re-arm the hold (current
        //   code's ENTER branch requires !in_recovery_).
        d = stepSwitching(SwitchMode::stabilized_switch, {true, 0},
                          0.2, ENTRY, EXIT, HOLD);
        CHECK(d.state.in_recovery == true);
        CHECK(d.state.hold_left == -1);
        CHECK(d.enter_edge == false);
    }

    // B7: stabilized default regression — the helper must reproduce the exact
    // pre-P1-07 state sequence over a deterministic RA series that crosses
    // every threshold equality. The same series is also run under the paper
    // mode against the stateless paper oracle (same RA sequences, both modes).
    {
        // clang-format off
        const double seq[] = {
             0.00, -0.90, -0.90, -0.06, -0.08, -0.09,  0.05, -0.07, -0.06,
            -0.20, -0.05,  0.00, -0.04, -0.06, -0.08, -0.10,  0.10, -0.05,
            -0.049999999999, -0.050000001, -0.079999999999, -0.080000001,
            -0.9, 0.0, -0.05, 0.0, -0.9, 0.0, -0.09, -0.05, -0.08, -0.09
        };
        // clang-format on
        const int N = static_cast<int>(sizeof(seq) / sizeof(seq[0]));

        // stabilized vs reference mirror of current code
        bool ref_in = false;
        int ref_hold = 0;
        SwitchState helper{false, 0};
        // paper mode, same sequence, stateless oracle
        SwitchState paper{false, 0};

        for (int i = 0; i < N; ++i)
        {
            const double ra = seq[i];

            auto ds = stepSwitching(SwitchMode::stabilized_switch, helper,
                                    ra, ENTRY, EXIT, HOLD);
            CHECK(ds.invalid == false);
            helper = ds.state;
            reference_stabilized(ref_in, ref_hold, ra);
            CHECK(helper.in_recovery == ref_in);
            CHECK(helper.hold_left == ref_hold);

            const bool was_paper = paper.in_recovery;
            auto dp = stepSwitching(SwitchMode::paper_faithful_switch, paper,
                                    ra, ENTRY, EXIT, HOLD);
            CHECK(dp.invalid == false);
            paper = dp.state;
            const bool paper_oracle = (ra >= ENTRY);
            CHECK(paper.in_recovery == paper_oracle);
            CHECK(dp.enter_edge == (paper_oracle && !was_paper));
            CHECK(paper.hold_left == 0);
        }

        // Initial default state is exactly the deployment initial members:
        // in_recovery_=false, rec_hold_left_=0.
        SwitchState init;
        CHECK(init.in_recovery == false);
        CHECK(init.hold_left == 0);
    }

    // ------------------------------------------------------------------
    // C. safety / configuration
    // ------------------------------------------------------------------
    {
        // C1: invalid switch-mode strings are rejected (no silent fallback).
        SwitchMode parsed = SwitchMode::paper_faithful_switch;
        CHECK(abs_switching::parseSwitchMode("bogus", parsed) == false);
        CHECK(abs_switching::parseSwitchMode("", parsed) == false);
        CHECK(abs_switching::parseSwitchMode("stabilized", parsed) == false);
        CHECK(abs_switching::parseSwitchMode("paper_faithful", parsed) == false);
        CHECK(abs_switching::parseSwitchMode("PAPER_FAITHFUL_SWITCH", parsed) == false);
        CHECK(abs_switching::parseSwitchMode("Stabilized_switch", parsed) == false);
        // parsed must not have been modified by a failed parse
        CHECK(parsed == SwitchMode::paper_faithful_switch);

        // C2: the two valid names parse and round-trip.
        CHECK(abs_switching::parseSwitchMode("stabilized_switch", parsed) == true);
        CHECK(parsed == SwitchMode::stabilized_switch);
        CHECK(std::string(abs_switching::switchModeName(parsed)) == "stabilized_switch");
        CHECK(abs_switching::parseSwitchMode("paper_faithful_switch", parsed) == true);
        CHECK(parsed == SwitchMode::paper_faithful_switch);
        CHECK(std::string(abs_switching::switchModeName(parsed)) == "paper_faithful_switch");

        // C3: the default selection is stabilized (enum value 0; StateRL.h
        // member init = abs_switching::SwitchMode::stabilized_switch).
        SwitchMode def{};
        CHECK(def == SwitchMode::stabilized_switch);

        // C4: non-finite RA never becomes an Agile/Recovery transition.
        //     1) from agile: NaN/±Inf must NOT enter Recovery;
        //     2) from recovery: NaN/±Inf must NOT exit (and must not be
        //        treated as "safe"); the caller's fail-closed path is the
        //        authoritative handling and remains untouched.
        const double nan = std::numeric_limits<double>::quiet_NaN();
        const double pinf = std::numeric_limits<double>::infinity();
        const double ninf = -pinf;

        for (int m = 0; m < 2; ++m)
        {
            const SwitchMode mode = (m == 0) ? SwitchMode::stabilized_switch
                                             : SwitchMode::paper_faithful_switch;
            // from agile
            auto d = stepSwitching(mode, {false, 0}, nan, ENTRY, EXIT, HOLD);
            CHECK(d.invalid == true);
            CHECK(d.state.in_recovery == false);
            CHECK(d.state.hold_left == 0);
            CHECK(d.enter_edge == false);
            d = stepSwitching(mode, {false, 0}, pinf, ENTRY, EXIT, HOLD);
            CHECK(d.invalid == true && d.state.in_recovery == false);
            d = stepSwitching(mode, {false, 0}, ninf, ENTRY, EXIT, HOLD);
            CHECK(d.invalid == true && d.state.in_recovery == false);
            // from recovery (hold expired): must not "exit" on NaN/Inf
            d = stepSwitching(mode, {true, 0}, nan, ENTRY, EXIT, HOLD);
            CHECK(d.invalid == true && d.state.in_recovery == true);
            d = stepSwitching(mode, {true, 0}, pinf, ENTRY, EXIT, HOLD);
            CHECK(d.invalid == true && d.state.in_recovery == true);
            d = stepSwitching(mode, {true, 0}, ninf, ENTRY, EXIT, HOLD);
            CHECK(d.invalid == true && d.state.in_recovery == true);
            // from recovery (hold active): NaN must not force an exit
            d = stepSwitching(mode, {true, 5}, nan, ENTRY, EXIT, HOLD);
            CHECK(d.invalid == true && d.state.in_recovery == true && d.state.hold_left == 5);
        }

        // C5: finite RA in both modes always yields invalid == false.
        auto d = stepSwitching(SwitchMode::stabilized_switch, {false, 0},
                               -0.05, ENTRY, EXIT, HOLD);
        CHECK(d.invalid == false);
        d = stepSwitching(SwitchMode::paper_faithful_switch, {false, 0},
                          0.1, ENTRY, EXIT, HOLD);
        CHECK(d.invalid == false);
    }

    if (g_fail)
    {
        std::printf("RESULT: FAIL (%d/%d checks failed)\n", g_fail ? 1 : 0, g_checks);
        return 1;
    }
    std::printf("RESULT: PASS (%d checks)\n", g_checks);
    return 0;
}
