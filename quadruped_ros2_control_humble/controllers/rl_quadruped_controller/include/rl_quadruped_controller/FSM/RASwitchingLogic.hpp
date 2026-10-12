//
// RA-to-Recovery switching decision helper. Research-facing configuration is
// parsed separately by AbsSwitchConfig.hpp.
//
// Pure, dependency-free state machine boundary: no torch, no ROS, no YAML.
// This is the single place that decides whether the inline RA-based Recovery
// path is active for a given RA value and mode.  All numeric thresholds and the
// forced-hold count remain owned by the caller (config) and are passed in; this
// helper never defines its own constants.
//
// A/B/C/D research modes share this decision point; legacy modes remain for
// regression compatibility.
//   - paper_faithful_switch: the paper rule only
//       RA >= entry_thr  -> Recovery
//       RA <  entry_thr  -> Agile
//       no hysteresis, no forced hold, equality at the threshold enters Recovery.
//   - stabilized_switch: legacy helper retained for regression compatibility
//       ENTER: !in_recovery && RA >  entry_thr          (strict, no equality)
//       EXIT : in_recovery && hold expired && RA < exit_thr
//       forced hold of `hold_steps` policy steps after every ENTER.
//
// A non-finite RA (NaN/Inf) yields `invalid=true` and leaves the state
// unchanged: it can never turn into an Agile/Recovery transition.  The caller's
// existing fail-closed path (safetyVeto) remains the authoritative handling.
//
// All functions are inline so the header compiles standalone for offline tests.
//

#ifndef RASWITCHINGLOGIC_HPP
#define RASWITCHINGLOGIC_HPP

#include <cmath>
#include <cstdint>
#include <string>

namespace abs_switching {

// The research-facing A/B/C/D modes plus the retained legacy candidate.
enum class SwitchMode
{
    stabilized_switch,     // current deployed behavior (default)
    paper_faithful_switch, // group A: single threshold, no hold
    hysteresis_switch,     // group B: entry/exit thresholds, no hold
    minimum_hold_switch,   // group C: single threshold plus sim-time Recovery hold
    combined_switch        // group D: hysteresis plus sim-time Recovery hold
};

inline bool usesMinimumHold(SwitchMode mode)
{
    return mode == SwitchMode::minimum_hold_switch || mode == SwitchMode::combined_switch;
}

inline bool usesHysteresis(SwitchMode mode)
{
    return mode == SwitchMode::hysteresis_switch || mode == SwitchMode::combined_switch;
}

// Parse a config string into a mode.  Returns false for any value other than
// the two valid names; the caller must treat that as a configuration error and
// must NOT fall back to a default mode.
inline bool parseSwitchMode(const std::string& value, SwitchMode& out)
{
    if (value == "stabilized_switch")
    {
        out = SwitchMode::stabilized_switch;
        return true;
    }
    if (value == "paper_faithful_switch")
    {
        out = SwitchMode::paper_faithful_switch;
        return true;
    }
    if (value == "hysteresis_switch")
    {
        out = SwitchMode::hysteresis_switch;
        return true;
    }
    if (value == "minimum_hold_switch") { out = SwitchMode::minimum_hold_switch; return true; }
    if (value == "combined_switch") { out = SwitchMode::combined_switch; return true; }
    return false;
}

inline const char* switchModeName(SwitchMode mode)
{
    switch (mode)
    {
        case SwitchMode::stabilized_switch: return "stabilized_switch";
        case SwitchMode::paper_faithful_switch: return "paper_faithful_switch";
        case SwitchMode::hysteresis_switch: return "hysteresis_switch";
        case SwitchMode::minimum_hold_switch: return "minimum_hold_switch";
        case SwitchMode::combined_switch: return "combined_switch";
    }
    return "invalid";
}

// The paper candidate exits at the same threshold it enters; the stabilized
// candidate uses the caller's configured hysteresis threshold. Keep the
// recorder and stepSwitching() caller on this one candidate-aware value.
inline double effectiveExitThreshold(SwitchMode mode,
                                     double entry_thr,
                                     double configured_exit_thr)
{
    return mode == SwitchMode::paper_faithful_switch
        ? entry_thr : configured_exit_thr;
}

// Mutable switching state held across policy steps.
struct SwitchState
{
    bool in_recovery = false;  // inline RA-based Recovery is active
    int hold_left = 0;         // forced-hold counter (stabilized mode only)
    double recovery_enter_sim_time_s = 0.0;
    uint64_t recovery_clock_segment = 0;
    bool recovery_enter_time_valid = false;
};

enum HoldStatus : uint32_t { kHoldDisabled = 0, kHoldNotInRecovery = 1,
                             kHoldActive = 2, kHoldExpired = 3, kHoldClockInvalid = 4 };

// Result of one policy-step switching decision.
struct SwitchDecision
{
    SwitchState state;         // updated in_recovery / hold_left
    bool enter_edge = false;   // true only on Agile -> Recovery transition
                               // (caller re-optimizes the safe twist and caches it)
    bool invalid = false;      // true only when ra_value was non-finite:
    HoldStatus hold_status = kHoldDisabled;
    double hold_elapsed_s = 0.0;
                               // state unchanged, caller must rely on fail-closed path
};

// Advance switching by one policy step.
//
// Parameters:
//   mode       - which switching rule to apply
//   cur        - switching state at the start of the step
//   ra_value   - RA model output for this step (finite in normal operation;
//                non-finite -> invalid result, no transition)
//   entry_thr  - Recovery entry threshold
//   exit_thr   - legacy helper exit threshold (stabilized mode only)
//   hold_steps - legacy forced hold length in policy steps (stabilized mode only)
inline SwitchDecision stepSwitching(SwitchMode mode,
                                    const SwitchState& cur,
                                    double ra_value,
                                    double entry_thr,
                                    double exit_thr,
                                    int hold_steps,
                                    double sim_time_s = 0.0,
                                    bool sim_clock_valid = false,
                                    uint64_t sim_clock_segment = 0,
                                    double recovery_min_hold_s = 0.0)
{
    SwitchDecision d;
    d.state = cur;

    // Non-finite RA must never drive a transition.  The caller's fail-closed
    // path is authoritative; here we simply refuse to decide.
    if (!std::isfinite(ra_value))
    {
        d.invalid = true;
        return d;
    }

    switch (mode)
    {
        case SwitchMode::paper_faithful_switch:
        {
            // Paper rule, stateless per step: RA >= threshold -> Recovery,
            // RA < threshold -> Agile.  No hysteresis, no forced hold; an
            // in-recovery state with a non-zero hold counter is irrelevant and
            // cannot delay an exit.
            const bool want_recovery = (ra_value >= entry_thr);
            const bool was_recovery = cur.in_recovery;
            d.state.in_recovery = want_recovery;
            d.state.hold_left = 0;  // paper has no forced hold
            d.enter_edge = want_recovery && !was_recovery;
            break;
        }
        case SwitchMode::hysteresis_switch:
        {
            // Group B adds only hysteresis: enter at RA >= E; while in
            // Recovery, exit only at RA < X. Equality at X retains Recovery.
            if (!cur.in_recovery && ra_value >= entry_thr)
            {
                d.state.in_recovery = true;
                d.state.hold_left = 0;
                d.enter_edge = true;
            }
            else if (cur.in_recovery && ra_value < exit_thr)
            {
                d.state.in_recovery = false;
                d.state.hold_left = 0;
            }
            else
            {
                d.state.hold_left = 0;
            }
            break;
        }
        case SwitchMode::stabilized_switch:
        {
            // Exact mirror of the current deployment block (StateRL.cpp
            // ENTER/EXIT): strict > entry, unconditional hold decrement every
            // step spent in Recovery, exit only after hold expiry AND < exit.
            if (!cur.in_recovery && ra_value > entry_thr)
            {
                d.state.in_recovery = true;
                d.state.hold_left = hold_steps;
                d.enter_edge = true;
            }
            else if (cur.in_recovery)
            {
                d.state.hold_left -= 1;  // current code decrements unconditionally
                if (d.state.hold_left <= 0 && ra_value < exit_thr)
                {
                    d.state.in_recovery = false;
                }
            }
            break;
        }
        case SwitchMode::minimum_hold_switch:
        case SwitchMode::combined_switch:
        {
            if (!cur.in_recovery)
            {
                if (ra_value >= entry_thr && sim_clock_valid && std::isfinite(sim_time_s))
                {
                    d.state.in_recovery = true;
                    d.state.recovery_enter_sim_time_s = sim_time_s;
                    d.state.recovery_clock_segment = sim_clock_segment;
                    d.state.recovery_enter_time_valid = true;
                    d.enter_edge = true;
                    d.hold_status = kHoldActive;
                }
                else if (ra_value >= entry_thr)
                    d.hold_status = kHoldClockInvalid;
                else
                    d.hold_status = kHoldNotInRecovery;
            }
            else
            {
                const bool time_ok = sim_clock_valid && std::isfinite(sim_time_s) &&
                    cur.recovery_enter_time_valid &&
                    sim_clock_segment == cur.recovery_clock_segment &&
                    sim_time_s >= cur.recovery_enter_sim_time_s;
                if (!time_ok)
                {
                    d.hold_status = kHoldClockInvalid;
                }
                else
                {
                    d.hold_elapsed_s = sim_time_s - cur.recovery_enter_sim_time_s;
                    d.hold_status = d.hold_elapsed_s >= recovery_min_hold_s ? kHoldExpired : kHoldActive;
                    const double exit_threshold = mode == SwitchMode::combined_switch ? exit_thr : entry_thr;
                    if (d.hold_status == kHoldExpired && ra_value < exit_threshold)
                    {
                        d.state.in_recovery = false;
                        d.state.recovery_enter_time_valid = false;
                    }
                }
            }
            break;
        }
    }
    return d;
}

}  // namespace abs_switching

#endif  // RASWITCHINGLOGIC_HPP
