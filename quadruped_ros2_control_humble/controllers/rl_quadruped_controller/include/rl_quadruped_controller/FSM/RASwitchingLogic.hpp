//
// P1-07 — RA-to-Recovery switching decision helper.
//
// Pure, dependency-free state machine boundary: no torch, no ROS, no YAML.
// This is the single place that decides whether the inline RA-based Recovery
// path is active for a given RA value and mode.  All numeric thresholds and the
// forced-hold count remain owned by the caller (config) and are passed in; this
// helper never defines its own constants.
//
// Two and only two modes exist (P1-07 contract):
//   - paper_faithful_switch: the paper rule only
//       RA >= entry_thr  -> Recovery
//       RA <  entry_thr  -> Agile
//       no hysteresis, no forced hold, equality at the threshold enters Recovery.
//   - stabilized_switch (default): current deployed behavior, preserved exactly
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
#include <string>

namespace abs_switching {

// The two explicit RA-to-Recovery switching modes.
enum class SwitchMode
{
    stabilized_switch,     // current deployed behavior (default)
    paper_faithful_switch  // paper rule only
};

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
    return false;
}

inline const char* switchModeName(SwitchMode mode)
{
    switch (mode)
    {
        case SwitchMode::stabilized_switch: return "stabilized_switch";
        case SwitchMode::paper_faithful_switch: return "paper_faithful_switch";
    }
    return "invalid";
}

// Mutable switching state held across policy steps.
struct SwitchState
{
    bool in_recovery = false;  // inline RA-based Recovery is active
    int hold_left = 0;         // forced-hold counter (stabilized mode only)
};

// Result of one policy-step switching decision.
struct SwitchDecision
{
    SwitchState state;         // updated in_recovery / hold_left
    bool enter_edge = false;   // true only on Agile -> Recovery transition
                               // (caller re-optimizes the safe twist and caches it)
    bool invalid = false;      // true only when ra_value was non-finite:
                               // state unchanged, caller must rely on fail-closed path
};

// Advance switching by one policy step.
//
// Parameters:
//   mode       - which switching rule to apply
//   cur        - switching state at the start of the step
//   ra_value   - RA model output for this step (finite in normal operation;
//                non-finite -> invalid result, no transition)
//   entry_thr  - Recovery entry threshold (-0.05 deployment)
//   exit_thr   - Recovery exit threshold (-0.08 deployment, stabilized only)
//   hold_steps - forced hold length in policy steps (stabilized only; 30 deployment)
inline SwitchDecision stepSwitching(SwitchMode mode,
                                    const SwitchState& cur,
                                    double ra_value,
                                    double entry_thr,
                                    double exit_thr,
                                    int hold_steps)
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
    }
    return d;
}

}  // namespace abs_switching

#endif  // RASWITCHINGLOGIC_HPP
