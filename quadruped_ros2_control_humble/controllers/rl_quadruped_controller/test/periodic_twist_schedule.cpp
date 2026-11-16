#include "rl_quadruped_controller/FSM/PeriodicTwistSchedule.hpp"

#include <cstdio>

int main()
{
    int failures = 0;
    const auto check = [&failures](bool ok, const char* label) {
        std::printf("%s: %s\n", ok ? "PASS" : "FAIL", label);
        if (!ok) ++failures;
    };
    using abs_twist_update::PeriodicSchedule;
    using abs_twist_update::ScheduleResult;

    PeriodicSchedule schedule;
    schedule.begin(0.0, 1);
    check(schedule.check(0.059, 1) == ScheduleResult::not_due, "before deadline does not update");
    check(schedule.check(0.06, 1) == ScheduleResult::due, "first deadline updates on due cycle");
    schedule.markUpdated(0.06, 1);
    check(schedule.check(0.24, 1) == ScheduleResult::due, "multi-period jump yields one due update");
    schedule.markUpdated(0.24, 1);
    check(schedule.check(0.299, 1) == ScheduleResult::not_due, "next deadline restarts from successful update");
    check(schedule.check(0.30, 1) == ScheduleResult::due, "next successful interval is due at 0.06");

    schedule.reset();  // caller resets after Recovery exits; no schedule is active while Agile.
    check(schedule.check(0.30, 1) == ScheduleResult::clock_rebased,
          "exit reset prevents a stale deadline from updating");
    schedule.begin(4.0, 1);
    check(schedule.check(4.059, 1) == ScheduleResult::not_due, "re-entry starts a fresh interval");
    check(schedule.check(4.06, 1) == ScheduleResult::due, "re-entry interval reaches its deadline");

    check(schedule.check(9.0, 2) == ScheduleResult::clock_rebased, "clock segment change rebases");
    check(schedule.check(9.059, 2) == ScheduleResult::not_due, "rebased segment waits a full interval");
    check(schedule.check(9.06, 2) == ScheduleResult::due, "rebased segment updates at its deadline");
    schedule.markUpdated(9.06, 2);
    check(schedule.check(8.0, 2) == ScheduleResult::clock_rebased, "backward sim time rebases");
    check(schedule.check(8.059, 2) == ScheduleResult::not_due, "reset clock waits a full interval");
    check(schedule.check(8.06, 2) == ScheduleResult::due, "reset clock updates at its new deadline");

    return failures == 0 ? 0 : 1;
}
