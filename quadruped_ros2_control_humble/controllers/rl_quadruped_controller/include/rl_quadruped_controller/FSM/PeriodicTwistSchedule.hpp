#ifndef PERIODIC_TWIST_SCHEDULE_HPP
#define PERIODIC_TWIST_SCHEDULE_HPP

#include <cstdint>

namespace abs_twist_update {

constexpr double kPeriodicPeriodS = 0.06;

enum class ScheduleResult { not_due, due, clock_rebased };

// One Recovery-local simulation-clock deadline. The caller supplies only
// already-validated simulation-clock samples and calls markUpdated() after a
// successful optimizer write.
class PeriodicSchedule {
public:
    void reset()
    {
        active_ = false;
        segment_id_ = 0;
        last_observed_s_ = 0.0;
        deadline_s_ = 0.0;
        last_update_s_ = 0.0;
    }

    void begin(double sim_time_s, uint64_t segment_id)
    {
        active_ = true;
        segment_id_ = segment_id;
        last_observed_s_ = sim_time_s;
        last_update_s_ = sim_time_s;
        deadline_s_ = sim_time_s + kPeriodicPeriodS;
    }

    ScheduleResult check(double sim_time_s, uint64_t segment_id)
    {
        if (!active_ || segment_id != segment_id_ || sim_time_s < last_observed_s_)
        {
            begin(sim_time_s, segment_id);
            return ScheduleResult::clock_rebased;
        }
        last_observed_s_ = sim_time_s;
        return sim_time_s >= deadline_s_ ? ScheduleResult::due : ScheduleResult::not_due;
    }

    void markUpdated(double sim_time_s, uint64_t segment_id)
    {
        segment_id_ = segment_id;
        last_observed_s_ = sim_time_s;
        last_update_s_ = sim_time_s;
        deadline_s_ = sim_time_s + kPeriodicPeriodS;
        active_ = true;
    }

    double elapsedSinceUpdate(double sim_time_s) const { return sim_time_s - last_update_s_; }

private:
    bool active_ = false;
    uint64_t segment_id_ = 0;
    double last_observed_s_ = 0.0;
    double last_update_s_ = 0.0;
    double deadline_s_ = 0.0;
};

}  // namespace abs_twist_update

#endif  // PERIODIC_TWIST_SCHEDULE_HPP
