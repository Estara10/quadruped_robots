#ifndef NAVIGATION_STATE_SOURCE_HPP
#define NAVIGATION_STATE_SOURCE_HPP

#include <array>
#include <cmath>
#include <cstdint>
#include <mutex>
#include <stdexcept>
#include <string>

// Explicit selection of the navigation/velocity state source consumed by ABS.
//
// ODOM      : current default. MuJoCo odometer sensor, or SportModeState
//             position/velocity on hardware (world-frame velocity, world Z is
//             height). Unchanged behaviour.
// VINS_BODY : independent source (go2_body_odometry). pose is base-in-world,
//             twist is ALREADY base-frame. Must never be rotated again.
//
// This header holds no ROS, SDK or torch dependency so the selection and frame
// rules can be checked offline without DDS, a device or a policy.
namespace abs_navigation_state {

enum class Source
{
    ODOM,
    VINS_BODY
};

inline Source parseSource(const std::string& name)
{
    if (name == "odom") return Source::ODOM;
    if (name == "vins_body") return Source::VINS_BODY;
    throw std::invalid_argument("navigation_state_source must be 'odom' or 'vins_body'");
}

inline const char* sourceName(Source source)
{
    return source == Source::ODOM ? "odom" : "vins_body";
}

// One independent-source sample.
// position / quaternion (xyzw): base reference point in the source world frame.
// linear_velocity / angular_velocity: expressed in the base body frame.
// stamp_ns: source measurement stamp (communication age is measured separately
// from arrival_ns, which is the local monotonic receipt time).
struct Sample
{
    int64_t stamp_ns = 0;
    uint64_t arrival_ns = 0;
    std::array<double, 3> position{};
    std::array<double, 4> quaternion{0.0, 0.0, 0.0, 1.0};
    std::array<double, 3> linear_velocity{};
    std::array<double, 3> angular_velocity{};
};

inline bool finite(const Sample& sample)
{
    for (const double value : sample.position) if (!std::isfinite(value)) return false;
    for (const double value : sample.quaternion) if (!std::isfinite(value)) return false;
    for (const double value : sample.linear_velocity) if (!std::isfinite(value)) return false;
    for (const double value : sample.angular_velocity) if (!std::isfinite(value)) return false;
    if (sample.stamp_ns <= 0) return false;
    if (sample.arrival_ns == 0) return false;
    const double norm = sample.quaternion[0] * sample.quaternion[0] + sample.quaternion[1] * sample.quaternion[1] +
                        sample.quaternion[2] * sample.quaternion[2] + sample.quaternion[3] * sample.quaternion[3];
    return std::isfinite(norm) && norm > 1e-6;
}

inline double wrapPi(double angle)
{
    return std::atan2(std::sin(angle), std::cos(angle));
}

// Heading of the base in the source world frame (REP-103, z up), from xyzw.
inline double sourceYaw(const Sample& sample)
{
    const double x = sample.quaternion[0];
    const double y = sample.quaternion[1];
    const double z = sample.quaternion[2];
    const double w = sample.quaternion[3];
    return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}

// Navigation frame: origin captured once when an RL session starts; its x axis
// is the base heading at that instant. Only the navigation consumers use it;
// LowState attitude still feeds gravity and the body quaternion observation.
struct Origin
{
    bool captured = false;
    std::array<double, 2> position{};
    double yaw = 0.0;
};

inline void captureOrigin(const Sample& sample, Origin& origin)
{
    origin.captured = true;
    origin.position = {sample.position[0], sample.position[1]};
    origin.yaw = sourceYaw(sample);
}

// Rigid transform of one sample into the navigation frame. Position and yaw come
// from the same transform, so they cannot disagree about the world frame.
inline bool toNavigationFrame(const Sample& sample, const Origin& origin,
                             std::array<double, 2>& position, double& yaw)
{
    if (!origin.captured || !finite(sample)) return false;
    const double c = std::cos(origin.yaw);
    const double s = std::sin(origin.yaw);
    const double dx = sample.position[0] - origin.position[0];
    const double dy = sample.position[1] - origin.position[1];
    position = {c * dx + s * dy, -s * dx + c * dy};
    yaw = wrapPi(sourceYaw(sample) - origin.yaw);
    return std::isfinite(position[0]) && std::isfinite(position[1]) && std::isfinite(yaw);
}

// Thread-safe mailbox between the executor subscription and the controller
// update thread. Holds only the latest sample; no interpolation, no zero fill,
// no fallback to another source.
//
// A detected time regression latches the mailbox invalid. New messages cannot
// clear it, so a message that arrives between two control cycles cannot hide the
// anomaly (100 -> 90 -> 91 stays unusable). Recovery requires an explicit new
// navigation session, started by the owner on RL entry.
class Mailbox
{
public:
    void store(const Sample& sample)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (tripped_)
        {
            ++rejected_after_trip_;
            return;
        }
        if (have_latest_ && sample.stamp_ns == latest_.stamp_ns)
        {
            // Exact duplicate of the stored measurement: drop it and, crucially,
            // do not refresh the arrival time, so the same old measurement can
            // not be kept alive indefinitely.
            ++duplicate_rejections_;
            return;
        }
        if (have_latest_ && sample.stamp_ns < latest_.stamp_ns)
        {
            // Source clock/frame regression: latch invalid until an explicit new
            // session. A later forward message must NOT re-enable the source,
            // because the navigation origin captured for the previous run may no
            // longer belong to the published world frame.
            ++stamp_rejections_;
            tripped_ = true;
            have_latest_ = false;
            return;
        }
        latest_ = sample;
        have_latest_ = true;
        ++accepted_;
    }

    // Communication-age check plus first-data check. Returns false when the
    // source is missing, stale, non-finite or latched invalid.
    bool fresh(uint64_t now_ns, uint64_t timeout_ns, Sample& out) const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (tripped_) return false;
        if (!have_latest_ || timeout_ns == 0 || latest_.arrival_ns == 0) return false;
        if (now_ns < latest_.arrival_ns) return false;
        if (now_ns - latest_.arrival_ns > timeout_ns) return false;
        if (!finite(latest_)) return false;
        out = latest_;
        return true;
    }

    // Clears the current sample only; a latched time regression stays latched.
    void invalidate()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        have_latest_ = false;
    }

    // Explicit new navigation session. Called once per RL entry by the owner,
    // never from a message callback. The controller resets its navigation origin
    // in the same transition, so the new source run gets a new origin instead of
    // inheriting the previous one.
    void beginSession()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        tripped_ = false;
        ++sessions_;
    }

    bool tripped() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return tripped_;
    }

    uint64_t accepted() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return accepted_;
    }

    uint64_t stampRejections() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return stamp_rejections_;
    }

    uint64_t duplicateRejections() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return duplicate_rejections_;
    }

    uint64_t rejectedAfterTrip() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return rejected_after_trip_;
    }

    uint64_t sessions() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return sessions_;
    }

private:
    mutable std::mutex mutex_;
    Sample latest_{};
    bool have_latest_ = false;
    bool tripped_ = false;
    uint64_t accepted_ = 0;
    uint64_t stamp_rejections_ = 0;
    uint64_t duplicate_rejections_ = 0;
    uint64_t rejected_after_trip_ = 0;
    uint64_t sessions_ = 0;
};

// Only the odometer path reports a world-frame velocity that has to be rotated
// into the base frame. The independent source already reports base-frame twist.
inline bool requiresWorldToBodyRotation(Source source)
{
    return source == Source::ODOM;
}

// Fetch the currently selected independent sample. A non-independent source
// never yields a sample here, so the odometer can never stand in for it.
inline bool selectSample(Source source, const Mailbox* mailbox, uint64_t now_ns,
                        uint64_t timeout_ns, Sample& out)
{
    if (source != Source::VINS_BODY || mailbox == nullptr) return false;
    return mailbox->fresh(now_ns, timeout_ns, out);
}

}  // namespace abs_navigation_state

#endif  // NAVIGATION_STATE_SOURCE_HPP
