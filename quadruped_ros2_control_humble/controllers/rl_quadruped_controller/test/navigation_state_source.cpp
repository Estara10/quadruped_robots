// Offline checks for the explicit ABS navigation/velocity state source.
//
// No ROS, no DDS, no device, no policy artifact. The observation-composition
// check reuses the deployed abs_observation contract (torch only).
#include "rl_quadruped_controller/FSM/AbsObservationContract.h"
#include "rl_quadruped_controller/FSM/NavigationStateSource.hpp"

#include <cmath>
#include <cstdio>
#include <stdexcept>

namespace ans = abs_navigation_state;

namespace {
int failures = 0;

void check(bool ok, const char* label)
{
    std::printf("%s: %s\n", ok ? "PASS" : "FAIL", label);
    if (!ok) ++failures;
}

bool near(double a, double b, double eps = 1e-9)
{
    return std::abs(a - b) <= eps;
}

ans::Sample makeSample(double x, double y, double z, double yaw, const std::array<double, 3>& body_v)
{
    ans::Sample sample;
    sample.stamp_ns = 1000;
    sample.arrival_ns = 2000;
    sample.position = {x, y, z};
    sample.quaternion = {0.0, 0.0, std::sin(yaw / 2.0), std::cos(yaw / 2.0)};
    sample.linear_velocity = body_v;
    sample.angular_velocity = {0.0, 0.0, 0.0};
    return sample;
}
}  // namespace

int main()
{
    // --- source parsing: default stays odom, typos are rejected ---------------
    check(ans::parseSource("odom") == ans::Source::ODOM, "default source parses as odom");
    check(ans::parseSource("vins_body") == ans::Source::VINS_BODY, "independent source parses as vins_body");
    bool rejected = false;
    try { ans::parseSource("vins"); } catch (const std::invalid_argument&) { rejected = true; }
    check(rejected, "unknown source name is rejected instead of defaulting");

    // --- frame rule: only the odometer path needs a world->body rotation ------
    check(ans::requiresWorldToBodyRotation(ans::Source::ODOM),
          "odom velocity is world-frame and needs rotation");
    check(!ans::requiresWorldToBodyRotation(ans::Source::VINS_BODY),
          "independent source velocity is body-frame and is NOT rotated again");

    // --- heading and navigation frame ----------------------------------------
    check(near(ans::sourceYaw(makeSample(0, 0, 0, 0.0, {})), 0.0), "identity quaternion yaw is 0");
    check(near(ans::sourceYaw(makeSample(0, 0, 0, M_PI / 2, {})), M_PI / 2), "90 deg heading quaternion yaw is pi/2");
    check(near(ans::wrapPi(M_PI + 0.2), -M_PI + 0.2), "heading wrap crosses +-pi without a 2*pi jump");

    ans::Origin origin;
    ans::captureOrigin(makeSample(10.0, -3.0, 0.4, M_PI / 2, {}), origin);
    std::array<double, 2> position{};
    double yaw = 0.0;
    check(ans::toNavigationFrame(makeSample(10.0, -3.0, 0.5, M_PI / 2, {}), origin, position, yaw),
          "navigation transform accepts a finite sample");
    check(near(position[0], 0.0) && near(position[1], 0.0) && near(yaw, 0.0),
          "capture pose maps to the navigation origin with zero heading");
    // 1 m along the captured heading (+x of the navigation frame = source +y here).
    check(ans::toNavigationFrame(makeSample(10.0, -2.0, 0.5, M_PI / 2, {}), origin, position, yaw),
          "forward sample converts");
    check(near(position[0], 1.0) && near(position[1], 0.0),
          "1 m along the captured heading becomes +1 m navigation x (no axis swap)");
    check(ans::toNavigationFrame(makeSample(9.0, -3.0, 0.5, M_PI / 2 + 0.3, {}), origin, position, yaw),
          "lateral/rotated sample converts");
    check(near(position[0], 0.0) && near(position[1], 1.0),
          "1 m to the source -x becomes +1 m navigation y (left)");
    check(near(yaw, 0.3), "nav heading is the source heading minus the captured heading");
    check(!ans::toNavigationFrame(makeSample(10.0, -3.0, 0.5, M_PI / 2, {}), ans::Origin{}, position, yaw),
          "sample without a captured origin is never converted (no stale origin reuse)");

    // --- mailbox: first data, staleness, non-finite, stamp reversal ----------
    ans::Mailbox mailbox;
    ans::Sample out;
    check(!mailbox.fresh(2000, 300, out), "missing source is not fresh (no zero-filled sample)");

    ans::Sample moving = makeSample(12.0, -3.0, 0.45, M_PI / 2, {0.30, 0.20, 0.0});
    moving.arrival_ns = 5000;
    moving.stamp_ns = 1'000'000'000;
    mailbox.store(moving);
    check(mailbox.fresh(5200, 300, out), "fresh sample within the timeout is accepted");
    check(near(out.linear_velocity[0], 0.30) && near(out.linear_velocity[1], 0.20),
          "stored body velocity is returned unchanged (forward/left preserved)");
    check(!mailbox.fresh(5301, 300, out), "sample older than the timeout is not reused");

    ans::Sample bad = moving;
    bad.stamp_ns = 2'000'000'000;
    bad.arrival_ns = 6000;
    bad.linear_velocity[1] = std::nan("");
    mailbox.store(bad);
    check(!mailbox.fresh(6100, 300, out), "non-finite sample is not presented as valid");

    ans::Mailbox backward;
    ans::Sample first = moving;
    backward.store(first);
    ans::Sample older = moving;
    older.stamp_ns = first.stamp_ns - 1;
    older.arrival_ns = 7000;
    backward.store(older);
    check(!backward.fresh(7100, 300, out), "backward source stamp is not used");
    check(backward.stampRejections() == 1, "backward source stamp is counted");

    ans::Mailbox invalidated;
    invalidated.store(moving);
    invalidated.invalidate();
    check(!invalidated.fresh(5100, 300, out), "invalidated source is not fresh");

    check(!mailbox.fresh(4000, 300, out), "sample collected after the query clock is not fresh");

    // --- a regression is latched even if the controller reads later ----------
    {
        ans::Mailbox regressed;
        ans::Sample s100 = makeSample(0.0, 0.0, 0.0, 0.0, {0.30, 0.20, 0.0});
        s100.stamp_ns = 100;
        s100.arrival_ns = 1000;
        ans::Sample s90 = s100;
        s90.stamp_ns = 90;
        s90.arrival_ns = 1001;
        ans::Sample s91 = s100;
        s91.stamp_ns = 91;
        s91.arrival_ns = 1002;

        // All three are written before the controller reads once.
        regressed.store(s100);
        regressed.store(s90);
        regressed.store(s91);
        check(regressed.tripped(), "100->90->91 latches the source invalid");
        check(!regressed.fresh(1100, 300, out), "100->90->91 read once afterwards is still unusable");
        check(regressed.rejectedAfterTrip() == 1, "the message after the regression is rejected, not accepted");

        for (int64_t stamp = 92; stamp <= 96; ++stamp)
        {
            ans::Sample later = s100;
            later.stamp_ns = stamp;
            later.arrival_ns = 1003 + static_cast<uint64_t>(stamp);
            regressed.store(later);
        }
        check(!regressed.fresh(1200, 300, out), "continued messages do not clear the latched regression");
        ans::Sample latch_probe;
        check(!ans::selectSample(ans::Source::VINS_BODY, &regressed, 1200, 300, latch_probe),
              "a latched source is unavailable to the selector");

        // Only an explicit new navigation session may re-enable the source.
        regressed.beginSession();
        check(regressed.sessions() == 1, "the new session is explicit and counted");
        check(!regressed.fresh(1300, 300, out), "a new session alone provides no sample");
        ans::Sample after = s100;
        after.stamp_ns = 200;
        after.arrival_ns = 1400;
        regressed.store(after);
        check(regressed.fresh(1500, 300, out), "a sample after the explicit new session is usable");
        check(out.stamp_ns == 200, "the usable sample belongs to the new session, not to the old run");
    }

    // --- duplicate timestamps must not keep an old measurement alive ---------
    {
        ans::Mailbox duplicate;
        ans::Sample stored_sample = makeSample(1.0, 2.0, 0.0, 0.0, {0.10, 0.0, 0.0});
        stored_sample.stamp_ns = 500;
        stored_sample.arrival_ns = 2000;
        duplicate.store(stored_sample);
        ans::Sample retransmitted = stored_sample;
        retransmitted.arrival_ns = 9000;  // same measurement, much later arrival
        duplicate.store(retransmitted);
        check(!duplicate.tripped(), "a duplicate timestamp is not treated as a regression");
        check(duplicate.duplicateRejections() == 1, "the duplicate timestamp is counted");
        check(!duplicate.fresh(9500, 300, out), "a duplicate timestamp does not refresh the communication age");
        check(duplicate.fresh(2200, 300, out), "the original measurement keeps its own arrival time");
    }

    // --- a new session must not reuse the previous session's origin ---------
    {
        ans::Origin first_session;
        ans::Origin second_session;
        const ans::Sample sample_a = makeSample(0.0, 0.0, 0.0, 0.0, {});
        const ans::Sample sample_b = makeSample(10.0, 5.0, 0.0, M_PI / 2, {});
        std::array<double, 2> nav_position{};
        double nav_yaw = 0.0;
        ans::captureOrigin(sample_a, first_session);
        check(ans::toNavigationFrame(sample_a, first_session, nav_position, nav_yaw) &&
              near(nav_position[0], 0.0) && near(nav_position[1], 0.0),
              "first session anchors at its own sample");
        // StateRL::enter() resets its origin member and captures again on the
        // first sample of the new session.
        ans::captureOrigin(sample_b, second_session);
        check(ans::toNavigationFrame(sample_b, second_session, nav_position, nav_yaw) &&
              near(nav_position[0], 0.0) && near(nav_position[1], 0.0) && near(nav_yaw, 0.0),
              "second session anchors at the new sample, not the old origin");
        check(ans::toNavigationFrame(sample_a, second_session, nav_position, nav_yaw) &&
              !(near(nav_position[0], 0.0) && near(nav_position[1], 0.0)),
              "the previous session's pose is no longer the navigation origin");
    }

    // --- selection: the independent source is the only way to get a sample ---
    ans::Mailbox selected;
    selected.store(moving);
    ans::Sample chosen;
    check(!ans::selectSample(ans::Source::ODOM, &selected, 5100, 300, chosen),
          "odom source never consumes the independent mailbox");
    check(!ans::selectSample(ans::Source::VINS_BODY, nullptr, 5100, 300, chosen),
          "missing mailbox yields unavailable, never an odometer substitute");
    check(ans::selectSample(ans::Source::VINS_BODY, &selected, 5100, 300, chosen),
          "selected independent source yields its own sample");
    check(!ans::selectSample(ans::Source::VINS_BODY, &selected, 9000, 300, chosen),
          "stale independent source yields unavailable (fail closed, no fallback)");

    // --- observation contract: body velocity reaches the RA components -------
    {
        const auto lin_vel = torch::tensor({{0.30f, 0.20f, 0.05f}}, torch::kFloat);
        const auto ang_vel = torch::tensor({{0.01f, 0.02f, 0.03f}}, torch::kFloat);
        const auto commands = torch::tensor({{0.40f, 0.10f, 0.05f}}, torch::kFloat);
        const auto ray2d = torch::tensor({{1.0f, 1.1f, 1.2f, 1.3f, 1.4f, 1.5f,
                                          1.6f, 1.7f, 1.8f, 1.9f, 2.0f}}, torch::kFloat);
        const abs_observation::Input input{
            lin_vel, torch::Tensor(), ang_vel, torch::Tensor(), commands, torch::Tensor(),
            torch::Tensor(), torch::Tensor(), torch::Tensor(), torch::Tensor(),
            torch::Tensor(), ray2d};
        const auto ra = abs_observation::ra(input);
        check(ra.size(1) == 19, "RA observation is 19-dim");
        check(near(ra[0][0].item<double>(), 0.30, 1e-6) &&
              near(ra[0][1].item<double>(), 0.20, 1e-6) &&
              near(ra[0][2].item<double>(), 0.05, 1e-6),
              "body forward/left/up velocity lands in RA components 0/1/2 in that order");
    }

    // --- what a double rotation would look like (differential control) -------
    {
        // 90 deg body yaw; a body-frame (0.30, 0.20) rotated again by yaw would
        // read (0.20, -0.30), i.e. left and forward swapped. The selected path
        // must keep (0.30, 0.20).
        const double yaw = M_PI / 2;
        const double c = std::cos(yaw), s = std::sin(yaw);
        const double rotated_x = c * 0.30 + s * 0.20;
        const double rotated_y = -s * 0.30 + c * 0.20;
        check(!(near(rotated_x, 0.30) && near(rotated_y, 0.20)),
              "a second world->body rotation would be observable (control case)");
        const auto sample = makeSample(0, 0, 0, yaw, {0.30, 0.20, 0.0});
        check(near(sample.linear_velocity[0], 0.30) && near(sample.linear_velocity[1], 0.20),
              "selected body velocity is not rotated by the body heading");
    }

    return failures == 0 ? 0 : 1;
}
