#include <abs_normal_shutdown_contract.h>

#include <cassert>
#include <cstdint>

int main()
{
    using abs_normal_shutdown::Decision;
    using abs_normal_shutdown::Feedback;

    assert(abs_normal_shutdown::decelerationScale(0.0, 0.6) == 1.0);
    assert(abs_normal_shutdown::decelerationScale(0.3, 0.6) == 0.5);
    assert(abs_normal_shutdown::decelerationScale(0.6, 0.6) == 0.0);
    assert(abs_normal_shutdown::decelerationScale(0.8, 0.6) == 0.0);
    assert(abs_normal_shutdown::decelerationScale(0.1, 0.0) == 0.0);

    Feedback settled{};
    settled.valid = true;
    settled.horizontal_speed_mps = 0.03;
    settled.angular_speed_radps = 0.05;
    settled.roll_rad = 0.04;
    settled.pitch_rad = -0.03;
    settled.base_height_m = 0.34;
    settled.max_joint_position_error_rad = 0.02;
    settled.max_joint_speed_radps = 0.03;
    settled.max_kp_command = 80.0;
    settled.max_kd_command = 3.0;
    settled.supported_feet = 4;

    assert(abs_normal_shutdown::decelerationSettled(settled));
    assert(abs_normal_shutdown::standingConfirmed(settled));
    assert(abs_normal_shutdown::standingSupportEnabled(settled));
    assert(!abs_normal_shutdown::downConfirmed(settled));
    Feedback unsupported = settled;
    unsupported.max_kp_command = 0.0;
    assert(!abs_normal_shutdown::standingSupportEnabled(unsupported));

    assert(abs_normal_shutdown::evaluate(false, false, true, true,
        400000000ULL, 900000000ULL, 400000000ULL, 5000000000ULL) == Decision::CONFIRM);
    assert(abs_normal_shutdown::evaluate(false, false, true, false,
        0, 5000000000ULL, 400000000ULL, 5000000000ULL) == Decision::FAIL);
    assert(abs_normal_shutdown::evaluate(true, false, true, true,
        900000000ULL, 900000000ULL, 400000000ULL, 5000000000ULL) == Decision::ABORT);
    assert(abs_normal_shutdown::evaluate(false, true, true, true,
        900000000ULL, 900000000ULL, 400000000ULL, 5000000000ULL) == Decision::FAIL);
    assert(abs_normal_shutdown::evaluate(false, false, false, true,
        900000000ULL, 900000000ULL, 400000000ULL, 5000000000ULL) == Decision::FAIL);

    Feedback down = settled;
    down.base_height_m = 0.23;
    assert(abs_normal_shutdown::downConfirmed(down));
    down.supported_feet = 1;
    assert(!abs_normal_shutdown::downConfirmed(down));
    down.supported_feet = 2;
    down.max_joint_speed_radps = 0.11;
    assert(!abs_normal_shutdown::downConfirmed(down));
    down.max_joint_speed_radps = 0.03;
    down.roll_rad = 0.4;
    assert(!abs_normal_shutdown::downConfirmed(down));

    return 0;
}
