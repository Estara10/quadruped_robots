#include "rl_quadruped_controller/control/CtrlComponent.h"
#include "rl_quadruped_controller/control/Estimator.h"
#include "rl_quadruped_controller/robot/QuadrupedRobot.h"
#include <controller_common/CtrlInterfaces.h>
#include <controller_common/common/mathTools.h>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/loaned_state_interface.hpp>

#include <array>
#include <algorithm>
#include <cmath>
#include <deque>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
constexpr int kRateHz = 200;
const std::array<std::string, 12> kJoints{
    "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
    "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
    "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
    "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint"};
const std::array<std::string, 4> kFeet{"FR", "FL", "RR", "RL"};
const std::array<std::string, 10> kImu{
    "orientation.w", "orientation.x", "orientation.y", "orientation.z",
    "angular_velocity.x", "angular_velocity.y", "angular_velocity.z",
    "linear_acceleration.x", "linear_acceleration.y", "linear_acceleration.z"};

std::vector<std::string> split(const std::string& line) {
    std::vector<std::string> result;
    std::stringstream stream(line);
    std::string field;
    while (std::getline(stream, field, ',')) result.push_back(field);
    if (!line.empty() && line.back() == ',') result.emplace_back();
    return result;
}

std::string slurp(const std::string& path) {
    std::ifstream file(path);
    if (!file) throw std::runtime_error("cannot open " + path);
    std::ostringstream content;
    content << file.rdbuf();
    return content.str();
}

struct Storage {
    std::array<double, 12> q{};
    std::array<double, 12> dq{};
    std::array<double, 10> imu{};
    std::array<double, 4> force{};
    std::deque<hardware_interface::StateInterface> state_handles;
    std::deque<hardware_interface::LoanedStateInterface> loaned_handles;
    CtrlInterfaces interfaces;

    void add(const std::string& prefix, const std::string& name, double* value,
             std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>& target) {
        state_handles.emplace_back(prefix, name, value);
        loaned_handles.emplace_back(state_handles.back());
        target.emplace_back(loaned_handles.back());
    }

    Storage() {
        interfaces.frequency_ = kRateHz;
        for (size_t i = 0; i < kJoints.size(); ++i) {
            add(kJoints[i], "position", &q[i], interfaces.joint_position_state_interface_);
            add(kJoints[i], "velocity", &dq[i], interfaces.joint_velocity_state_interface_);
        }
        for (size_t i = 0; i < kImu.size(); ++i)
            add("imu_sensor", kImu[i], &imu[i], interfaces.imu_state_interface_);
        for (size_t i = 0; i < kFeet.size(); ++i)
            add("foot_force/" + kFeet[i], "force", &force[i], interfaces.foot_force_state_interface_);
    }
};

double value(const std::vector<std::string>& fields, size_t index) {
    if (index >= fields.size() || fields[index].empty()) throw std::runtime_error("required input column absent");
    const double result = std::stod(fields[index]);
    if (!std::isfinite(result)) throw std::runtime_error("non-finite replay input");
    return result;
}

void csvVec(std::ostream& output, const Vec3& vector) {
    output << vector.x() << ',' << vector.y() << ',' << vector.z();
}

int runLegConstraintReplay(const std::string& input_path, const std::string& urdf_path,
                           const std::string& output_path) {
    std::ifstream input(input_path);
    if (!input) throw std::runtime_error("cannot open " + input_path);
    std::string line;
    if (!std::getline(input, line)) throw std::runtime_error("empty input CSV");
    const auto header = split(line);
    std::map<std::string, size_t> column;
    for (size_t i = 0; i < header.size(); ++i) column.emplace(header[i], i);
    auto col = [&](const std::string& name) -> size_t {
        const auto it = column.find(name);
        if (it == column.end()) throw std::runtime_error("missing CSV header: " + name);
        return it->second;
    };
    const size_t time_col = col("update_time_s");
    std::array<size_t, 12> q_col{}, dq_col{};
    for (size_t i = 0; i < kJoints.size(); ++i) {
        q_col[i] = col(kJoints[i] + "_q_rad");
        dq_col[i] = col(kJoints[i] + "_dq_rad_s");
    }
    std::array<size_t, 10> imu_col{};
    for (size_t i = 0; i < kImu.size(); ++i) imu_col[i] = col(kImu[i]);
    std::array<size_t, 4> force_col{};
    for (size_t i = 0; i < kFeet.size(); ++i) force_col[i] = col(kFeet[i] + "_foot_force_raw");
    const size_t last_input_col = std::max({time_col, *std::max_element(q_col.begin(), q_col.end()),
        *std::max_element(dq_col.begin(), dq_col.end()), *std::max_element(imu_col.begin(), imu_col.end()),
        *std::max_element(force_col.begin(), force_col.end())});

    Storage storage;
    const std::string description = slurp(urdf_path);
    QuadrupedRobot robot(storage.interfaces, description,
        std::vector<std::string>{"FR_foot", "FL_foot", "RR_foot", "RL_foot"}, "base");
    if (!(robot.mass_ > 0.0)) throw std::runtime_error("URDF KDL model has zero mass");
    std::ofstream output(output_path);
    if (!output) throw std::runtime_error("cannot open output " + output_path);
    output << "update_time_s,leg,force_raw,contact_force_ge_1,hip_q_rad,hip_dq_rad_s,"
              "thigh_q_rad,thigh_dq_rad_s,calf_q_rad,calf_dq_rad_s,"
              "p_body_x_m,p_body_y_m,p_body_z_m,jqd_body_x_m_s,jqd_body_y_m_s,jqd_body_z_m_s,"
              "gyro_input_x,gyro_input_y,gyro_input_z,"
              "base_v_body_noomega_x,base_v_body_noomega_y,base_v_body_noomega_z,"
              "base_v_world_noomega_x,base_v_world_noomega_y,base_v_world_noomega_z,"
              "base_v_body_omega_x,base_v_body_omega_y,base_v_body_omega_z,"
              "base_v_world_omega_x,base_v_world_omega_y,base_v_world_omega_z\n";

    size_t rows = 0, short_rows = 0;
    while (std::getline(input, line)) {
        auto fields = split(line);
        while (!fields.empty() && fields.back().empty()) fields.pop_back();
        if (fields.empty()) continue;
        if (fields.size() <= last_input_col) throw std::runtime_error("short row truncates an Estimator input field");
        if (fields.size() != header.size()) ++short_rows;
        const double time_s = value(fields, time_col);
        for (size_t i = 0; i < 12; ++i) {
            storage.q[i] = value(fields, q_col[i]);
            storage.dq[i] = value(fields, dq_col[i]);
        }
        for (size_t i = 0; i < 10; ++i) storage.imu[i] = value(fields, imu_col[i]);
        for (size_t i = 0; i < 4; ++i) storage.force[i] = value(fields, force_col[i]);
        robot.update();

        Quat quaternion;
        quaternion << storage.imu[0], storage.imu[1], storage.imu[2], storage.imu[3];
        const RotMat body_to_world = quatToRotMat(quaternion);
        Vec3 gyro_input;
        gyro_input << storage.imu[4], storage.imu[5], storage.imu[6];
        const auto foot_positions = robot.getFeet2BPositions();
        const auto foot_velocities = robot.getFeet2BVelocities();
        for (size_t leg = 0; leg < 4; ++leg) {
            const Vec3 p_body(foot_positions[leg].p.data);
            const Vec3 j_qd_body(foot_velocities[leg].data);
            const auto no_gyro_measurement = Estimator::footMeasurementToWorld(
                p_body, j_qd_body, gyro_input, body_to_world, false);
            const auto gyro_measurement = Estimator::footMeasurementToWorld(
                p_body, j_qd_body, gyro_input, body_to_world, true);
            const Vec3 base_body_noomega = -j_qd_body;
            const Vec3 base_body_omega = -(gyro_input.cross(p_body) + j_qd_body);
            const Vec3 base_world_noomega = -no_gyro_measurement.velocity;
            const Vec3 base_world_omega = -gyro_measurement.velocity;
            output << std::setprecision(12) << time_s << ',' << kFeet[leg] << ',' << storage.force[leg]
                   << ',' << (storage.force[leg] >= 1.0 ? 1 : 0);
            for (size_t joint = 0; joint < 3; ++joint)
                output << ',' << storage.q[3 * leg + joint] << ',' << storage.dq[3 * leg + joint];
            output << ','; csvVec(output, p_body);
            output << ','; csvVec(output, j_qd_body);
            output << ','; csvVec(output, gyro_input);
            output << ','; csvVec(output, base_body_noomega);
            output << ','; csvVec(output, base_world_noomega);
            output << ','; csvVec(output, base_body_omega);
            output << ','; csvVec(output, base_world_omega);
            output << '\n';
        }
        ++rows;
    }
    output.flush();
    std::cerr << "LEG_CONSTRAINT_REPLAY_OK rows=" << rows << " leg_rows=" << rows * 4
              << " short_rows_inputs_recovered=" << short_rows << "\n";
    return 0;
}
}

int main(int argc, char** argv) try {
    if (argc == 2 && std::string(argv[1]) == "--self-check") {
        const double half = std::sqrt(0.5);
        Quat q;
        q << half, 0.0, 0.0, half;
        const RotMat r = quatToRotMat(q);
        Vec3 p, j_qd, gyro;
        p << 1.0, 0.0, 0.0;
        j_qd.setZero();
        gyro << 0.0, 0.0, 2.0;
        const auto b = Estimator::footMeasurementToWorld(p, j_qd, gyro, r, false);
        const auto c = Estimator::footMeasurementToWorld(p, j_qd, gyro, r, true);
        if ((b.position - Vec3(0.0, 1.0, 0.0)).norm() > 1e-12 || b.velocity.norm() > 1e-12 ||
            (c.velocity - Vec3(-2.0, 0.0, 0.0)).norm() > 1e-12)
            throw std::runtime_error("world-frame/gyro cross-product sign check failed");
        std::cout << "SELF_CHECK_OK yaw90_position=world_y gyro_plus_z_velocity=world_minus_x\n";
        return 0;
    }
    if (argc == 8 && std::string(argv[1]) == "--leg-constraints" &&
        std::string(argv[2]) == "--input" && std::string(argv[4]) == "--urdf" &&
        std::string(argv[6]) == "--output")
        return runLegConstraintReplay(argv[3], argv[5], argv[7]);
    if (argc != 7 || std::string(argv[1]) != "--input" || std::string(argv[3]) != "--urdf" ||
        std::string(argv[5]) != "--output") {
        std::cerr << "Usage: go2_estimator_replay --input estimator.csv --urdf go2.urdf --output replay.csv\n"
                     "       go2_estimator_replay --leg-constraints --input estimator.csv --urdf go2.urdf --output constraints.csv\n"
                     "       go2_estimator_replay --self-check\n";
        return 2;
    }
    const std::string input_path = argv[2], urdf_path = argv[4], output_path = argv[6];
    std::ifstream input(input_path);
    if (!input) throw std::runtime_error("cannot open " + input_path);
    std::string line;
    if (!std::getline(input, line)) throw std::runtime_error("empty input CSV");
    const auto header = split(line);
    std::map<std::string, size_t> column;
    for (size_t i = 0; i < header.size(); ++i) column.emplace(header[i], i);
    auto col = [&](const std::string& name) -> size_t {
        const auto it = column.find(name);
        if (it == column.end()) throw std::runtime_error("missing CSV header: " + name);
        return it->second;
    };
    const size_t time_col = col("update_time_s");
    std::array<size_t, 12> q_col{}, dq_col{};
    for (size_t i = 0; i < kJoints.size(); ++i) {
        q_col[i] = col(kJoints[i] + "_q_rad");
        dq_col[i] = col(kJoints[i] + "_dq_rad_s");
    }
    std::array<size_t, 10> imu_col{};
    for (size_t i = 0; i < kImu.size(); ++i) imu_col[i] = col(kImu[i]);
    std::array<size_t, 4> force_col{};
    for (size_t i = 0; i < kFeet.size(); ++i) force_col[i] = col(kFeet[i] + "_foot_force_raw");
    const size_t last_input_col = std::max({time_col, *std::max_element(q_col.begin(), q_col.end()),
        *std::max_element(dq_col.begin(), dq_col.end()), *std::max_element(imu_col.begin(), imu_col.end()),
        *std::max_element(force_col.begin(), force_col.end())});

    Storage storage;
    CtrlComponent component;
    std::string description = slurp(urdf_path);
    component.robot_model_ = std::make_shared<QuadrupedRobot>(storage.interfaces, description,
        std::vector<std::string>{"FR_foot", "FL_foot", "RR_foot", "RL_foot"}, "base");
    if (!(component.robot_model_->mass_ > 0.0)) throw std::runtime_error("URDF KDL model has zero mass");
    Estimator original(storage.interfaces, component);
    Estimator rotated(storage.interfaces, component, 1, Estimator::FootMeasurementMode::RotateToWorld);
    Estimator rotated_gyro(storage.interfaces, component, 1, Estimator::FootMeasurementMode::RotateToWorldWithGyro);
    std::ofstream output(output_path);
    if (!output) throw std::runtime_error("cannot open output " + output_path);
    output << "update_time_s,original_row_complete,csv_world_vx,csv_world_vy,csv_world_vz,"
              "A_world_vx,A_world_vy,A_world_vz,A_body_vx,A_body_vy,A_body_vz,"
              "B_world_vx,B_world_vy,B_world_vz,B_body_vx,B_body_vy,B_body_vz,"
              "C_world_vx,C_world_vy,C_world_vz,C_body_vx,C_body_vy,C_body_vz,yaw_rad";
    for (const auto& leg : kFeet)
        output << ',' << leg << "_fixed_foot_residual_world_x_m_s," << leg
               << "_fixed_foot_residual_world_y_m_s," << leg << "_fixed_foot_residual_world_z_m_s";
    output << '\n';

    size_t rows = 0, short_rows = 0, compared_rows = 0;
    double max_original_replay_difference = 0.0;
    while (std::getline(input, line)) {
        auto fields = split(line);
        while (!fields.empty() && fields.back().empty()) fields.pop_back();
        if (fields.empty()) continue;
        if (fields.size() <= last_input_col) throw std::runtime_error("short row truncates an Estimator input field");
        if (fields.size() != header.size()) ++short_rows;
        const double time_s = value(fields, time_col);
        for (size_t i = 0; i < 12; ++i) {
            storage.q[i] = value(fields, q_col[i]);
            storage.dq[i] = value(fields, dq_col[i]);
        }
        for (size_t i = 0; i < 10; ++i) storage.imu[i] = value(fields, imu_col[i]);
        for (size_t i = 0; i < 4; ++i) storage.force[i] = value(fields, force_col[i]);
        component.robot_model_->update();
        original.update(); rotated.update(); rotated_gyro.update();
        const Vec3 a_world = original.getVelocity(), b_world = rotated.getVelocity(), c_world = rotated_gyro.getVelocity();
        const RotMat rotation = original.getRotation();
        const Vec3 a_body = rotation.transpose() * a_world;
        const Vec3 b_body = rotation.transpose() * b_world;
        const Vec3 c_body = rotation.transpose() * c_world;
        Vec3 gyro_body;
        gyro_body << storage.imu[4], storage.imu[5], storage.imu[6];
        const auto foot_positions = component.robot_model_->getFeet2BPositions();
        const auto foot_velocities = component.robot_model_->getFeet2BVelocities();
        std::array<Vec3, 4> fixed_foot_residual;
        for (size_t leg = 0; leg < 4; ++leg) {
            const Vec3 p_body(foot_positions[leg].p.data);
            const Vec3 j_qd_body(foot_velocities[leg].data);
            const auto foot_velocity = Estimator::footMeasurementToWorld(
                p_body, j_qd_body, gyro_body, rotation, true);
            fixed_foot_residual[leg] = c_world + foot_velocity.velocity;
        }
        bool source_complete = fields.size() == header.size();
        Vec3 csv_world;
        if (source_complete) {
            csv_world << value(fields, col("estimator_world_vx_m_s")),
                         value(fields, col("estimator_world_vy_m_s")),
                         value(fields, col("estimator_world_vz_m_s"));
            max_original_replay_difference = std::max(max_original_replay_difference, (csv_world - a_world).cwiseAbs().maxCoeff());
            ++compared_rows;
        }
        output << std::setprecision(12) << time_s << ',' << (source_complete ? 1 : 0) << ',';
        if (source_complete) output << csv_world.x() << ',' << csv_world.y() << ',' << csv_world.z();
        else output << ",,";
        output << ','; csvVec(output, a_world); output << ','; csvVec(output, a_body);
        output << ','; csvVec(output, b_world); output << ','; csvVec(output, b_body);
        output << ','; csvVec(output, c_world); output << ','; csvVec(output, c_body);
        output << ',' << std::atan2(rotation(1, 0), rotation(0, 0));
        for (const auto& residual : fixed_foot_residual) { output << ','; csvVec(output, residual); }
        output << '\n';
        ++rows;
    }
    output.flush();
    std::cerr << "REPLAY_OK rows=" << rows << " short_rows_inputs_recovered=" << short_rows
              << " complete_rows_compared=" << compared_rows << " max_abs_original_world_v_difference_m_s="
              << max_original_replay_difference << " dt_s=" << (1.0 / kRateHz) << '\n';
    return 0;
} catch (const std::exception& error) {
    std::cerr << "REPLAY_ERROR: " << error.what() << '\n';
    return 1;
}
