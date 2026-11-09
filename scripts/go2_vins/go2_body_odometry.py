#!/usr/bin/env python3
"""Read-only Go2 body odometry adapter. No robot command or TF publisher."""

import argparse
import sys

from body_frame import SamplePairing, convert, load_mount


def stamp_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def xyz(value):
    return [value.x, value.y, value.z]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mount", required=True, help="measured T_base_imu JSON")
    parser.add_argument("--input", default="/vins/imu_propagate")
    parser.add_argument("--imu", default="/camera/camera/imu")
    parser.add_argument("--output", default="/go2/vins/base_odometry")
    args = parser.parse_args()
    # Refuse unknown calibration BEFORE creating a ROS context or touching DDS.
    try:
        config, r_bi, t_bi = load_mount(args.mount)
    except (OSError, ValueError, TypeError, KeyError) as error:
        print(f"安装配置错误：{error}", file=sys.stderr)
        return 2
    if len({args.input, args.imu, args.output}) != 3:
        print("input/imu/output topics must differ", file=sys.stderr)
        return 2

    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Imu

    class BodyOdometry(Node):
        def __init__(self):
            super().__init__("go2_body_odometry")
            self.pairing = SamplePairing()
            self.published = 0
            self.pub = self.create_publisher(Odometry, args.output, 10)
            self.create_subscription(Odometry, args.input, self.on_odometry, 40)
            self.create_subscription(Imu, args.imu, self.on_imu, qos_profile_sensor_data)
            self.create_timer(5.0, self.report)
            self.get_logger().info(
                f"只发布状态：{config['world_frame']} -> {config['base_frame']}；"
                "角速度采用同步原始相机gyro（仍含零偏）；不发送机器人命令、不发布TF"
            )

        def on_odometry(self, msg):
            if (msg.header.frame_id != config["world_frame"]
                    or msg.child_frame_id != config["imu_frame"]):
                raise ValueError("VINS frame 与安装配置不符，拒绝转换")
            self.publish_pairs(self.pairing.add_odometry(stamp_ns(msg.header.stamp), msg))

        def on_imu(self, msg):
            if msg.header.frame_id != config["imu_frame"]:
                raise ValueError("IMU frame 与 VINS body 不同，不能直接使用此gyro")
            if msg.angular_velocity_covariance[0] == -1.0:
                raise ValueError("IMU 消息声明角速度不可用")
            self.publish_pairs(self.pairing.add_imu(stamp_ns(msg.header.stamp), msg))

        def publish_pairs(self, pairs):
            for odom, imu in pairs:
                pose = odom.pose.pose
                q = pose.orientation
                p, q_out, v, w = convert(
                    xyz(pose.position), [q.x, q.y, q.z, q.w],
                    xyz(odom.twist.twist.linear), xyz(imu.angular_velocity), r_bi, t_bi
                )
                out = Odometry()
                out.header = odom.header  # Keep VINS measurement stamp and world frame.
                out.child_frame_id = config["base_frame"]
                out.pose.pose.position.x, out.pose.pose.position.y, out.pose.pose.position.z = map(float, p)
                (out.pose.pose.orientation.x, out.pose.pose.orientation.y,
                 out.pose.pose.orientation.z, out.pose.pose.orientation.w) = map(float, q_out)
                (out.twist.twist.linear.x, out.twist.twist.linear.y,
                 out.twist.twist.linear.z) = map(float, v)
                (out.twist.twist.angular.x, out.twist.twist.angular.y,
                 out.twist.twist.angular.z) = map(float, w)
                # VINS has no covariance estimate here. Leave zero/unspecified;
                # do not copy a covariance belonging to another reference point.
                self.pub.publish(out)
                self.published += 1

        def report(self):
            self.get_logger().info(
                f"published={self.published} pending={len(self.pairing.pending)} "
                f"unmatched_dropped={self.pairing.dropped}；仅为转换观察，未验证定位健康"
            )

    rclpy.init(args=[])
    node = None
    result = 0
    try:
        node = BodyOdometry()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as error:
        print(f"转换停止：{error}", file=sys.stderr)
        result = 1
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return result


if __name__ == "__main__":
    sys.exit(main())
