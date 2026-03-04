#by Cyrus Chung


import math
import random
from dataclasses import dataclass

import sys
import select
import termios
import tty
from enum import Enum
import numpy as np
import rclpy
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from tf2_ros import Buffer
from tf2_ros import TransformBroadcaster
from tf2_ros import TransformException
from tf2_ros import TransformListener

class DriveMode(Enum):
    MANUAL = "MANUAL"
    AUTO = "AUTO"

def _normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _yaw_to_quaternion(yaw: float) -> Quaternion: 
    q = Quaternion() 
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


def _quaternion_to_yaw(q: Quaternion) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _pose_to_transform_matrix(x: float, y: float, yaw: float) -> np.ndarray: #Converts a 2D pose (x, y, yaw) into a 4x4 homogenous transformation matrix.
    c = math.cos(yaw)
    s = math.sin(yaw) #yaw is the angle to rotate, and (x,y) is the translation.
    transform = np.array( #homogenous transformation here
        [
            [c, -s, 0.0, x], #X
            [s, c, 0.0, y], #Y
            [0.0, 0.0, 1.0, 0.0], #Z
            [0.0, 0.0, 0.0, 1.0], #homogenous coordinate
        ],
        dtype=float,
    )
    return transform


@dataclass
class PIDTerms:
    output: float
    p: float
    i: float
    d: float


class PIDController:
    def __init__(
        self,
        kp: float,
        ki: float,
        kd: float,
        output_min: float,
        output_max: float,
    ) -> None:
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_min = output_min
        self.output_max = output_max
        self.integral = 0.0
        self.prev_error = 0.0
        self.initialized = False

    def reset(self) -> None:
        self.integral = 0.0
        self.prev_error = 0.0
        self.initialized = False

    def update(self, error: float, dt: float) -> PIDTerms:
        dt = max(dt, 1e-4)
        if not self.initialized:
            self.prev_error = error
            self.initialized = True

        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        self.prev_error = error

        p_term = self.kp * error
        i_term = self.ki * self.integral                                                                                                                                                                                                                                                                                                #ni
        d_term = self.kd * derivative

        output = p_term + i_term + d_term
        output = max(self.output_min, min(self.output_max, output))
        return PIDTerms(output=output, p=p_term, i=i_term, d=d_term)


class PidAutoDrive(Node):
    def __init__(self) -> None:
        super().__init__("pid_auto_drive")

        self.declare_parameter("start_mode", "MANUAL")
        self.declare_parameter("enable_keyboard", True)

        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("world_frame", "map")
        self.declare_parameter("base_frame", "ego_racecar/base_link")
        self.declare_parameter("target_frame", "pid_target")
        self.declare_parameter("control_rate_hz", 20.0)
        self.declare_parameter("goal_tolerance_m", 0.40)
        self.declare_parameter("max_linear_speed", 2.0)
        self.declare_parameter("max_angular_speed", 0.35)
        self.declare_parameter("dist_kp", 1.2)
        self.declare_parameter("dist_ki", 0.03)
        self.declare_parameter("dist_kd", 0.08)
        self.declare_parameter("yaw_kp", 2.4)
        self.declare_parameter("yaw_ki", 0.02)
        self.declare_parameter("yaw_kd", 0.12)
        self.declare_parameter("log_every_n", 5)

        start_mode_str = str(self.get_parameter("start_mode").value).upper()
        self.mode = DriveMode.AUTO if start_mode_str == "AUTO" else DriveMode.MANUAL
        self.enable_keyboard = bool(self.get_parameter("enable_keyboard").value)
        
        self.get_logger().info(f"Starting in {self.mode.value} mode (keyboard: {self.enable_keyboard})")

        map_topic = str(self.get_parameter("map_topic").value)
        cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        self.world_frame = str(self.get_parameter("world_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.target_frame = str(self.get_parameter("target_frame").value)
        control_rate_hz = float(self.get_parameter("control_rate_hz").value)
        self.goal_tolerance_m = float(self.get_parameter("goal_tolerance_m").value)
        max_linear_speed = float(self.get_parameter("max_linear_speed").value)
        max_angular_speed = float(self.get_parameter("max_angular_speed").value)
        self.log_every_n = max(1, int(self.get_parameter("log_every_n").value))

        self.distance_pid = PIDController(
            kp=float(self.get_parameter("dist_kp").value),
            ki=float(self.get_parameter("dist_ki").value),
            kd=float(self.get_parameter("dist_kd").value),
            output_min=0.0,
            output_max=max_linear_speed,
        )
        self.yaw_pid = PIDController(
            kp=float(self.get_parameter("yaw_kp").value),
            ki=float(self.get_parameter("yaw_ki").value),
            kd=float(self.get_parameter("yaw_kd").value),
            output_min=-max_angular_speed,
            output_max=max_angular_speed,
        )

        self.map_msg: OccupancyGrid | None = None
        self.target_x: float | None = None
        self.target_y: float | None = None
        self.target_yaw: float = 0.0
        self.control_step = 0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self) #tf tree here, will publish target as TF frame for visualization

        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.map_sub = self.create_subscription(OccupancyGrid, map_topic, self._map_callback, 10)

        timer_period = 1.0 / control_rate_hz if control_rate_hz > 0.0 else 0.05
        self.timer = self.create_timer(timer_period, self._control_callback)
        self.last_t = self.get_clock().now()

        self.get_logger().info(
            f"PID auto-drive enabled. Following random in-map targets using frame {self.base_frame}."
        )

        # Only set up keyboard if enabled AND stdin is a TTY
        if self.enable_keyboard and sys.stdin.isatty():
            try:
                self.old_settings = termios.tcgetattr(sys.stdin)
                tty.setcbreak(sys.stdin.fileno())
                self.key_timer = self.create_timer(0.05, self._check_key_callback)
            except termios.error:
                self.get_logger().warn("Cannot enable keyboard (no TTY). Mode switching disabled.")
                self.enable_keyboard = False
        else:
            self.enable_keyboard = False

    def _check_key_callback(self) -> None:
        """Non-blocking keyboard check for mode switching."""
        if not self.enable_keyboard:
            return
        if select.select([sys.stdin], [], [], 0)[0]:
            key = sys.stdin.read(1).lower()
            
            if key == 'm' and self.mode != DriveMode.MANUAL:
                self.mode = DriveMode.MANUAL
                self.get_logger().info(" Switched to MANUAL mode ")
                # Stop auto-drive
                cmd = Twist()
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                self.cmd_pub.publish(cmd)
                
            elif key == 'n' and self.mode != DriveMode.AUTO:
                self.mode = DriveMode.AUTO
                self.get_logger().info(" Switched to AUTO mode ")
                self._sample_new_target()

    def _map_callback(self, msg: OccupancyGrid) -> None:
        self.map_msg = msg
        if self.target_x is None or self.target_y is None:
            self._sample_new_target()

    def _sample_new_target(self) -> bool:
        if self.map_msg is None:
            return False

        width = self.map_msg.info.width
        height = self.map_msg.info.height
        if width == 0 or height == 0:
            return False

        map_array = np.array(self.map_msg.data, dtype=np.int16).reshape(height, width)
        free_rows, free_cols = np.where(map_array == 0)
        if free_rows.size == 0:
            self.get_logger().warning("No free map cells found for random target sampling.")
            return False

        idx = random.randrange(free_rows.size)
        row = int(free_rows[idx])
        col = int(free_cols[idx])

        resolution = self.map_msg.info.resolution
        origin = self.map_msg.info.origin
        self.target_x = origin.position.x + (col + 0.5) * resolution
        self.target_y = origin.position.y + (row + 0.5) * resolution
        self.target_yaw = random.uniform(-math.pi, math.pi)

        self.distance_pid.reset()
        self.yaw_pid.reset()

        self.get_logger().info(
            f"New target in map free-space: x={self.target_x:.2f}, y={self.target_y:.2f}, yaw={self.target_yaw:.2f}"
        )
        return True

    def _publish_target_tf(self) -> None:
        if self.target_x is None or self.target_y is None:
            return

        target_tf = TransformStamped()
        target_tf.header.stamp = self.get_clock().now().to_msg()
        target_tf.header.frame_id = self.world_frame
        target_tf.child_frame_id = self.target_frame
        target_tf.transform.translation.x = self.target_x
        target_tf.transform.translation.y = self.target_y
        target_tf.transform.translation.z = 0.0
        target_tf.transform.rotation = _yaw_to_quaternion(self.target_yaw)
        self.tf_broadcaster.sendTransform(target_tf)

    def _control_callback(self) -> None:
        if self.mode != DriveMode.AUTO:
            return  # Skip control loop in manual mode
            
        if self.target_x is None or self.target_y is None:
            if not self._sample_new_target():
                return

        self._publish_target_tf()

        try:
            tf_wr1 = self.tf_buffer.lookup_transform(
                self.world_frame,
                self.base_frame,
                rclpy.time.Time(),
            )
        except TransformException as ex:
            self.get_logger().debug(f"Waiting for transform {self.world_frame} -> {self.base_frame}: {ex}")
            return

        now = self.get_clock().now()
        dt = (now - self.last_t).nanoseconds * 1e-9
        self.last_t = now

        robot_x = float(tf_wr1.transform.translation.x)
        robot_y = float(tf_wr1.transform.translation.y)
        robot_yaw = _quaternion_to_yaw(tf_wr1.transform.rotation)

        t_wr1 = _pose_to_transform_matrix(robot_x, robot_y, robot_yaw)
        t_wr2 = _pose_to_transform_matrix(self.target_x, self.target_y, self.target_yaw)

        t_r1r2 = np.linalg.inv(t_wr1) @ t_wr2
        xyz_r1r2 = t_r1r2[0:3, 3]

        r_wr1 = t_wr1[0:3, 0:3]
        r_r1r2 = np.linalg.inv(t_r1r2)[0:3, 0:3]
        r_wr2 = r_wr1 @ np.linalg.inv(r_r1r2)

        _target_yaw_from_rotation = math.atan2(r_wr2[1, 0], r_wr2[0, 0])
        _ = _target_yaw_from_rotation

        distance_error = float(math.hypot(xyz_r1r2[0], xyz_r1r2[1]))
        yaw_error = _normalize_angle(math.atan2(xyz_r1r2[1], xyz_r1r2[0]))

        dist_terms = self.distance_pid.update(distance_error, dt)
        yaw_terms = self.yaw_pid.update(yaw_error, dt)

        cmd = Twist()
        cmd.linear.x = float(dist_terms.output)
        cmd.angular.z = float(yaw_terms.output)

        if distance_error < self.goal_tolerance_m:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.cmd_pub.publish(cmd)
            self._sample_new_target()
            return

        self.cmd_pub.publish(cmd)

        self.control_step += 1
        if self.control_step % self.log_every_n == 0:
            self.get_logger().info(
                "PID log | "
                f"e_dist(t)={distance_error:.3f}, dist(P,I,D)=({dist_terms.p:.3f},{dist_terms.i:.3f},{dist_terms.d:.3f}), v={dist_terms.output:.3f} | "
                f"e_yaw(t)={yaw_error:.3f}, yaw(P,I,D)=({yaw_terms.p:.3f},{yaw_terms.i:.3f},{yaw_terms.d:.3f}), w={yaw_terms.output:.3f}"
            )
    
    def destroy_node(self) -> None:
        """Restore terminal on shutdown."""
        if self.enable_keyboard and hasattr(self, 'old_settings'):
            try:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
            except:
                pass
        super().destroy_node()

def main() -> None:
    rclpy.init()
    node = PidAutoDrive()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop = Twist()
        node.cmd_pub.publish(stop)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
