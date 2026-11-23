#!/usr/bin/env python3
"""
Both old nodes read the same information (where the drone is and which way it
points), so one subscription to /fmu/out/vehicle_odometry is enough:
  - publishes /odom (nav_msgs/Odometry), TF odom -> base_link, /drone/path
  - prints a one-line status (NED position, altitude, yaw, speed) in the terminal
"""
import math
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from px4_msgs.msg import VehicleOdometry
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster

# ---------- Quaternion math (order: w, x, y, z) ----------
S = math.sqrt(0.5)
Q_NED_ENU = (0.0, S, S, 0.0)      # 180 deg about (1,1,0): world frame NED -> ENU
Q_FRD_FLU = (0.0, 1.0, 0.0, 0.0)  # 180 deg about x: body frame FRD -> FLU


def qmul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (aw*bw - ax*bx - ay*by - az*bz,
            aw*bx + ax*bw + ay*bz - az*by,
            aw*by - ax*bz + ay*bw + az*bx,
            aw*bz + ax*by - ay*bx + az*bw)


def qconj(q):
    return (q[0], -q[1], -q[2], -q[3])


def qnorm(q):
    n = math.sqrt(sum(c * c for c in q))
    return tuple(c / n for c in q)


def rotate(q, v):
    """Rotate vector v by quaternion q."""
    r = qmul(qmul(q, (0.0, v[0], v[1], v[2])), qconj(q))
    return (r[1], r[2], r[3])


def yaw_ned(q):
    """Heading (rad) from a FRD->NED quaternion: 0 = North, clockwise positive.
    Same value PX4 publishes as `heading` in vehicle_local_position."""
    w, x, y, z = q
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def has_nan(seq):
    return any(math.isnan(x) for x in seq)


class Px4ToOdom(Node):
    def __init__(self):
        super().__init__('px4_to_odom')

        self.declare_parameter('odom_topic', '/fmu/out/vehicle_odometry')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('path_min_dist', 0.05)    # m, skip points closer than this
        self.declare_parameter('path_max_poses', 5000)   # cap the trajectory length
        self.declare_parameter('print_status', True)     # one-line live status
        self.declare_parameter('print_period', 0.25)     # s

        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.publish_tf = self.get_parameter('publish_tf').value
        self.min_dist = self.get_parameter('path_min_dist').value
        self.max_poses = self.get_parameter('path_max_poses').value
        self.print_on = self.get_parameter('print_status').value
        topic = self.get_parameter('odom_topic').value

        # PX4 publishes BEST_EFFORT: the subscriber must match or it receives nothing
        px4_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                             durability=DurabilityPolicy.VOLATILE,
                             history=HistoryPolicy.KEEP_LAST, depth=1)
        
        self.create_subscription(VehicleOdometry, topic, self.on_odom, px4_qos)

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.path_pub = self.create_publisher(Path, '/drone/path', 10)
        self.tf_br = TransformBroadcaster(self)

        self.path = Path()
        self.path.header.frame_id = self.odom_frame
        self.last_xy = None
        self.warned_vframe = False

        # Latest values, shared between the subscription callback and the print timer.
        # Both run in the same (single-threaded) executor, so no lock is needed.
        self.have_data = False
        self.ned = (0.0, 0.0, 0.0)
        self.yaw_deg = 0.0
        self.speed = 0.0

        if self.print_on:
            self.create_timer(self.get_parameter('print_period').value, self.print_status)
        self.get_logger().info(f'Listening to {topic}')

    # ------------------------------------------------------------------
    def on_odom(self, m: VehicleOdometry):
        # PX4 message fields are numpy.float32; rclpy only accepts Python float
        pos = [float(x) for x in m.position]
        quat = tuple(float(x) for x in m.q)
        lin = [float(x) for x in m.velocity]
        ang = [float(x) for x in m.angular_velocity]

        if has_nan(pos) or has_nan(quat):
            return

        # ---- keep the raw NED values for the status line ----
        n, e, d = pos
        self.ned = (n, e, d)
        self.yaw_deg = math.degrees(yaw_ned(quat))
        self.speed = 0.0 if has_nan(lin) else math.sqrt(sum(v * v for v in lin))
        self.have_data = True

        # ---- position NED -> ENU ----
        px, py, pz = e, n, -d

        # ---- orientation: q(FRD->NED) => q(FLU->ENU) ----
        q = qnorm(qmul(qmul(Q_NED_ENU, quat), Q_FRD_FLU))

        # ---- linear velocity, expressed in base_link (FLU) as nav_msgs/Odometry requires ----
        vel = (0.0, 0.0, 0.0)
        if not has_nan(lin):
            if m.velocity_frame == 1:            # NED
                vel = rotate(qconj(q), (lin[1], lin[0], -lin[2]))
            elif m.velocity_frame == 3:          # BODY_FRD
                vel = (lin[0], -lin[1], -lin[2])
            elif not self.warned_vframe:
                self.get_logger().warn(f'velocity_frame={m.velocity_frame} not handled, velocity set to 0')
                self.warned_vframe = True

        # ---- angular velocity: body FRD -> FLU ----
        w = (0.0, 0.0, 0.0)
        if not has_nan(ang):
            w = (ang[0], -ang[1], -ang[2])

        stamp = self.get_clock().now().to_msg()

        # ---- /odom ----
        o = Odometry()
        o.header.stamp = stamp
        o.header.frame_id = self.odom_frame
        o.child_frame_id = self.base_frame
        o.pose.pose.position.x, o.pose.pose.position.y, o.pose.pose.position.z = px, py, pz
        (o.pose.pose.orientation.w, o.pose.pose.orientation.x,
         o.pose.pose.orientation.y, o.pose.pose.orientation.z) = q
        o.twist.twist.linear.x, o.twist.twist.linear.y, o.twist.twist.linear.z = vel
        o.twist.twist.angular.x, o.twist.twist.angular.y, o.twist.twist.angular.z = w
        self.odom_pub.publish(o)

        # ---- TF odom -> base_link ----
        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = px, py, pz
            (t.transform.rotation.w, t.transform.rotation.x,
             t.transform.rotation.y, t.transform.rotation.z) = q
            self.tf_br.sendTransform(t)

        # ---- trajectory ----
        if self.last_xy is None or math.hypot(px - self.last_xy[0], py - self.last_xy[1]) >= self.min_dist:
            ps = PoseStamped()
            ps.header.stamp = stamp
            ps.header.frame_id = self.odom_frame
            ps.pose = o.pose.pose
            self.path.poses.append(ps)
            if len(self.path.poses) > self.max_poses:
                self.path.poses.pop(0)
            self.last_xy = (px, py)
        self.path.header.stamp = stamp
        self.path_pub.publish(self.path)

    # ------------------------------------------------------------------
    def print_status(self):
        """One line that overwrites itself (short, so it fits a narrow tmux pane)."""
        if not self.have_data:
            text = 'Waiting data from PX4...'
        else:
            n, e, d = self.ned
            text = (f'N{n:+7.2f} E{e:+7.2f} Alt{-d:5.2f} '
                    f'Yaw{self.yaw_deg:+5.0f}° V{self.speed:3.1f}')
        sys.stdout.write('\r\033[K' + text)
        sys.stdout.flush()


def main(args=None):
    rclpy.init(args=args)
    node = Px4ToOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        sys.stdout.write('\n')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()