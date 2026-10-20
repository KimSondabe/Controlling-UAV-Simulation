#!/usr/bin/env python3
"""
            Node ROS 2 control UAV with PX4_SITL Gazebo 

------------------------------TURTORIAL-----------------------------------------
                T               :   TAKE OFF
                W / S           :   FLY FORWARD     (X)
                A / D           :   LEFT/RIGHT      (Y)
                UP/DOWN ARROW   :   INCREASE HEIGHT (Z)
                LEFT/RIGHT ARROW:   TURN LEFT/RIGHT (YAWN)
                Space           :   STOP AND HOLD POSITION
                M               :   FLY WITH MISSION
                L               :   LAND
                Q / Ctrl+C      :   EXIT
                    COORDINATE SYSTEMS NORTH-EAST-DOWN
--------------------------------------------------------------------------------
"""

import math 

#Read keyboard libraries
import os
import select
import sys
import termios
import tty

#ROS2 Libraries
import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy)
from px4_msgs.msg import (OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus)

#----Manual control Parameters----
TAKEOFF_ALT = 2.5      #TAKE OFF HEIGHT (m)
SPEED_XY = 1.5         #m/s
SPEED_Z = 1.0          #m/s
YAW_RATE = math.radians(45.0)  # rad/s
KEY_HOLD_TIME = 0.35   #
MIN_ALT = 0.3          #MIN HEIGHT FOR FLY
RATE_HZ = 20.0  


#----Missions Parameters----
MISSION_YAW_RATE = math.radians(40.0)
TOL_POS = 0.5
TOL_YAW = math.radians(8.0)
KP_MISSION = 1.0

HELP = __doc__

DEFAULT_MISSION = [
    dict(x=1.0,  y=0.0, z=-3.0,  yaw=45.0,  hold=5.0),
    dict(x=10.0, y=0.0, z=-3.0,  yaw=0.0,   hold=5.0),
    dict(x=5.0,  y=5.0, z=-5.0,  yaw=90.0,  hold=5.0),
    dict(x=7.0,  y=5.0, z=-10.0, yaw=180.0, hold=5.0),
    dict(x=8.0,  y=0.0, z=-9.0,  yaw=-90.0, hold=5.0),
    dict(x=0.0,  y=6.0, z=-4.5,  yaw=0.0,   hold=5.0),
]

MOVEMENT_KEY = {'w', 's', 'a', 'd', 'UP', 'DOWN', 'LEFT', 'RIGHT'}

def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))

def to_ned_goal(origin, wp):
    return [origin[0] + wp['x'], origin[1] + wp['y'], origin[2] + wp['z']]

def yaw_to_rad(deg):
    return wrap(math.radians(deg))

class KeyboardOffboard(Node):
    def __init__(self):
        super().__init__('control_node')
        #Setting Quality of Service
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL, #Keep n last messages in history (n = depth)
            history=HistoryPolicy.KEEP_LAST,depth=1)

        qos_sub = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE, #Not keep anything
            history=HistoryPolicy.KEEP_LAST,depth=1)

        #Declare parameter
        self.declare_parameter('local_position_topic', '/fmu/out/vehicle_local_position_v1')
        self.declare_parameter('status_topic', '/fmu/out/vehicle_status_v4')
        lp_topic = self.get_parameter('local_position_topic').value
        st_topic = self.get_parameter('status_topic').value

        self.mission = DEFAULT_MISSION

        #Create publisher and subsriber
        self.pub_mode = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos)
        self.pub_sp = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos)
        self.pub_cmd = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos)
        self.create_subscription(VehicleLocalPosition, lp_topic, self.on_local_pos, qos_sub)
        self.create_subscription(VehicleStatus, st_topic, self.on_status, qos_sub)
        
        #Initialize status variables
        self.local_pos = None
        self.status = None
        self.origin = None
        self.check_armed = False

        #Initialize mission variables
        self.mode = 'MANUAL'
        self.idx = 0
        self.reached_since = None 

        #Initialize position variables and control
        self.hold = [0.0, 0.0, 0.0]   # hold position (x,y,z) (NED)
        self.yaw = 0.0                # yaw (rad)
        self.moving = False
        self.cmd_body = [0.0, 0.0, 0.0]  # forward, right, down-vel
        self.yaw_cmd = 0.0
        self.last_key_time = {}       # last time recive keyboard


        #Initialize controling status program variables
        self.tick = 0
        self.takeoff_requested = False
        self.takeoff_tick = 0
        self.stream_point_time = 20
        self.landing = False
        self.running = True

        # Terminal raw mode
        self.fd = sys.stdin.fileno() #Recive the input of terminal
        self.old_term = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd) #Change terminal into cbreak mode to recive character at the moment instead of waiting Enter

        self.dt = 1.0 / RATE_HZ
        self.create_timer(self.dt, self.loop)
        print(HELP)

    # ---------- Callbacks ----------
    def on_local_pos(self, msg):
        first_time_recive = self.local_pos is None
        self.local_pos = msg
        if first_time_recive:
            self.origin = [msg.x, msg.y, msg.z]
            self.hold = [msg.x, msg.y, msg.z]
            self.yaw = msg.heading

    def on_status(self, msg):
        self.status = msg

    def is_ready_to_fly(self): #Check if drone is ready to fly
        return (self.status is not None and self.status.arming_state == VehicleStatus.ARMING_STATE_ARMED and self.status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD)


    # ---------- KEYBOARD ----------
    def read_keys(self):
        keys = []
        while select.select([sys.stdin], [], [], 0)[0]:
            data = os.read(self.fd, 64)
            if not data:
                break
            s = data.decode(errors='ignore')
            i = 0
            while i < len(s):
                if s[i] == '\x1b' and i + 2 < len(s) and s[i + 1] in '[O':
                    code = s[i + 2]
                    mapping = {'A': 'UP', 'B': 'DOWN', 'C': 'RIGHT', 'D': 'LEFT'}
                    if code in mapping:
                        keys.append(mapping[code])
                    i += 3
                else:
                    keys.append(s[i].lower())
                    i += 1
        return keys

    def process_keys(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        for k in self.read_keys():
            if self.mode == 'MISSION' and (k in MOVEMENT_KEY or k == ' '):
                self.cancel_mission('CHANGE TO USER CONTROL')
                continue
            if k in MOVEMENT_KEY:
                self.last_key_time[k] = now
            elif k == ' ':
                self.last_key_time.clear()
            elif k == 't':
                self.request_takeoff()
            elif k == 'm':
                self.start_mission()
            elif k == 'l':
                self.land()
            elif k == 'q' or k == '\x03':
                self.running = False

        def active(key):
            return now - self.last_key_time.get(key, -1e9) < KEY_HOLD_TIME

        fwd = (1 if active('w') else 0) - (1 if active('s') else 0)
        right = (1 if active('d') else 0) - (1 if active('a') else 0)
        up = (1 if active('UP') else 0) - (1 if active('DOWN') else 0)
        yaw_dir = (1 if active('RIGHT') else 0) - (1 if active('LEFT') else 0)

        # Won't let lower if near land
        if up < 0 and self.local_pos is not None and -self.local_pos.z < MIN_ALT:
            up = 0

        self.cmd_body = [fwd * SPEED_XY, right * SPEED_XY, -up * SPEED_Z]
        self.yaw_cmd = yaw_dir * YAW_RATE

    # ---------- MISSION ----------
    def start_mission(self):
        if self.local_pos is None:
            self.get_logger().warn('Location not yet recived')
            return
        if self.mode == 'MISSION':
            return
        #Start from current position
        self.hold = [self.local_pos.x, self.local_pos.y, self.local_pos.z]
        self.yaw = self.local_pos.heading
        self.idx = 0
        self.reached_since = None
        self.mode = 'MISSION'
        self.moving = False
        if not self.takeoff_requested:
            self.landing = False
            self.takeoff_requested = True
            self.check_armed = False
            self.takeoff_tick = self.tick
        self.get_logger().info(
            f'Start mission ({len(self.mission)} waypoint). ''Press Space to cancle mission')

    def cancel_mission(self, reason):
        self.mode = 'MANUAL'
        self.last_key_time.clear()
        if self.local_pos is not None:
            self.hold = [self.local_pos.x, self.local_pos.y, self.local_pos.z]
            self.yaw = self.local_pos.heading
        self.get_logger().warn(f'End mission: {reason}. Holding postion...')

    def run_mission(self):
        wp = self.mission[self.idx]
        goal = to_ned_goal(self.origin, wp)
        yaw_deg = wp.get('yaw')
        goal_yaw = None if yaw_deg is None else yaw_to_rad(yaw_deg)

        p = self.local_pos
        dx = goal[0] - p.x
        dy = goal[1] - p.y
        dz = goal[2] - p.z
        dxy = math.hypot(dx, dy)
        dist = math.sqrt(dxy ** 2 + dz ** 2)

        vx = vy = 0.0
        if dxy > 1e-3:
            v_xy = min(SPEED_XY, KP_MISSION * dxy)
            vx = dx / dxy * v_xy
            vy = dy / dxy * v_xy
        vz = max(-SPEED_Z, min(SPEED_Z, KP_MISSION * dz))

        if goal_yaw is not None:
            sy = MISSION_YAW_RATE * self.dt
            err = wrap(goal_yaw - self.yaw)
            self.yaw = wrap(self.yaw + max(-sy, min(sy, err)))

        yaw_ok = goal_yaw is None or abs(wrap(goal_yaw - p.heading)) < (TOL_YAW * 1.5)

        if dist < TOL_POS and yaw_ok:
            now = self.get_clock().now().nanoseconds * 1e-9
            if self.reached_since is None:
                self.reached_since = now
                self.get_logger().info(
                    f'Go to waypoint {self.idx + 1}/{len(self.mission)}, holding position...')
            if now - self.reached_since >= wp.get('hold', 0.0):
                self.get_logger().info(f'Done waypoint {self.idx + 1}.')
                self.idx += 1
                self.reached_since = None
                if self.idx >= len(self.mission):
                    self.mode = 'MANUAL'
                    self.hold = [p.x, p.y, p.z]   
                    self.yaw = p.heading
                    self.get_logger().info('Done mission, press L to land.')
                    return 0.0, 0.0, 0.0
        else:
            if dist > TOL_POS * 1.5:
                self.reached_since = None

        return vx, vy, vz


    # ---------- PX4 COMMANDS ----------
    def timestamp(self):
        return int(self.get_clock().now().nanoseconds / 1000)

    #Send commands
    def send_cmd(self, command, p1=0.0, p2=0.0):
        m = VehicleCommand()
        m.command = command
        m.param1 = p1
        m.param2 = p2
        m.target_system = 1
        m.target_component = 1
        m.source_system = 1
        m.source_component = 1
        m.from_external = True
        m.timestamp = self.timestamp()
        self.pub_cmd.publish(m)

    def request_takeoff(self):
        if self.local_pos is None:
            self.get_logger().warn('Warning can not recive local position from PX4 ')
            return
        self.landing = False
        self.takeoff_requested = True
        self.check_armed = False
        self.takeoff_tick = self.tick
        self.hold = [self.local_pos.x, self.local_pos.y, self.local_pos.z - TAKEOFF_ALT]
        self.yaw = self.local_pos.heading
        self.get_logger().info(f'Taking off to (1) {TAKEOFF_ALT} m...')

    def land(self):
        self.mode = 'MANUAL'
        self.landing = True
        self.takeoff_requested = False
        self.send_cmd(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info('Landing...')

    # ---------- MAIN LOOP ----------
    def loop(self):
        if not self.running:
            self.shutdown()
            return

        self.process_keys()
        self.tick += 1

        if self.local_pos is None or self.landing:
            return

        if self.takeoff_requested:
            t = self.tick - self.takeoff_tick
            if self.is_ready_to_fly():
                self.check_armed = True           
            elif not self.check_armed and t >= 40 and t % 20 == 0:
                self.send_cmd(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
                self.send_cmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)

        nan = float('nan')
        sp = TrajectorySetpoint()
        mode = OffboardControlMode()

        ned_vel = None

        if self.mode == 'MISSION':
            if self.is_ready_to_fly():
                ned_vel = self.run_mission()
            else:
                ned_vel = (0.0, 0.0, 0.0)
            if self.mode != 'MISSION':        
                ned_vel = None
        else:
            self.yaw = wrap(self.yaw + self.yaw_cmd * self.dt)
            if self.takeoff_requested and any(abs(v) > 1e-6 for v in self.cmd_body):
                f, r, vz = self.cmd_body
                vx = math.cos(self.yaw) * f - math.sin(self.yaw) * r
                vy = math.sin(self.yaw) * f + math.cos(self.yaw) * r
                ned_vel = (vx, vy, vz)

        if ned_vel is not None:
            sp.position = [nan, nan, nan]
            sp.velocity = [float(v) for v in ned_vel]
            mode.position = False
            mode.velocity = True
            self.moving = (self.mode == 'MANUAL')
        else:
            if self.moving and self.takeoff_requested:
                self.hold = [self.local_pos.x, self.local_pos.y, self.local_pos.z]
            self.moving = False
            sp.position = [float(v) for v in self.hold]
            sp.velocity = [nan, nan, nan]
            mode.position = True
            mode.velocity = False

        sp.acceleration = [nan, nan, nan]
        sp.yaw = float(self.yaw)
        sp.yawspeed = nan
        sp.timestamp = self.timestamp()

        mode.acceleration = False
        mode.attitude = False
        mode.body_rate = False
        mode.timestamp = self.timestamp()

        self.pub_mode.publish(mode)
        self.pub_sp.publish(sp)
    
    def shutdown(self):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_term)
        self.get_logger().info('Exiting...')
        rclpy.shutdown()
    
    def destroy_node(self):
        try:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_term)
        except Exception:
            pass
        super().destroy_node()
    


def main():
    rclpy.init()
    node = KeyboardOffboard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()