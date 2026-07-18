#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from std_srvs.srv import Trigger, SetBool
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist, Pose
from spot_msgs.srv import SetLocomotion, SetStairsMode
from spot_msgs.msg import Feedback, MobilityParams, PowerState

import math

# PS4 Controller Mapping (Standard ROS Joy)
BTN_SQUARE = 0
BTN_CROSS = 1
BTN_CIRCLE = 2
BTN_TRIANGLE = 3
BTN_L1 = 4
BTN_R1 = 5
BTN_L2 = 6
BTN_R2 = 7
BTN_SHARE = 8
BTN_OPTIONS = 9
BTN_PS = 10
BTN_L3 = 11
BTN_R3 = 12

BTN_DEADMAN_MANUAL = BTN_L1     # L1 (Button 4)
BTN_DEADMAN_AUTO = BTN_R1       # L2 (Button 5 as requested)

AXIS_L_LR = 0
AXIS_L_UD = 1

AXIS_R_LR = 2
AXIS_R_UD = 5

AXIS_L2 = 3
AXIS_R2 = 4

AXIS_DPAD_LR = 6
AXIS_DPAD_UD = 7

# Spot Constants
MAX_LINEAR_VEL = 1.0  # m/s
MAX_ANGULAR_VEL = 1.0 # rad/s
MAX_BODY_HEIGHT = 0.3 # m
BODY_HEIGHT_STEP = 0.05
MAX_ROLL = 0.4
MAX_PITCH = 0.4
MAX_YAW = 0.4

# Speed control (L2)
SLOW_SPEED_SCALE = 0.5
FAST_SPEED_SCALE = 1.0
L2_AXIS_PRESSED_THRESHOLD = 0.5 # (not used in the new robust detection)

#how far the L2 axis must move from neutral to count as "pressed"
L2_AXIS_DELTA_THRESHOLD = 0.2

# joy_linux reports trigger axes as 0.0 until the trigger is first physically
# actuated, after which they rest at 1.0 (unpressed) / -1.0 (fully pressed).
# Using 0.0 as neutral causes the trigger to appear permanently pressed once
# it has been touched.  Default to the true hardware rest value instead.
L2_AXIS_DEFAULT_NEUTRAL = 1.0

def btnv(msg: Joy, i: int) -> int:
    return msg.buttons[i] if i < len(msg.buttons) else 0

def axv(msg: Joy, i: int) -> float:
    return msg.axes[i] if i < len(msg.axes) else 0.0

def dz(x, dead=0.08):
    if abs(x) < dead:
        return 0.0
    # rescale so it still hits full speed at the edge
    return math.copysign((abs(x) - dead) / (1.0 - dead), x)

class SpotJoyTeleop(Node):
    def __init__(self):
        super().__init__('spot_joy_teleop')
        self.l2_neutral = L2_AXIS_DEFAULT_NEUTRAL
        self.l2_btn_neutral = None

        self.is_sitting = False
        self.suppress_until = self.get_clock().now()
        self.prev_buttons = []
        self.prev_dpad_ud = 0.0
        # State
        self.body_height = 0.0
        self.in_stand_mode = False
        self.last_feedback = None
        self.last_power_state = None

        # Parameters
        self.declare_parameter('verbose', False)
        self.verbose = self.get_parameter('verbose').get_parameter_value().bool_value
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 1)
        self.body_pose_pub = self.create_publisher(Pose, 'body_pose', 1)
        
        # Service Clients
        self.cli_claim = self.create_client(Trigger, 'claim')
        self.cli_power_on = self.create_client(Trigger, 'power_on')
        self.cli_stand = self.create_client(Trigger, 'stand')
        self.cli_sit = self.create_client(Trigger, 'sit')
        self.cli_stop = self.create_client(Trigger, 'stop')
        self.cli_self_right = self.create_client(Trigger, 'self_right')
        self.cli_rollover = self.create_client(Trigger, 'rollover')
        self.cli_locomotion = self.create_client(SetLocomotion, 'locomotion_mode')
        self.cli_stairs = self.create_client(SetStairsMode, 'stairs_mode')
        self.cli_dog_mode = self.create_client(SetBool, '/dog_mode/enable')
        self.dog_mode_enabled = False
        
        # Subscription
        self.joy_sub = self.create_subscription(Joy, 'joy', self.joy_callback, 1)
        self.cmd_vel_int_sub = self.create_subscription(Twist, '/cmd_vel_intermediate', self.cmd_vel_intermediate_callback, 1)
        self.feedback_sub = self.create_subscription(Feedback, '/status/feedback', self.feedback_callback, 10)
        self.power_sub = self.create_subscription(PowerState, '/status/power_states', self.power_callback, 10)
        
        self.auto_active = False

        self.get_logger().info("Spot Joy Teleop Node Started")

    def publish_neutral_pose(self):
        pose = Pose()
        pose.position.z = self.body_height
        pose.orientation.w = 1.0
        self.body_pose_pub.publish(pose)

    def released(self, msg, idx: int) -> bool:
        if not self.prev_buttons or idx >= len(self.prev_buttons):
            return False
        return (btnv(msg, idx) == 0) and (self.prev_buttons[idx] == 1)

    def pressed(self, msg, idx: int) -> bool:
        if not self.prev_buttons or idx >= len(self.prev_buttons):
            return False
        return (btnv(msg, idx) == 1) and (self.prev_buttons[idx] == 0)

    def joy_callback(self, msg: Joy):
        now = self.get_clock().now()
        if now < self.suppress_until:
            # make the messages quiet so we don't fight sit/stand transitions
            self.prev_buttons = [btnv(msg, i) for i in range(len(msg.buttons))]
            self.prev_dpad_ud = axv(msg, AXIS_DPAD_UD)

            if self.l2_btn_neutral is None:
                self.l2_btn_neutral = btnv(msg, BTN_L2)

            return

        # --- Service Calls (Edge Detection / Button Presses) ---
        if (not self.prev_buttons) or (len(self.prev_buttons) != len(msg.buttons)):
            self.prev_buttons = [btnv(msg, i) for i in range(len(msg.buttons))]
            self.prev_dpad_ud = axv(msg, AXIS_DPAD_UD)

            if self.l2_btn_neutral is None:
                self.l2_btn_neutral = btnv(msg, BTN_L2)

            return

        # START: Claim & Power On
        if self.pressed(msg, BTN_OPTIONS):
            self.call_trigger(self.cli_claim, "Claim")
            self.call_trigger(self.cli_power_on, "Power On")

        # Stand: square
        if self.pressed(msg, BTN_SQUARE):
            self.is_sitting = False
            self.call_trigger(self.cli_stand, "Stand")
            self.in_stand_mode = True
            self.get_logger().info(
                "Stand mode requested. Hold L1 for body pose control; press Triangle to switch back to walk mode."
            )
            
        # Sit: cross
        if self.pressed(msg, BTN_CROSS):
            # Stop any motion and then suppress teleop for a moment
            self.is_sitting = True
            self.cmd_vel_pub.publish(Twist())

            self.call_trigger(self.cli_sit, "Sit")
            self.suppress_until = self.get_clock().now() + Duration(seconds=2.0)
            self.in_stand_mode = False

        # Walk: triangle
        if self.pressed(msg, BTN_TRIANGLE):
            self.is_sitting = False
            self.publish_neutral_pose()
            self.set_locomotion(1)
            self.in_stand_mode = False
            self.get_logger().info("Walk mode requested. Hold L1 for velocity control.")

        # Circle: toggle Auto Dog Mode (spot_dog_mode gaze controller)
        if self.pressed(msg, BTN_CIRCLE):
            self.dog_mode_enabled = not self.dog_mode_enabled
            self.call_dog_mode(self.dog_mode_enabled)

        # D-PAD Up/Down: Body Height
        if len(msg.axes) > AXIS_DPAD_UD:
            dpad_ud = axv(msg, AXIS_DPAD_UD)
        else:
            dpad_ud = 0.0
        if dpad_ud > 0.5 and self.prev_dpad_ud <= 0.5:
            self.change_height(BODY_HEIGHT_STEP)
        elif dpad_ud < -0.5 and self.prev_dpad_ud >= -0.5:
            self.change_height(-BODY_HEIGHT_STEP)
        self.prev_dpad_ud = dpad_ud

        # early return if sitting
        if self.is_sitting:
            self.prev_buttons = [btnv(msg, i) for i in range(len(msg.buttons))]
            self.prev_dpad_ud = axv(msg, AXIS_DPAD_UD)
            return

        # --- Continuous Control (cmd_vel / body_pose) ---

        # Deadman Checks
        manual_active = btnv(msg, BTN_DEADMAN_MANUAL)
        self.auto_active = (btnv(msg, BTN_DEADMAN_AUTO) == 1)

        if self.released(msg, BTN_DEADMAN_MANUAL):
            self.publish_neutral_pose()
            self.cmd_vel_pub.publish(Twist())

        if manual_active:
            if self.in_stand_mode:
                if self.verbose:
                    self.get_logger().info("Manual Mode: Body Pose Control", throttle_duration_sec=1.0)

                pose_msg = Pose()
                pose_msg.position.z = self.body_height

                roll  = dz(axv(msg, AXIS_L_LR)) * MAX_ROLL
                pitch = dz(axv(msg, AXIS_R_UD)) * MAX_PITCH
                yaw   = dz(axv(msg, AXIS_R_LR)) * MAX_YAW

                cy = math.cos(yaw * 0.5)
                sy = math.sin(yaw * 0.5)
                cp = math.cos(pitch * 0.5)
                sp = math.sin(pitch * 0.5)
                cr = math.cos(roll * 0.5)
                sr = math.sin(roll * 0.5)

                pose_msg.orientation.w = cr * cp * cy + sr * sp * sy
                pose_msg.orientation.x = sr * cp * cy - cr * sp * sy
                pose_msg.orientation.y = cr * sp * cy + sr * cp * sy
                pose_msg.orientation.z = cr * cp * sy - sr * sp * cy

                self.body_pose_pub.publish(pose_msg)

            else:
                if self.verbose:
                    self.get_logger().info("Manual Mode: Velocity Control", throttle_duration_sec=1.0)

                twist = Twist()

                lx = dz(axv(msg, AXIS_L_UD))
                ly = dz(axv(msg, AXIS_L_LR))
                az = dz(axv(msg, AXIS_R_LR))

                # Speed control (FIXED)
                l2_axis = axv(msg, AXIS_L2)
                l2_btn  = btnv(msg, BTN_L2)

                if self.l2_btn_neutral is None:
                    self.l2_btn_neutral = l2_btn

                l2_btn_usable = (self.l2_btn_neutral == 0)
                l2_btn_pressed = (l2_btn_usable and l2_btn == 1)

                l2_axis_pressed = (abs(l2_axis - self.l2_neutral) > L2_AXIS_DELTA_THRESHOLD)

                l2_pressed = l2_btn_pressed or l2_axis_pressed

                speed_scale = FAST_SPEED_SCALE if l2_pressed else SLOW_SPEED_SCALE
                max_lin = MAX_LINEAR_VEL * speed_scale
                max_ang = MAX_ANGULAR_VEL * speed_scale

                if self.verbose:
                    self.get_logger().info(
                        f"speed_scale={speed_scale:.2f} l2_btn={l2_btn} (neutral={self.l2_btn_neutral}) "
                        f"l2_axis={l2_axis:.2f} (neutral={self.l2_neutral:.2f}) pressed={l2_pressed}",
                        throttle_duration_sec=1.0
                    )

                twist.linear.x = lx * max_lin
                twist.linear.y = ly * max_lin
                twist.angular.z = az * max_ang
                self.warn_if_drive_not_ready(twist)
                self.cmd_vel_pub.publish(twist)

        elif self.auto_active:
            if self.verbose:
                self.get_logger().info("Auto Mode: Yielding control", throttle_duration_sec=1.0)
            pass

        else:
            if self.verbose:
                self.get_logger().info("Safety: Stopped", throttle_duration_sec=1.0)
            self.cmd_vel_pub.publish(Twist())

        self.prev_buttons = [btnv(msg, i) for i in range(len(msg.buttons))]

    def call_trigger(self, client, name):
        if name in ["Self Right", "Rollover"]:
            self.get_logger().warn(f"Service {name} not available")
            return
        if not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(f"Service {name} not available")
            return

        req = Trigger.Request()
        future = client.call_async(req)

        def _done(fut):
            try:
                resp = fut.result()
                self.get_logger().info(f"{name}: success={resp.success} msg='{resp.message}'")
            except Exception as e:
                self.get_logger().error(f"{name} service call failed: {e}")

        future.add_done_callback(_done)

    def call_dog_mode(self, enable: bool):
        if not self.cli_dog_mode.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Dog mode service not available")
            self.dog_mode_enabled = not enable
            return
        req = SetBool.Request()
        req.data = enable
        self.cli_dog_mode.call_async(req)
        self.get_logger().info(f"Auto Dog Mode: {'ON' if enable else 'OFF'}")

    def set_locomotion(self, mode_id):
        if not self.cli_locomotion.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Locomotion service not available")
            return
        req = SetLocomotion.Request()
        req.locomotion_mode = mode_id
        self.cli_locomotion.call_async(req)
        self.get_logger().info(f"Set Locomotion: {mode_id}")

    def set_stairs_mode(self, enabled):
        if not self.cli_stairs.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Stairs mode service not available")
            return
        req = SetStairsMode.Request()
        req.data = enabled
        self.cli_stairs.call_async(req)
        self.get_logger().info(f"Set Stairs Mode: {enabled}")

    def change_height(self, delta):
        self.body_height += delta
        self.body_height = max(-MAX_BODY_HEIGHT, min(MAX_BODY_HEIGHT, self.body_height))
        pose = Pose()
        pose.position.z = self.body_height
        pose.orientation.w = 1.0
        self.body_pose_pub.publish(pose)
        self.get_logger().info(f"Adjusted Height: {self.body_height:.2f}")

    def cmd_vel_intermediate_callback(self, msg: Twist):
        if self.auto_active:
            self.cmd_vel_pub.publish(msg)

    def feedback_callback(self, msg: Feedback):
        self.last_feedback = msg

    def power_callback(self, msg: PowerState):
        self.last_power_state = msg

    def warn_if_drive_not_ready(self, twist: Twist):
        moving = (
            abs(twist.linear.x) > 1e-3
            or abs(twist.linear.y) > 1e-3
            or abs(twist.angular.z) > 1e-3
        )
        if not moving:
            return

        reasons = []
        if self.last_power_state is not None and self.last_power_state.motor_power_state != PowerState.STATE_ON:
            reasons.append(f"motor_power_state={self.last_power_state.motor_power_state}")
        if self.last_feedback is not None and not self.last_feedback.standing:
            reasons.append(
                f"standing={self.last_feedback.standing} sitting={self.last_feedback.sitting} moving={self.last_feedback.moving}"
            )

        if reasons:
            self.get_logger().warn(
                "Manual drive requested while Spot may not be ready for walking "
                f"({', '.join(reasons)}). Press Triangle for walk mode and confirm Spot is claimed/powered.",
                throttle_duration_sec=1.0,
            )

def main(args=None):
    rclpy.init(args=args)
    node = SpotJoyTeleop()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
