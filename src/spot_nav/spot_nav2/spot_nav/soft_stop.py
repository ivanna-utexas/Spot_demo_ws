#!/usr/bin/env python3
"""Soft stop — press Enter to cancel nav2 goals and stop the robot.

This is NOT a hardware E-Stop. It depends on rclpy, DDS, and the driver
being responsive. The tablet's physical E-Stop button is the real safety layer.

Command pipeline: controller → /cmd_vel_nav → velocity_smoother → /cmd_vel → Spot driver
We publish zeros on BOTH topics to stop at every level.
"""

import time
import threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger
from action_msgs.srv import CancelGoal


GREEN = '\033[92m'
RED = '\033[91m'
YELLOW = '\033[93m'
BOLD = '\033[1m'
RESET = '\033[0m'

CANCEL_SERVICES = [
    '/navigate_to_pose/_action/cancel_goal',
    '/navigate_through_poses/_action/cancel_goal',
    '/follow_path/_action/cancel_goal',
]


class SoftStop(Node):
    def __init__(self):
        super().__init__('soft_stop')
        # Publish on both topics to stop at every level of the pipeline
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.cmd_vel_nav_pub = self.create_publisher(Twist, '/cmd_vel_nav', 10)
        self.stop_client = self.create_client(Trigger, '/stop')
        self.release_client = self.create_client(Trigger, '/release')
        self.cancel_clients = [
            self.create_client(CancelGoal, name) for name in CANCEL_SERVICES
        ]
        
    def check_readiness(self):
        print('\nChecking service readiness...')
        for name, client in [('/stop', self.stop_client), ('/release', self.release_client)]:
            if client.wait_for_service(timeout_sec=5.0):
                print(f'  {GREEN}[OK]{RESET}  {name}')
            else:
                print(f'  {RED}[NOT FOUND]{RESET}  {name}')
        for name, client in zip(CANCEL_SERVICES, self.cancel_clients):
            if client.wait_for_service(timeout_sec=2.0):
                print(f'  {GREEN}[OK]{RESET}  {name}')
            else:
                print(f'  {YELLOW}[NOT YET]{RESET}  {name} (normal if nav2 not started)')
        print()

    def stop(self):
        # 1. Spot /stop — closest to hardware, do this first
        try:
            self.stop_client.call_async(Trigger.Request())
            self.get_logger().info('Called /stop')
        except Exception as e:
            self.get_logger().error(f'/stop failed: {e}')

        # 2. Cancel all nav2 goals — stop the source of cmd_vel
        req = CancelGoal.Request()
        for name, client in zip(CANCEL_SERVICES, self.cancel_clients):
            try:
                client.call_async(req)
                self.get_logger().info(f'Cancelling {name}')
            except Exception as e:
                self.get_logger().warn(f'{name}: {e}')

        # 3. Zero velocity burst in a separate thread
        #    Publishes on BOTH /cmd_vel_nav (before smoother) and /cmd_vel (after smoother)
        #    50 Hz for 2 seconds = 100 messages, overwhelming any residual nav2 output
        threading.Thread(target=self._zero_burst, daemon=True).start()

    def _zero_burst(self):
        zero = Twist()
        for _ in range(100):  # 50 Hz * 2 seconds
            self.cmd_vel_nav_pub.publish(zero)
            self.cmd_vel_pub.publish(zero)
            time.sleep(1.0 / 50.0)
        self.get_logger().info('Zero velocity burst complete (2s)')

    def release_lease(self):
        try:
            self.release_client.call_async(Trigger.Request())
            self.get_logger().info('Called /release — lease released')
            print(f'{YELLOW}Lease released. Tablet can take over.{RESET}')
            print(f'{YELLOW}To re-claim: press Options on PS4 controller.{RESET}\n')
        except Exception as e:
            self.get_logger().error(f'/release failed: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = SoftStop()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    node.check_readiness()

    print('=' * 50)
    print(f'  {BOLD}{RED}SOFT STOP{RESET}')
    print(f'  Press {BOLD}ENTER{RESET} to stop the robot')
    print(f'  Type {BOLD}r{RESET} + ENTER to release lease (tablet takeover)')
    print('=' * 50 + '\n')

    try:
        while rclpy.ok():
            line = input()
            if line.strip().lower() == 'r':
                node.release_lease()
            else:
                print(f'{RED}*** STOPPING ***{RESET}')
                node.stop()
                print('Robot stopped. Press ENTER to stop again.\n')
    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
