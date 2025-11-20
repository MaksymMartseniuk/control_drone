import rclpy
from rclpy.node import Node
from ros_px4_my.msg import RCControl
from ros_px4_my.srv import TakeOff, Land
import sys
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
import threading
import time

if sys.platform == 'win32':
    import msvcrt
else:
    import tty
    import termios

msg = """
===================================
Керування PX4 Offboard Teleop
-----------------------------------
W/S: Throttle (Тяга вгору/вниз) [0.0 - 1.0]
A/D: Yaw Rate (Поворот вліво/вправо)
Стрілки: Pitch/Roll Rate (Нахили)
R: Скидання всіх команд швидкості до 0.0

Space: Arm / Disarm
Q: Перехід в режим Offboard (Після ARM)
T: Takeoff (Зліт на 5м)
L: Land (Автоматична посадка)

CTRL-C: Вихід
===================================
"""

def get_key():
    if sys.platform == 'win32':
        return msvcrt.getch().decode('utf-8')
    else:
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            key = sys.stdin.read(1)
            if key == '\x1b':
                key += sys.stdin.read(2)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return key

def save_terminal_settings():
    if sys.platform == 'win32':
        return None
    fd = sys.stdin.fileno()
    return termios.tcgetattr(fd)

def restore_terminal_settings(old_settings):
    if sys.platform == 'win32' or old_settings is None:
        return None
    fd = sys.stdin.fileno()
    termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

class RCControlNode(Node):
    def __init__(self):
        super().__init__('rc_control_node')
        self.get_logger().info(msg)

        qos_reliable = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )
        # ---PUBLISHERS---
        self.rc_command_publisher = self.create_publisher(
            RCControl,
            'rc_command',
            qos_reliable
        )
        # ---CLIENT---
        self.takeoff_client = self.create_client(TakeOff, 'takeoff_command/drone')
        self.land_client = self.create_client(Land, 'land_command/drone')

        # ---LOCAL STATE---
        self.arm_state = False
        self.offboard_state = False
        
        self.throttle_step = 0.05
        self.rate_step = 0.1
        self.max_rate = 1.5
        self.max_yaw_rate = 2.0
        self.max_throttle = 1.0 

        self.throttle_target = 0.0
        self.target_roll_rate = 0.0
        self.target_pitch_rate = 0.0
        self.target_yaw_rate = 0.0

        self.timer_period = 0.01
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.key_listener_thread = threading.Thread(target=self.key_listener, daemon=True)
        self.key_listener_thread.start()

    def reset_controls(self):
        self.throttle_target = 0.0
        self.target_roll_rate = 0.0
        self.target_pitch_rate = 0.0
        self.target_yaw_rate = 0.0
        self.get_logger().info("Controls (Rates/Throttle) reset to 0.0")

    def timer_callback(self):
        rc_control_msg = RCControl()
        rc_control_msg.arm_state = self.arm_state
        rc_control_msg.offboard_state = self.offboard_state
        
        rc_control_msg.throttle_target = self.throttle_target
        rc_control_msg.target_roll_rate = self.target_roll_rate
        rc_control_msg.target_pitch_rate = self.target_pitch_rate
        rc_control_msg.target_yaw_rate = self.target_yaw_rate

        self.rc_command_publisher.publish(rc_control_msg)

    def key_listener(self):
        key_actions = {
            'w': lambda: setattr(self, 'throttle_target', min(self.max_throttle, self.throttle_target + self.throttle_step)),
            's': lambda: setattr(self, 'throttle_target', max(0.0, self.throttle_target - self.throttle_step)), 
            
            'a': lambda: setattr(self, 'target_yaw_rate', min(self.max_yaw_rate, self.target_yaw_rate + self.rate_step)),
            'd': lambda: setattr(self, 'target_yaw_rate', max(-self.max_yaw_rate, self.target_yaw_rate - self.rate_step)),
           
            '\x1b[A': lambda: setattr(self, 'target_pitch_rate', max(-self.max_rate, self.target_pitch_rate - self.rate_step)), # arrow up
            '\x1b[B': lambda: setattr(self, 'target_pitch_rate', min(self.max_rate, self.target_pitch_rate + self.rate_step)), # arrow down
            '\x1b[C': lambda: setattr(self, 'target_roll_rate', min(self.max_rate, self.target_roll_rate + self.rate_step)),  # arrow right
            '\x1b[D': lambda: setattr(self, 'target_roll_rate', max(-self.max_rate, self.target_roll_rate - self.rate_step)),  # arrow left
            'r': self.reset_controls,
            't': self.takeoff_drone_command,
            'l': self.land_drone_command,
        }
        
        try:
            while rclpy.ok():
                key = get_key()
                
                if key == '\x03': # CTRL-C
                    self.get_logger().info("CTRL-C detected, shutting down...")
                    rclpy.shutdown()
                    break

                if key == ' ':
                    self.arm_state = not self.arm_state
                    self.get_logger().warn(f'ARM STATE REQUEST: {"ARMED" if self.arm_state else "DISARMED"}')
                    if not self.arm_state:
                        self.offboard_state = False
                        self.reset_controls()
                    continue
                
                elif key.lower() == 'q':
                    if self.arm_state:
                        self.offboard_state = True
                        self.get_logger().warn('OFFBOARD MODE REQUESTED')
                    else:
                        self.get_logger().error('Cannot enter Offboard mode while disarmed!')
                    continue

                if self.arm_state and self.offboard_state:
                    if key in key_actions:
                        key_actions[key]()

                elif key in ['t', 'l']:
                     key_actions[key]()

                self.target_roll_rate = max(-self.max_rate, min(self.max_rate, self.target_roll_rate))
                self.target_pitch_rate = max(-self.max_rate, min(self.max_rate, self.target_pitch_rate))
                self.target_yaw_rate = max(-self.max_yaw_rate, min(self.max_yaw_rate, self.target_yaw_rate))
                self.throttle_target = max(0.0, min(self.max_throttle, self.throttle_target))

                self.print_status_on_one_line()
                
        except Exception as e:
            if rclpy.ok():
                self.get_logger().error(f"Exception in key_listener: {e}")

    def print_status_on_one_line(self):
        status_msg = (
            f"ARM: {'YES' if self.arm_state else 'NO '} | "
            f"OFFB: {'YES' if self.offboard_state else 'NO '} | "
            f"Thr: {self.throttle_target: .2f} | "
            f"Pit: {self.target_pitch_rate: .1f} | "
            f"Rol: {self.target_roll_rate: .1f} | "
            f"Yaw: {self.target_yaw_rate: .1f}"
        )

        sys.stdout.write(f"\r{status_msg}")
        sys.stdout.flush()

    def takeoff_drone_command(self):
        if not self.arm_state:
            self.get_logger().error("\nCannot takeoff while disarmed! Press SPACE first.")
            return
        if not self.takeoff_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("\nTakeOff service not available!")
            return
        
        req = TakeOff.Request()
        req.target_height = 5.0
        future = self.takeoff_client.call_async(req)
        self.get_logger().info("\nSending takeoff command...")
        
        def response_callback(fut):
            try:
                resp = fut.result()
                self.get_logger().info(f"Takeoff response: {resp.message}")
                if resp.success:
                    self.offboard_state = True
            except Exception as e:
                self.get_logger().error(f"Takeoff call failed: {e}")

        future.add_done_callback(response_callback)

    def land_drone_command(self):
        if not self.land_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("\nLanding service not available")
            return

        req = Land.Request()
        self.get_logger().warn("\nSending LAND command...")
        
        future = self.land_client.call_async(req)
        
        def response_callback(fut):
            try:
                resp = fut.result()
                self.get_logger().info(f"Landing response: {resp.message}")
                if resp.success:
                    self.offboard_state = False
                    self.arm_state = False 
                    self.reset_controls()
            except Exception as e:
                self.get_logger().error(f"Landing call failed: {e}")
                
        future.add_done_callback(response_callback)

def main(args=None):
    old_settings = save_terminal_settings()
    rclpy.init(args=args)
    node = None
    try:
        node = RCControlNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.reset_controls()
            time.sleep(0.1) 
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        restore_terminal_settings(old_settings)

if __name__ == '__main__':
    main()