import rclpy
from rclpy.node import Node
from ros_px4_my.msg import RCControl
from ros_px4_my.srv import TakeOff, Land,ServoCommand
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
W/S: Throttle (Тяга вгору/вниз)
A/D: Yaw Rate (Поворот)
Стрілки: Pitch/Roll Rate
R: Скидання (Reset)

Space: Arm / Disarm
Q: Перемикач Offboard (Тільки якщо ARMED)
T: Takeoff (Зліт)
L: Land (Посадка)

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
        self.get_logger().info(f"{msg}")

        qos_reliable = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )
        # ---PUBLISHERS---
        self.rc_command_publisher = self.create_publisher(
            RCControl, 'rc_command', qos_reliable)
        
        # ---CLIENTS---
        self.takeoff_client = self.create_client(TakeOff, 'takeoff_command/drone')
        self.land_client = self.create_client(Land, 'land_command/drone')

        self.servo_client=self.create_client(ServoCommand,"servo_command/drone")

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

        self.status_message = "Ready"
        self.target_mount_rad=0.0
        self.target_pole_rad=0.0
        self.mount_step = 0.1
        self.pole_step = 0.1

        self.timer_period = 0.02
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.key_listener_thread = threading.Thread(target=self.key_listener, daemon=True)
        self.key_listener_thread.start()

    def reset_controls(self):
        self.throttle_target = 0.0
        self.target_roll_rate = 0.0
        self.target_pitch_rate = 0.0
        self.target_yaw_rate = 0.0
        self.status_message = "Controls Reset"

    def timer_callback(self):
        rc_control_msg = RCControl()
        rc_control_msg.arm_state = self.arm_state
        rc_control_msg.offboard_state = self.offboard_state
        
        rc_control_msg.throttle_target = self.throttle_target
        rc_control_msg.target_roll_rate = self.target_roll_rate
        rc_control_msg.target_pitch_rate = self.target_pitch_rate
        rc_control_msg.target_yaw_rate = self.target_yaw_rate

        rc_control_msg.target_mount_rad = float(self.target_mount_rad)
        rc_control_msg.target_pole_rad = float(self.target_pole_rad)

        self.rc_command_publisher.publish(rc_control_msg)

    def key_listener(self):
        move_actions = {
            'w': lambda: setattr(self, 'throttle_target', min(self.max_throttle, self.throttle_target + self.throttle_step)),
            's': lambda: setattr(self, 'throttle_target', max(0.0, self.throttle_target - self.throttle_step)), 
            'a': lambda: setattr(self, 'target_yaw_rate', min(self.max_yaw_rate, self.target_yaw_rate + self.rate_step)),
            'd': lambda: setattr(self, 'target_yaw_rate', max(-self.max_yaw_rate, self.target_yaw_rate - self.rate_step)),
            '\x1b[A': lambda: setattr(self, 'target_pitch_rate', max(-self.max_rate, self.target_pitch_rate - self.rate_step)),
            '\x1b[B': lambda: setattr(self, 'target_pitch_rate', min(self.max_rate, self.target_pitch_rate + self.rate_step)),
            '\x1b[C': lambda: setattr(self, 'target_roll_rate', min(self.max_rate, self.target_roll_rate + self.rate_step)),
            '\x1b[D': lambda: setattr(self, 'target_roll_rate', max(-self.max_rate, self.target_roll_rate - self.rate_step)),
            '4': lambda: setattr(self, 'target_mount_rad', self.target_mount_rad + self.mount_step),
            '6': lambda: setattr(self, 'target_mount_rad', self.target_mount_rad - self.mount_step),
            '8': lambda: setattr(self, 'target_pole_rad', self.target_pole_rad + self.pole_step),
            '2': lambda: setattr(self, 'target_pole_rad', self.target_pole_rad - self.pole_step),
        }
        
        try:
            while rclpy.ok():
                key = get_key()
                
                if key == '\x03': # CTRL-C
                    self.get_logger().info("Shutting down...")
                    rclpy.shutdown()
                    break

                
                if key == ' ':
                    self.arm_state = not self.arm_state
                    if not self.arm_state:
                        self.offboard_state = False
                        self.reset_controls()
                        self.status_message = "DISARMED"
                    else:
                        self.status_message = "ARMED"
                    
                elif key.lower() == 'q':
                    if self.arm_state:
                        self.offboard_state = not self.offboard_state
                        self.status_message = f"OFFBOARD: {'ON' if self.offboard_state else 'OFF'}"
                    else:
                        self.status_message = "ERR: Must ARM first!"

                elif key.lower() == 't':
                    self.takeoff_drone_command()
                
                elif key.lower() == 'l':
                    self.land_drone_command()
                
                elif key.lower() == 'r':
                    self.reset_controls()
                
                elif key.lower()=='c':
                    self.servo_command()

                elif self.arm_state:
                    if key in move_actions:
                        move_actions[key]()

                self.target_roll_rate = max(-self.max_rate, min(self.max_rate, self.target_roll_rate))
                self.target_pitch_rate = max(-self.max_rate, min(self.max_rate, self.target_pitch_rate))
                self.target_yaw_rate = max(-self.max_yaw_rate, min(self.max_yaw_rate, self.target_yaw_rate))
                self.throttle_target = max(0.0, min(self.max_throttle, self.throttle_target))

                self.print_status_on_one_line()
                
        except Exception as e:
            pass

    def print_status_on_one_line(self):
        status_str = (
            f"ARM:{'YES' if self.arm_state else 'NO '}|"
            f"OFFB:{'YES' if self.offboard_state else 'NO '}|"
            f"T:{self.throttle_target:.2f}|"
            f"R:{self.target_roll_rate:.1f}|"
            f"P:{self.target_pitch_rate:.1f}|"
            f"Y:{self.target_yaw_rate:.1f} || "
            f"MSG: {self.status_message}"
        )
        sys.stdout.write(f"\r{status_str}\033[K")
        sys.stdout.flush()

    def takeoff_drone_command(self):
        if not self.arm_state:
            self.status_message = "ERR: Arm before Takeoff!"
            return
        
        if not self.takeoff_client.wait_for_service(timeout_sec=0.5):
            self.status_message = "ERR: Service Unavailable"
            return
        
        self.status_message = "Sending Takeoff..."
        req = TakeOff.Request()
        req.target_height = 5.0
        future = self.takeoff_client.call_async(req)
        
        def response_callback(fut):
            try:
                resp = fut.result()
                if resp.success:
                    self.status_message = "Takeoff: OK"
                    self.offboard_state = False 
                else:
                    self.status_message = f"Takeoff Fail: {resp.message[:15]}..."
            except Exception as e:
                pass

        future.add_done_callback(response_callback)

    def land_drone_command(self):
        if not self.land_client.wait_for_service(timeout_sec=0.5):
            self.status_message = "ERR: Service Unavailable"
            return

        self.status_message = "Sending Land..."
        req = Land.Request()
        future = self.land_client.call_async(req)
        
        def response_callback(fut):
            try:
                resp = fut.result()
                if resp.success:
                    self.status_message = "Landing Mode: Auto..."
                    self.offboard_state = False
                    
                    self.reset_controls()
                else:
                    self.status_message = f"Land Fail: {resp.message[:15]}..."
            except:
                pass
                
        future.add_done_callback(response_callback)  

    def servo_command(self):
        if not self.arm_state:
            self.status_message = "ERR: Arm before dropping!"
            return
        
        if not self.servo_client.wait_for_service(timeout_sec=0.5):
            self.status_message = "ERR: Service Unavailable"
            return
        self.status_message = "Dropping payload..."
        req=ServoCommand.Request()
        req.angle = 1.57
        future=self.servo_client.call_async(req)
        def response_callback(fut):
            try:
                resp=fut.result()
                if resp.success:
                    self.status_message = "Payload DROPPED!"
                else:
                    self.status_message = f"Drop Fail: {resp.message}"
            except Exception as e:
                self.status_message = f"Service Error: {str(e)}"
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
        print()

if __name__ == '__main__':
    main()