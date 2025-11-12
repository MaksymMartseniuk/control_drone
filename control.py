import rclpy
import sys
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
import threading
from px4_msgs.msg import OffboardControlMode, VehicleRatesSetpoint, VehicleCommand

if sys.platform == 'win32':
    import msvcrt
else:
    import tty
    import termios



msg = """
Керування PX4 Offboard!
---------------------------
W/S: Throttle (Тяга вгору/вниз)
A/D: Yaw (Поворот вліво/вправо)
Стрілки: Pitch/Roll (Нахили вперед/назад, вліво/вправо)

Space: Arm / Disarm
Q: Перехід в режим Offboard (ПОТРІБНО ПІСЛЯ ARM)

CTRL-C: Вихід
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

class DroneTeleopNode(Node):
    def __init__(self):
        super().__init__('drone_teleop_node')

        qos_reliable = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        qos_best_effort = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.cmd_publisher_ = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_reliable)
            
        self.offboard_mode_publisher_ = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_reliable)
            
        self.rates_setpoint_publisher_ = self.create_publisher(
            VehicleRatesSetpoint, '/fmu/in/vehicle_rates_setpoint', qos_best_effort)

        self.arm_state = False
        self.offboard_state = False

        self.throttle_step = 0.1
       
        self.rate_step = 0.15
        self.throttle_target = 0.0
        self.target_roll_rate = 0.0
        self.target_pitch_rate = 0.0
        self.target_yaw_rate = 0.0

        self.timer_period = 0.01
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.get_logger().info(msg)

        self.key_listener_thread = threading.Thread(target=self.key_listener)
        self.key_listener_thread.daemon = True
        self.key_listener_thread.start()

    def key_listener(self):
        try:
            key_actions = {
                'w':lambda:setattr(self, 'throttle_target', min(1.0, self.throttle_target + self.throttle_step)),
                's':lambda:setattr(self, 'throttle_target', max(0.0, self.throttle_target - self.throttle_step)),
                'a':lambda:setattr(self, 'target_yaw_rate', min(2.0, self.target_yaw_rate + self.rate_step)),
                'd':lambda:setattr(self, 'target_yaw_rate', max(-2.0, self.target_yaw_rate - self.rate_step)),
                '\x1b[A':lambda:setattr(self, 'target_pitch_rate', max(-1.5, self.target_pitch_rate - self.rate_step)),  # arrow up
                '\x1b[B':lambda:setattr(self, 'target_pitch_rate', min(1.5, self.target_pitch_rate + self.rate_step)),  # arrow down
                '\x1b[C':lambda:setattr(self, 'target_roll_rate', min(1.5, self.target_roll_rate + self.rate_step)),  # arrow right
                '\x1b[D':lambda:setattr(self, 'target_roll_rate', max(-1.5, self.target_roll_rate - self.rate_step)),  # arrow left
                }
            while rclpy.ok():
                key = get_key()

                if key == '\x03':
                    self.get_logger().info("CTRL-C detected, shutting down...")
                    rclpy.shutdown()
                    break

                if key == ' ':
                    self.arm_state = not self.arm_state
                    self.public_arm_disarm(self.arm_state)
                    if not self.arm_state:
                        self.offboard_state = False
                    continue

                if key.lower() == 'q':
                    if self.arm_state:
                        self.offboard_state = True
                        self.get_logger().info("--- OFFBOARD MODE ENGAGED ---")
                    else:
                        self.get_logger().warn("Cannot enter OFFBOARD mode. Drone is not armed.")
                    continue

                if self.arm_state and self.offboard_state:
                    if key in key_actions:
                        action=key_actions.get(key.lower())
                        if action:
                            action()
                else:
                    self.reset_target()

                self.print_status_on_one_line()

        except Exception as e:
            if rclpy.ok():
                self.get_logger().error(f"Exception in key_listener: {e}")

    def reset_target(self):
        self.throttle_target = 0.0
        self.target_roll_rate = 0.0
        self.target_pitch_rate = 0.0
        self.target_yaw_rate = 0.0

    def timer_callback(self):
            if not rclpy.ok():
                return
            offboard_msg = OffboardControlMode()
            offboard_msg.position = False
            offboard_msg.velocity = False
            offboard_msg.acceleration = False
            offboard_msg.attitude = False
            offboard_msg.body_rate = True
            offboard_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
            self.offboard_mode_publisher_.publish(offboard_msg)
            if self.arm_state and self.offboard_state:
                rates_msg = VehicleRatesSetpoint()
                rates_msg.roll = self.target_roll_rate
                rates_msg.pitch = self.target_pitch_rate
                rates_msg.yaw = self.target_yaw_rate
                rates_msg.thrust_body[2] = -self.throttle_target 
                
                rates_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
                self.rates_setpoint_publisher_.publish(rates_msg)

    def print_status_on_one_line(self):
        status_msg = (
            f"ARM: {'YES' if self.arm_state else 'NO '} | "
            f"OFFB: {'YES' if self.offboard_state else 'NO '} | "
            f"Thr: {self.throttle_target: .2f} | "
            f"Pit: {self.target_pitch_rate: .1f} | "
            f"Rol: {self.target_roll_rate: .1f} | "
            f"Yaw: {self.target_yaw_rate: .1f}"
        )
        self.get_logger().info(status_msg)

    def public_arm_disarm(self, arm):
        msg = VehicleCommand()
        msg.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        msg.param1 = 1.0 if arm else 0.0
        msg.param2 = 0.0
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 255
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.cmd_publisher_.publish(msg)
        
        status_str = "ARMED" if arm else "DISARMED"
        self.get_logger().info(f"--- Drone {status_str} ---")


def main(args=None):
    old_settings = save_terminal_settings()
    
    rclpy.init(args=args)
    node = None
    try:
        node = DroneTeleopNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        if node:
            node.get_logger().error(f"Unhandled exception: {e}")
    finally:
        if node:
            node.get_logger().info("Shutting down node...")
            node.public_arm_disarm(False)
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()
    
        restore_terminal_settings(old_settings)
        print("\nTerminal settings restored. Exiting.")


if __name__ == '__main__':
    main()