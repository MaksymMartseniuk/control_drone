#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleCommand, OffboardControlMode, TrajectorySetpoint, VehicleStatus

class OffboardTakeoff(Node):
    def __init__(self):
        super().__init__('px4_offboard_takeoff')

        # --- QoS налаштування (ОБОВ'ЯЗКОВО BEST_EFFORT) ---
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Паблішери
        self.offboard_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos)
        self.traj_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos)
        self.cmd_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos)
        self.status_sub = self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status', self.status_cb, qos)

        # Змінні стану
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.arming_state = VehicleStatus.ARMING_STATE_DISARMED
        self.counter = 0

        # Таймер 10 Гц (0.1 сек) - це серцебиття програми
        self.timer = self.create_timer(0.1, self.timer_callback)

    def status_cb(self, msg):
        self.nav_state = msg.nav_state
        self.arming_state = msg.arming_state

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = param1
        msg.param2 = param2
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.cmd_pub.publish(msg)

    def timer_callback(self):
        # 1. ПОСТІЙНО публікуємо OffboardControlMode (Heartbeat)
        # Якщо цей потік перерветься на 0.5 сек -> Failsafe
        off_msg = OffboardControlMode()
        off_msg.position = True
        off_msg.velocity = False
        off_msg.acceleration = False
        off_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_pub.publish(off_msg)

        # 2. ПОСТІЙНО публікуємо Setpoint (куди летіти)
        # NED координати: Z = -5.0 метрів (це 5 метрів ВГОРУ)
        traj_msg = TrajectorySetpoint()
        traj_msg.position = [0.0, 0.0, -5.0] 
        traj_msg.yaw = 0.0
        traj_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.traj_pub.publish(traj_msg)

        # 3. Логіка перемикання режимів (тільки якщо дрон ще не готовий)
        self.counter += 1

        # Чекаємо 10 циклів (1 сек), щоб PX4 побачив стабільний потік даних
        if self.counter == 15:
            self.get_logger().info("Перемикання в режим Offboard...")
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)

        # Ще через трохи часу - Армінг
        if self.counter == 25 and self.arming_state != VehicleStatus.ARMING_STATE_ARMED:
            self.get_logger().info("Армінг...")
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)

        # Лог для перевірки
        if self.counter % 20 == 0:
            self.get_logger().info(f"Status: NavState={self.nav_state}, ArmState={self.arming_state}")

def main(args=None):
    rclpy.init(args=args)
    node = OffboardTakeoff()
    try:
        rclpy.spin(node) # Цей рядок не дає скрипту померти
    except KeyboardInterrupt:
        pass
    finally:
        # При виході бажано відправити команду на посадку
        node.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()