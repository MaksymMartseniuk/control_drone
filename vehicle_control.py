import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from rclpy.executors import MultiThreadedExecutor
from px4_msgs.msg import OffboardControlMode, VehicleCommand, VehicleRatesSetpoint, TrajectorySetpoint
from ros_px4_my.msg import RCControl 
from ros_px4_my.srv import TakeOff
from sensor_msgs.msg import LaserScan
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

class VehicleControlNode(Node):
    def __init__(self):
        super().__init__('vehicle_control_node')
        
        qos_reliable = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )
        
        qos_best_effort = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            durability=QoSDurabilityPolicy.VOLATILE
        )
        
        # --- Publishers ---
        self.cmd_publisher_ = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_reliable)
        self.offboard_mode_publisher_ = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_reliable)
        
        self.rates_setpoint_publisher_ = self.create_publisher(
            VehicleRatesSetpoint, '/fmu/in/vehicle_rates_setpoint', qos_best_effort) 
        
        self.position_setpoint_publisher_ = self.create_publisher(
            TrajectorySetpoint, "/fmu/in/trajectory_setpoint", qos_best_effort)
        
        # --- Subscribers ---
        self.rc_command_subscriber = self.create_subscription(
            RCControl,
            'rc_command',
            self.rc_command_callback,
            qos_reliable
        )

        self.lidar_subscriber=self.create_subscription(
            LaserScan,
            '/world/default/model/x_lidar_my_0/link/lidar_sensor_link/sensor/lidar/scan',
            self.lidar_callback,
            qos_best_effort
        )
        # ---Server---
        self.service_group = MutuallyExclusiveCallbackGroup()
        self.takeoff_server = self.create_service(
            TakeOff, 
            'takeoff_service', 
            self.takeoff_callback,
            callback_group=self.service_group
        )

        # ---State---
        self.arm_state = False
        self.offboard_state = False
        self.armed_state_sent = False 
        
        self.current_rc_control = RCControl()

        self.is_position_control = False 
        self.takeoff_target_height = 0.0

        # ---Timer---    
        self.timer = self.create_timer(0.01, self.timer_callback)
        self.get_logger().info("VehicleControlNode initialized.")
        self.get_logger().info('Lidar subscriber started.')

    def rc_command_callback(self, msg: RCControl):
        self.current_rc_control = msg
        self.arm_state = msg.arm_state
        self.offboard_state = msg.offboard_state

    def takeoff_callback(self):
        pass

    def lidar_callback(self, msg: LaserScan):
        if msg.ranges:
            valid_ranges = [r for r in msg.ranges if r != float('inf') and r > msg.range_min]
            
            if valid_ranges:
                min_range = min(valid_ranges)
            else:
                min_range = float('inf')
        else:
            min_range = float('inf')
            
        self.get_logger().info(
            f"Recieve data from LaserScan: "
            f"Min distance {min_range:.2f}m. "
        )

        if min_range >0.2 and min_range < 0.5:
            self.get_logger().warn("COLLISION WARNING: Object detected closer than 0.5m!")
   
    def timer_callback(self):
        if not rclpy.ok():
            return
            
        timestamp = int(self.get_clock().now().nanoseconds / 1000)

        if self.offboard_state:
            offboard_msg = OffboardControlMode()
            offboard_msg.position = False
            offboard_msg.velocity = False
            offboard_msg.acceleration = False
            offboard_msg.attitude = False
            offboard_msg.body_rate = True 
            offboard_msg.timestamp = timestamp
            self.offboard_mode_publisher_.publish(offboard_msg)
            
            if self.arm_state:
                 self.public_set_offboard_mode()
        
        if self.arm_state and not self.armed_state_sent:
            self.public_arm_disarm(True)
            self.armed_state_sent = True
        elif not self.arm_state and self.armed_state_sent:
            self.public_arm_disarm(False)
            self.armed_state_sent = False

        if self.arm_state and self.offboard_state:
            rates_msg = VehicleRatesSetpoint()

            rates_msg.roll = self.current_rc_control.target_roll_rate
            rates_msg.pitch = self.current_rc_control.target_pitch_rate
            rates_msg.yaw = self.current_rc_control.target_yaw_rate
            rates_msg.thrust_body[2] = -self.current_rc_control.throttle_target
            
            rates_msg.timestamp = timestamp
            self.rates_setpoint_publisher_.publish(rates_msg)

    def public_arm_disarm(self, arm: bool):
        msg = VehicleCommand()
        msg.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        msg.param1 = 1.0 if arm else 0.0
        msg.param2 = 2.0
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 255
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.cmd_publisher_.publish(msg)
        
        status_str = "ARMED" if arm else "DISARMED"
        self.get_logger().info(f"--- Drone {status_str} ---")

    def public_set_offboard_mode(self):
        msg = VehicleCommand()
        msg.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
        msg.param1 = 1.0  
        msg.param2 = 6.0
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 255
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.cmd_publisher_.publish(msg)
    
        

def main(args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor() 
    node = VehicleControlNode()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt, shutting down.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()