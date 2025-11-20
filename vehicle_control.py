import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from rclpy.executors import MultiThreadedExecutor
from px4_msgs.msg import OffboardControlMode, VehicleCommand, VehicleRatesSetpoint, TrajectorySetpoint
from ros_px4_my.msg import RCControl 
from ros_px4_my.srv import TakeOff, Land
from sensor_msgs.msg import LaserScan

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
        
        # ---PUBLISHERS---
        self.cmd_publisher_ = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_reliable)
        
        self.offboard_mode_publisher_ = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_reliable)
        
        self.rates_setpoint_publisher_ = self.create_publisher(
            VehicleRatesSetpoint, '/fmu/in/vehicle_rates_setpoint', qos_best_effort) 
        
        self.position_setpoint_publisher_ = self.create_publisher(
            TrajectorySetpoint, "/fmu/in/trajectory_setpoint", qos_best_effort)
        # ---SUBSCROBERS---
        self.rc_command_subscriber = self.create_subscription(
            RCControl, 'rc_command', self.rc_command_callback, qos_reliable)

        self.lidar_subscriber = self.create_subscription(
            LaserScan,
            '/world/default/model/x_lidar_my_0/link/lidar_sensor_link/sensor/lidar/scan',
            self.lidar_callback,
            qos_best_effort
        )
        # ---SERVICE---
        self.takeoff_service = self.create_service(TakeOff, 'takeoff_command/drone', self.takeoff_callback)
        self.land_service = self.create_service(Land, 'land_command/drone', self.land_callback)
        # ---LOCALSTATE---
        self.arm_state = False
        self.offboard_state = False
        self.armed_state_sent = False 
        self.current_setpoint_height = None

        self.offboard_setpoint_counter = 0
        self.offboard_mode_sent = False

        self.min_lidar_distance = float('inf')

        self.current_rc_control = RCControl()

        self.timer = self.create_timer(0.02, self.timer_callback)
        
        self.get_logger().info("VehicleControlNode initialized.")

    
    def land_callback(self, request, response):
        self.get_logger().info("Landing Service Called.")
        
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        
        self.offboard_state = False 
        self.current_setpoint_height = None
        self.offboard_mode_sent = False
        self.offboard_setpoint_counter = 0
        
        self.arm_state = True 
        
        response.success = True
        response.message = "PX4 Land Mode triggered."
        self.get_logger().info(response.message)
        return response

    def takeoff_callback(self, request, response):
        if not self.arm_state:
            response.success = False
            response.message = "Rejected: Drone disarmed."
            self.get_logger().error(response.message)
            return response

        self.current_setpoint_height = request.target_height
        self.offboard_state = True
        
        response.success = True
        response.message = f"Takeoff initiated to {self.current_setpoint_height}m."
        self.get_logger().info(response.message)
        return response

    def rc_command_callback(self, msg: RCControl):
        self.current_rc_control = msg
        self.arm_state = msg.arm_state

        if not msg.offboard_state and self.offboard_state:
             self.current_setpoint_height = None
        
        self.offboard_state = msg.offboard_state

    def lidar_callback(self, msg: LaserScan):
        if msg.ranges:
            valid_ranges = [r for r in msg.ranges if r != float('inf') and r > msg.range_min]
            self.min_lidar_distance = min(valid_ranges) if valid_ranges else float('inf')
        else:
            self.min_lidar_distance = float('inf')

        if self.arm_state:
            if 0.1 < self.min_lidar_distance < 0.5:
                self.get_logger().warn(f"COLLISION WARNING: Object at {self.min_lidar_distance:.2f}m!")
   
   
    def timer_callback(self):
        if not rclpy.ok():
            return

        timestamp = int(self.get_clock().now().nanoseconds / 1000)

        if self.arm_state and not self.armed_state_sent:
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
            self.armed_state_sent = True
        elif not self.arm_state and self.armed_state_sent:
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
            self.armed_state_sent = False
           
            self.offboard_state = False
            self.offboard_setpoint_counter = 0
            self.offboard_mode_sent = False

        if self.offboard_state:
            
            rc_nonzero = (
                abs(self.current_rc_control.target_roll_rate) > 0.0 or
                abs(self.current_rc_control.target_pitch_rate) > 0.0 or
                abs(self.current_rc_control.target_yaw_rate) > 0.0 or
                abs(self.current_rc_control.throttle_target) > 0.0
                )
            
            offboard_msg = OffboardControlMode()
            offboard_msg.timestamp = timestamp
            offboard_msg.position = False
            offboard_msg.velocity = False
            offboard_msg.acceleration = False
            offboard_msg.attitude = False
            offboard_msg.body_rate = False

            if self.current_setpoint_height is not None and not rc_nonzero:
                offboard_msg.position = True
            
                traj_msg = TrajectorySetpoint()
                traj_msg.timestamp = timestamp
                traj_msg.position = [0.0, 0.0, -float(self.current_setpoint_height)]
                traj_msg.yaw = 0.0

                self.position_setpoint_publisher_.publish(traj_msg)

            else:
                self.current_setpoint_height = None
                offboard_msg.body_rate = True

                rates_msg = VehicleRatesSetpoint()
                rates_msg.timestamp = timestamp
                rates_msg.roll = float(self.current_rc_control.target_roll_rate)
                rates_msg.pitch = float(self.current_rc_control.target_pitch_rate)
                rates_msg.yaw = float(self.current_rc_control.target_yaw_rate)
                rates_msg.thrust_body = [0.0, 0.0, -float(self.current_rc_control.throttle_target)]

                self.rates_setpoint_publisher_.publish(rates_msg)

            self.offboard_mode_publisher_.publish(offboard_msg)

            if self.offboard_setpoint_counter < 50:
                self.offboard_setpoint_counter += 1

            if self.offboard_setpoint_counter >= 50 and not self.offboard_mode_sent:
                self.publish_vehicle_command(
                    VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 
                    param1=1.0,
                    param2=6.0
                )
                self.offboard_mode_sent = True

        else:
            self.offboard_setpoint_counter = 0
            self.offboard_mode_sent = False


    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = param1
        msg.param2 = param2
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