import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from rclpy.executors import MultiThreadedExecutor
from px4_msgs.msg import OffboardControlMode, VehicleCommand, VehicleRatesSetpoint, VehicleGlobalPosition, VehicleStatus,VehicleLandDetected
from ros_px4_my.msg import RCControl 
from ros_px4_my.srv import TakeOff, Land, ServoCommand
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64
import threading


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
        
        self.servo_publisher=self.create_publisher(
            Float64,
            '/servo_cmd',
            qos_reliable
        )
        self.mount_rad_publisher = self.create_publisher(
            Float64, 
            '/camera_mount_cmd', 
            qos_reliable
        )
        self.pole_rad_publisher = self.create_publisher(
            Float64,
            '/camera_pole_cmd',
            qos_reliable
        )
        
        # ---SUBSCRIBERS---
        self.rc_command_subscriber = self.create_subscription(
            RCControl, 'rc_command', self.rc_command_callback, qos_reliable)
        
        self.global_position_subscriber = self.create_subscription(
            VehicleGlobalPosition,
            'fmu/out/vehicle_global_position',
            self.global_position_callback,
            qos_best_effort
        )
        
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status',
            self.vehicle_status_callback,
            qos_best_effort
        )

        self.lidar_subscriber = self.create_subscription(
            LaserScan,
            '/lidar_scan',
            self.lidar_callback,
            qos_best_effort
        )

        self.vehicle_land_detector=self.create_subscription(
            VehicleLandDetected,
            'fmu/out/vehicle_land_detected',
            self.land_detected_callback,
            qos_best_effort
        )
        
        # ---SERVICE---
        self.takeoff_service = self.create_service(TakeOff, 'takeoff_command/drone', self.takeoff_callback)
        self.land_service = self.create_service(Land, 'land_command/drone', self.land_callback)
        self.servo_service=self.create_service(ServoCommand,'servo_command/drone',self.servo_callback)
        
        # ---LOCALSTATE---
        self.armed_state_sent = False 
        self.current_setpoint_height = None

        self.is_taking_off = False
        self.is_in_air = False
        self.takeoff_start_alt = 0.0

        self.offboard_setpoint_counter = 0
        self.min_lidar_distance = float('inf')
        self.current_global_alt = 0.0
        self.offboard_mode_allowed = True 
        
        self.current_nav_state = 0 

        self.current_servo_angle = 0.0

        self.current_rc_control = RCControl()

        self.timer = self.create_timer(0.02, self.timer_callback)
        
        self.get_logger().info("VehicleControlNode initialized.")

    def land_detected_callback(self,msg:VehicleLandDetected):
        self.is_in_air = not msg.landed

    def rc_command_callback(self,msg:RCControl):
        self.current_rc_control=msg

    def servo_callback(self,request,response):
        safe_angle = max(-1.57, min(1.57, request.angle))
        self.current_servo_angle=safe_angle
        self.current_servo_angle = safe_angle
        self.get_logger().info(f"Servo OPENING to {safe_angle:.2f} rad")

        def return_servo_back():
            self.current_servo_angle = 0.0
            self.get_logger().info("Servo CLOSING (Auto-reset)")
        
        timer = threading.Timer(1.0, return_servo_back)
        timer.start()

        response.success = True
        response.message = "Drop initiated (Auto-reset scheduled)"
        return response


    def global_position_callback(self,msg:VehicleGlobalPosition):
        self.current_global_alt = msg.alt

    def vehicle_status_callback(self, msg: VehicleStatus):
        self.current_nav_state = msg.nav_state

    def lidar_callback(self,msg:LaserScan):
        if msg.ranges:
            valid_ranges = [r for r in msg.ranges if r > msg.range_min and r != float('inf')]
            if valid_ranges:
                self.min_lidar_distance = min(valid_ranges)
            else:
                self.min_lidar_distance = float('inf')

        if self.current_rc_control.arm_state:
            if 0.1 < self.min_lidar_distance < 0.5:
                self.get_logger().warn(f"COLLISION WARNING: Object at {self.min_lidar_distance:.2f}m!", throttle_duration_sec=1.0)

    def takeoff_callback(self,request, response):
        if not self.current_rc_control.arm_state:
            response.success = False
            response.message = "Rejected: Drone disarmed."
            self.get_logger().error(response.message)
            return response
        
        if self.is_taking_off:
            response.success = False
            response.message = "Rejected: Takeoff already in progress."
            self.get_logger().warn(response.message)
            return response
        
        if self.is_in_air:
            response.success = False
            response.message = "Rejected: Drone is already in air. Land and Disarm first."
            self.get_logger().warn(response.message)
            return response
        

        self.current_setpoint_height = request.target_height
        target_amsl_alt = self.current_global_alt + request.target_height

        self.takeoff_start_alt = self.current_global_alt
        self.is_taking_off = True
        
        self.offboard_mode_allowed = False 
        self.offboard_setpoint_counter = 0

        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF,
            param5=float('nan'),
            param6=float('nan'),
            param7=target_amsl_alt
        )

        response.success = True
        response.message = f"Takeoff initiated to {self.current_setpoint_height}m."
        self.get_logger().info(response.message)
        return response

    def land_callback(self, request, response):
        if not self.current_rc_control.arm_state:
            response.success = False
            response.message = "Rejected: Drone disarmed."
            self.get_logger().error(response.message)
            return response
    
        if not self.is_in_air:
            response.success = False
            response.message = "Rejected: Drone is already on the ground."
            self.get_logger().warn(response.message)
            return response
        
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.offboard_mode_allowed = False
        self.is_taking_off = False

        response.success = True
        response.message = "Landing sequence initiated."
        self.get_logger().info(response.message)
        return response
         

    def timer_callback(self):
        if not rclpy.ok():
            return
        try:
            mount_msg = Float64()
            mount_msg.data = self.current_rc_control.target_mount_rad
            self.mount_rad_publisher.publish(mount_msg)
        except Exception as e:
            self.get_logger().error(f"Error publishing mount command: {e}")

        try:
            pole_msg = Float64()
            pole_msg.data = self.current_rc_control.target_pole_rad
            self.pole_rad_publisher.publish(pole_msg)
        except Exception as e:
            self.get_logger().error(f"Error publishing pole command: {e}")

            
        servo_msg = Float64()
        servo_msg.data = self.current_servo_angle
        self.servo_publisher.publish(servo_msg)

        timestamp = int(self.get_clock().now().nanoseconds / 1000)
        
        if self.current_rc_control.arm_state and not self.armed_state_sent:
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
            self.armed_state_sent = True
            self.get_logger().info("Arm command sent")

        elif not self.current_rc_control.arm_state and self.armed_state_sent:
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
            
            self.publish_vehicle_command(
                VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 
                param1=1.0,
                param2=4.0,
                param3=3.0
            )
            
            self.armed_state_sent = False
            self.is_taking_off = False
            self.offboard_mode_allowed = True
            self.offboard_setpoint_counter = 0
            self.current_servo_angle = 0.0
            self.get_logger().info("Disarm command sent -> Reset to HOLD")

        if self.is_taking_off:
            current_agl = self.min_lidar_distance
            if current_agl != float('inf') and current_agl >= (self.current_setpoint_height * 0.95):
                self.is_taking_off = False
                self.get_logger().info(f"LIDAR Target Reached ({current_agl:.2f}m). Waiting for RC Offboard switch reset.")
            return

        if not self.current_rc_control.offboard_state:
            if not self.offboard_mode_allowed:
                self.get_logger().info("Offboard switch detected OFF. System UNLOCKED for Offboard.", throttle_duration_sec=5.0)
            self.offboard_mode_allowed = True
            self.offboard_setpoint_counter = 0
        
        if self.current_rc_control.offboard_state and self.offboard_mode_allowed:

            offboard_msg = OffboardControlMode()
            offboard_msg.timestamp = timestamp
            offboard_msg.position = False
            offboard_msg.velocity = False
            offboard_msg.acceleration = False
            offboard_msg.attitude = False
            offboard_msg.body_rate = True
            self.offboard_mode_publisher_.publish(offboard_msg)

            rates_msg = VehicleRatesSetpoint()
            rates_msg.timestamp = timestamp
            rates_msg.roll = float(self.current_rc_control.target_roll_rate)
            rates_msg.pitch = float(self.current_rc_control.target_pitch_rate)
            rates_msg.yaw = float(self.current_rc_control.target_yaw_rate)
            rates_msg.thrust_body = [0.0, 0.0, -float(self.current_rc_control.throttle_target)]
            self.rates_setpoint_publisher_.publish(rates_msg)

            self.offboard_setpoint_counter += 1
            
            if self.offboard_setpoint_counter > 20:
                
                if self.current_nav_state != 14:
                    if self.offboard_setpoint_counter % 5 == 0:
                        self.publish_vehicle_command(
                            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 
                            param1=1.0,
                            param2=6.0 
                        )
                        self.get_logger().info(f"Attempting to switch OFFBOARD... (NavState: {self.current_nav_state})")
                else:
                    pass

        elif self.current_rc_control.offboard_state and not self.offboard_mode_allowed:
             self.get_logger().warn("Safety Lock: Please toggle Offboard Switch to OFF first!", throttle_duration_sec=2.0)
             self.offboard_setpoint_counter = 0
        
        
        
    def publish_vehicle_command(self,command,param1=0.0,param2=0.0,param3=0.0,param4=0.0,param5=0.0,param6=0.0,param7=0.0):
        vh_command=VehicleCommand()
        vh_command.command=command
        vh_command.param1 = param1
        vh_command.param2 = param2
        vh_command.param3 = param3
        vh_command.param4 = param4
        vh_command.param5 = param5
        vh_command.param6 = param6
        vh_command.param7 = param7
        vh_command.target_system = 1
        vh_command.target_component = 1
        vh_command.source_system = 255
        vh_command.source_component = 1
        vh_command.from_external = True
        vh_command.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.cmd_publisher_.publish(vh_command)

   

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