import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_srvs.srv import Trigger
import os
import cv2
from ultralytics import YOLO
import time
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

class YoloDetectionNode(Node):
    def __init__(self):
        super().__init__("yolo_detection_node")
        qos_profile = QoSProfile(
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=1
        )
        self.model_path = YOLO("/home/maksym/code/runs/detect/train2/weights/best.pt")

        #---CAMERA BRIDGE--
        self.bridge = CvBridge()
        self.save_dir = 'dataset_images'
        self.latest_cv_image = None

        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)

        try:
            self.get_logger().info(f"Loading YOLO model from: {self.model_path}")
            self.model = YOLO(self.model_path)
            self.get_logger().info("YOLO model loaded successfully!")
        except Exception as e:
            self.get_logger().error(f"FAILED to load YOLO: {e}")
            self.model = None


        # ---SUBSCRIBERS---
        self.camera_subscriber = self.create_subscription(
            Image,
            'camera/image',
            self.image_callback,
            qos_profile
        )

        self.make_photo_service=self.create_service(Trigger,"camera/save_camera_photo",self.save_photo_callback)

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.latest_cv_image = cv_image 

            if self.model:
                results = self.model(cv_image, verbose=False)
                annotated_frame = results[0].plot()
                cv2.imshow("Camera Feed (YOLO)", annotated_frame)
            else:
                cv2.imshow("Camera Feed (Raw)", cv_image)

            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f"Error in image_callback: {e}")

    def save_photo_callback(self,request,response):
        if self.latest_cv_image is not None:
            timestamp = time.strftime("%Y%m%d-%H%M%S")
            fname = os.path.join(self.save_dir, f"drone_img_{timestamp}.jpg")
            cv2.imwrite(fname, self.latest_cv_image)
            
            response.success = True
            response.message = f"Photo saved as {fname}"
            self.get_logger().info(response.message)
        else:
            response.success = False
            response.message = "No image available to save"
        return response
    
def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
    

