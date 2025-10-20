import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from chapt4_interfaces.srv import FaceDetector
import cv2
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
import time
class FaceDetectionClient(Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        self.cv_bridge_ = CvBridge()
        self.iamge_path_ = get_package_share_directory('demo_python_service') + '/resource/default.jpg'
        self.image_ = cv2.imread(self.iamge_path_)
        self.get_logger().info(f'Face detection client will use image at {self.iamge_path_}')
        self.client_ = self.create_client(FaceDetector, 'face_detection_service')

    def send_request(self):
        while self.client_.wait_for_service(timeout_sec=1.0) == False:
            self.get_logger().info('service not available, waiting again...')
            return
        request = FaceDetector.Request()
        request.image = self.cv_bridge_.cv2_to_imgmsg(self.image_)
        future = self.client_.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        self.get_logger().info(f'Number of faces detected: {response.num}, time taken: {response.use_time} seconds')
        self.show_response(response)
    def show_response(self, response):
        for i in range(response.num):
            cv2.rectangle(self.image_, (response.left[i], response.top[i]), (response.right[i], response.bottom[i]), (0,255,0), 2)
        cv2.imshow("Face Detection Result", self.image_)
        cv2.waitKey(0)
def main():
    rclpy.init()
    node = FaceDetectionClient("face_detection_client")
    node.send_request()
    rclpy.spin(node)
    rclpy.shutdown()