import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from chapt4_interfaces.srv import FaceDetector
import face_recognition
import cv2
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
import time
from rcl_interfaces.msg import SetParametersResult
class FaceDetectionNode(Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        self.service_ = self.create_service(FaceDetector, 'face_detection_service', self.face_detection_callback)
        self.cv_bridge_ = CvBridge()
        self.declare_parameter('number_of_times_to_upsample', 1)
        self.declare_parameter('model', 'hog')
        self.number_of_times_to_upsample_ = self.get_parameter('number_of_times_to_upsample').value
        self.model_ = self.get_parameter('model').value
        self.add_on_set_parameters_callback(self.parameters_callback)
        self.get_logger().info(f'Service {self.service_.srv_name} is ready to receive requests.')
        # set self node parameters
        self.set_parameters([rclpy.Parameter('number_of_times_to_upsample', rclpy.Parameter.Type.INTEGER, 3)])
    def parameters_callback(self, parameters):
        result = SetParametersResult(successful=True)
        for param in parameters:
            if param.name == 'number_of_times_to_upsample':
                self.number_of_times_to_upsample_ = param.value
                self.get_logger().info(f'Updated upsample number to {self.number_of_times_to_upsample_}')
            elif param.name == 'model':
                self.model_ = param.value
                self.get_logger().info(f'updated model to {self.model_}')
        return result
    def face_detection_callback(self, request, response):
        if request.image.data:
            cv_image = self.cv_bridge_.imgmsg_to_cv2(request.image)
        else:
            image_path = get_package_share_directory('demo_python_service') + '/resource/default.jpg'
            cv_image = cv2.imread(image_path)
            self.get_logger().info(f'Using default image at {image_path}')
        start_time = time.time()
        self.get_logger().info('start face detection')
        face_locations = face_recognition.face_locations(cv_image, number_of_times_to_upsample=self.number_of_times_to_upsample_, model=self.model_)
        response.use_time = time.time() - start_time
        self.get_logger().info(f'Face detection completed in {response.use_time} seconds')
        response.num = len(face_locations)
        for top, right, bottom, left in face_locations:
            response.top.append(top)
            response.right.append(right)
            response.bottom.append(bottom)
            response.left.append(left)
        return response
    


def main():
    rclpy.init()
    node = FaceDetectionNode("face_detection_node")
    rclpy.spin(node)
    rclpy.shutdown()