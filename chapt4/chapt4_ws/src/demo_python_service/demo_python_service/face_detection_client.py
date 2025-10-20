import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from chapt4_interfaces.srv import FaceDetector
import cv2
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
import time
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
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
        # self.show_response(response)
    def show_response(self, response):
        for i in range(response.num):
            cv2.rectangle(self.image_, (response.left[i], response.top[i]), (response.right[i], response.bottom[i]), (0,255,0), 2)
        cv2.imshow("Face Detection Result", self.image_)
        cv2.waitKey(0)
    def update_param(self, model='hog', upsample_num = 1):
        param_model = Parameter()
        param_model.name = 'model'
        param_value = ParameterValue()
        param_value.type = ParameterType.PARAMETER_STRING
        param_value.string_value = model
        param_model.value = param_value
        param_sample_num = Parameter()
        param_sample_num.name = 'number_of_times_to_upsample'
        param_value = ParameterValue()
        param_value.type = ParameterType.PARAMETER_INTEGER
        param_value.integer_value = int(upsample_num)
        param_sample_num.value = param_value
        response = self.call_set_parameters([param_model, param_sample_num])
        for result in response.results:
            if result.successful:
                self.get_logger().info('Parameter updated successfully.')
            else:
                self.get_logger().info('Failed to update parameter.')
            self.get_logger().info(f'Reason: {result.reason}')

    def call_set_parameters(self, parameters):
        update_param_client = self.create_client(SetParameters, '/face_detection_node/set_parameters')
        while update_param_client.wait_for_service(timeout_sec = 1.0) == False:
            self.get_logger().info('parameter service not available, waiting again...')
        request = SetParameters.Request()
        request.parameters = parameters
        future = update_param_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        self.get_logger().info('Parameters updated successfully.')
        return response
def main():
    rclpy.init()
    node = FaceDetectionClient("face_detection_client")
    node.update_param(model='hog', upsample_num= 2)
    node.send_request()
    node.update_param(model='hog', upsample_num= 1)
    rclpy.spin(node)
    rclpy.shutdown()