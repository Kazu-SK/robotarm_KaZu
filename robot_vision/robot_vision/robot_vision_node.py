
import rclpy
import cv2
from rclpy.node import Node
import numpy as np

from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class RobotVision(Node):
    def __init__(self):
        super().__init__('robot_vision_node')
        self.image_sub = self.create_subscription(Image, "image_raw",self.callback, 10)
        self.bridge = CvBridge()

    def callback(self,data):

        #mtx = []
        #dist = []

        mtx = np.array([[698.28642314, 0.0, 316.85536112],[0.0, 696.32195198, 233.13342883],[0.0, 0.0, 1.0]])
        dist = np.array([[-2.86983128e-01, -8.12594804e-01, -1.46773385e-03, -6.55372020e-04, 2.56431010e+00]])
        h = 480
        w = 640

        
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        #cv2.imshow("cv_image", cv_image)

        newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))
        dst_image = cv2.undistort(cv_image, mtx, dist, None, newcameramtx)
        cv2.imshow("cv_image", dst_image)

        cv2.waitKey(3)
        
        
def main():
    try:
        rclpy.init()
        robot_vision = RobotVision()
        rclpy.spin(robot_vision)
        robot_vision.destroy_node
        rclpy.shutdown()
    except KeyboardInterrupt:
        print("shutting Down")
        cv2.destroyAllWindows()


if __name__ == '__main__':
    
    main()

