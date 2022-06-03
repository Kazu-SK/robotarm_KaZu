
import rclpy
import cv2
from rclpy.node import Node
import numpy as np

from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

#from vision_service.srv import CoordinateConversion 
from vision_interfaces.msg import ImageCoordinate
#from vision_interfaces.msg import WorldCoordinate


class RobotVision(Node):
    def __init__(self):
        super().__init__('robot_vision_node')
        
        
        self.publisher_ = self.create_publisher(ImageCoordinate, 'image_coordinate_topic', 100)
        timer_period = 0.3
        self.timer = self.create_timer(timer_period, self.pub_callback)
        

        self.image_sub = self.create_subscription(Image, "image_raw",self.sub_callback, 10)

        self.bridge = CvBridge()

        self.image_u = -999 
        self.image_v = -999
    
    def pub_callback(self): 

        msg = ImageCoordinate()
        #self.get_logger().info('image_u = %d, image_v = %d' % self.image_u, self.image_v) 
        self.image_u = 10;
        self.image_u = -10;
        msg.image_u = int(self.image_u)
        msg.image_v = int(self.image_v)

        self.publisher_.publish(msg)

    def sub_callback(self,data):

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

        #hsv_image = cv2.cvtColor(dst_image, cv2.COLOR_BGR2HSV)

        color_min = np.array([0, 0, 50])
        color_max = np.array([35, 35, 255])

        color_mask = cv2.inRange(dst_image, color_min, color_max);

        cv_image2 = cv2.bitwise_and(dst_image, dst_image, mask = color_mask)

        gray_image = cv2.cvtColor(cv_image2, cv2.COLOR_BGR2GRAY)

        
       # contours, hierarchy = cv2.findContours(gray_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        #contours = list(filter(lambda x: cv2.contourArea(x) > 100, contours))
        # contours = list(filter(lambda x: cv2.contourArea(x) < 1000, contours))

        #print(contours)

        mu = cv2.moments(gray_image, True)
        if mu["m00"] == 0:
            self.image_u,self.image_v = -999, -999 
        else:
            self.image_u,self.image_v = int(mu["m10"]/mu["m00"]), int(mu["m01"]/mu["m00"])
        
        cv2.circle(dst_image, (self.image_u,self.image_v), 7, (0,255,0),5)
        cv2.imshow("cv_image", dst_image)
        #cv2.imshow("color_mask",cv_image2)

        cv2.waitKey(3)

        
        #self.send_request(x, y)


def main(args=None):


    '''
    rclpy.init(args=args)

    robot_vision = RobotVision()


    while rclpy.ok():
        rclpy.spin_once(robot_vision)

        try:
            response = self.future.result()
            self.get_logger().info('world_x = %f, world_y = %f, world_z = %f' % response.world_x, response.world_y, response.world_z) 
        except Exception as e:
            self.get_logger().info('Service call failed %4' % (e,))

        try:
        except KeyboardInterrupt:
            print("shutting Down")
    robot_vision.destroy_node()
    rclpy.shutdown()
    '''

    try:
        rclpy.init()
        robot_vision = RobotVision()
        rclpy.spin(robot_vision)
        robot_vision.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        print("shutting Down")
        cv2.destroyAllWindows()


if __name__ == '__main__':
    
    main()

