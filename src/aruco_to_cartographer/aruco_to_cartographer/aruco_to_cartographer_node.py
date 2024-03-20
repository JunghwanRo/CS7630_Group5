import rclpy
from rclpy.node import Node

from aruco_opencv_msgs.msg import ArucoDetection
from cartographer_ros_msgs.msg import LandmarkList, LandmarkEntry


class ArucoToCarto(Node):

    def __init__(self):
        super().__init__('aruco_to_cartographer')
        self.sub = self.create_subscription(ArucoDetection,"/aruco_detections",self.callback,1)
        self.pub = self.create_publisher(LandmarkList,"/landmark",1)

    def callback(self,msg):
        L=LandmarkList()
        L.header = msg.header
        for m in msg.markers:
            if m.marker_id > 18:
                continue
            li = LandmarkEntry()
            li.id = str(m.marker_id)
            li.tracking_from_landmark_transform = m.pose
            li.translation_weight = 1.0
            li.rotation_weight = 0.1
            L.landmarks.append(li)
        self.pub.publish(L)




def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = ArucoToCarto()

    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
