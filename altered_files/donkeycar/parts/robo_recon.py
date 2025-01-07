import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from tensorflow.python.ops.metrics_impl import true_positives

### SELF-MAINTAINED
# Create DonkeyCar part to receive all necessary ROS data.
# This part has to be run in a separate thread because of it being a rospy node.
# To archive this, DonkeyCar's built in possibility to create 'threaded' parts can be utilized.

class RosReceiver:
    def __init__(self):
        self.depth_subscriber = None
        self.rgb_subscriber = None
        self.node = None

        self.bridge = CvBridge()

        self.depth_image = None
        self.rgb_image = None

    # Will be executed on part initialize.
    # ROS node is started here.
    # Additionally, ROS subscribers are defined and created.
    def update(self):
        self.depth_subscriber = rospy.Subscriber('/camera/depth/image_rect_raw', Image, self.depth_frame_received)
        self.rgb_subscriber = rospy.Subscriber('/camera/color/image_raw', Image, self.rgb_frame_received)

        self.node = rospy.init_node('donkey_receiver', anonymous=True, disable_signals=True)

    # Unused because of this being a 'threaded' part.
    # See vehicle.py line update_parts().
    def run(self):
        return self.rgb_image, self.depth_image

    # Returns transformed ROS data to DonkeyCar instance on update_parts() in vehicle.py.
    def run_threaded(self):
        return self.rgb_image, self.depth_image

    # Will be executed on part shutdown.
    # ROS node is shutdown here.
    # Would happen on program exit anyway, just want to make sure.
    def shutdown(self):
        rospy.signal_shutdown('donkey shutdown')
        pass

    # Transforms of sensor_msgs.msg.Image to further usable image/array.
    def depth_frame_received(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg)

    # Transforms of sensor_msgs.msg.Image to further usable image/array.
    def rgb_frame_received(self, msg):
        self.rgb_image = self.bridge.imgmsg_to_cv2(msg)