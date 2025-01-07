import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from tensorflow.python.ops.metrics_impl import true_positives

### SELF-MAINTAINED

class RosReceiver:
    def __init__(self):
        self.depth_subscriber = None
        self.rgb_subscriber = None
        self.node = None

        self.bridge = CvBridge()

        # todo: check if dummy defaults are needed
        self.depth_image = None
        self.rgb_image = None

    def update(self):
        self.depth_subscriber = rospy.Subscriber('/camera/depth/image_rect_raw', Image, self.depth_frame_received)
        self.rgb_subscriber = rospy.Subscriber('/camera/color/image_raw', Image, self.rgb_frame_received)

        self.node = rospy.init_node('donkey_receiver', anonymous=True, disable_signals=True)

    def run(self):
        return self.rgb_image, self.depth_image

    def run_threaded(self):
        return self.rgb_image, self.depth_image

    def shutdown(self):
        rospy.signal_shutdown('donkey shutdown')
        pass

    def depth_frame_received(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg)

    def rgb_frame_received(self, msg):
        self.rgb_image = self.bridge.imgmsg_to_cv2(msg)