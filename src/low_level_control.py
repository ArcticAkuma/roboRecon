#!/usr/bin/python

import rospy
from i2cpwm_board.msg import Servo, ServoArray
from geometry_msgs.msg import Twist
import time

# SERVO VALUES
#
# STEERING IDLE	350
# THROTTLE IDLE	320 - 340
# 
# MAX LEFT	390
# MAX RIGHT	310
# MAX FORWARDS	360 (350)
# MAX BACKWARDS	300 (310)

IDLE_TIMEOUT = 5


# Object to store servo data.
class ServoConvert:
    """
    Object to store servo data.
    """

    def __init__(self, id=1, center_value=333, range=90, direction=1, max_value=1.0):
        self.value = 0.0
        self.value_out = center_value
        self._center = center_value
        self._range = range
        self._half_range = 0.5 * range
        self._max_value = max_value
        self._dir = direction
        self.id = id

    def get_value_out(self, value_in):
        """
        Calculate steering signal for servo based on received value.
        Value is generated using servo idle signal and adding/subtracting normalized target value multiplied by range.
        """
        self.value = value_in / self._max_value
        self.value_out = int(self._dir * self.value * self._half_range + self._center)
        return self.value_out


class ServoController:
    def __init__(self):
        """
        Initialize {$ServoConvert} object for every servo.
        Configure ROS node: Register publisher and subscriber node.
        Initialize last controller input as current time to fore one idle update.
        """

        rospy.init_node('i2c_controller')
        # todo: cfg -> pin_throttle, pin_steering
        max_throttle = rospy.get_param('~max_throttle', 1)
        max_steering = rospy.get_param('~max_steering', 1)
        rospy.loginfo("Setting Up the Node...")

        self.actuators = {'throttle': ServoConvert(id=8, center_value=330, range=60, max_value=max_throttle),
                          'steering': ServoConvert(id=9, center_value=355, range=80, max_value=max_steering)}
        # todo: load from ROS parameter (this are PS5 settings)
        self._servo_msg = ServoArray()
        for i in range(2):
            self._servo_msg.servos.append(Servo())

        self.ros_pub_servo_array = rospy.Publisher("/servos_absolute", ServoArray, queue_size=1)
        self.ros_sub_twist = rospy.Subscriber("/cmd_vel", Twist, self.set_actuators_from_cmd_vel)

        self._last_time_cmd_rcv = time.time()

        rospy.loginfo("Initialization complete")

    def set_actuators_from_cmd_vel(self, message):
        """
        Called when geometry_msgs/Twist message is received on /cmd_vel topic
        Time of last controller input received is updated here.
        Generate steering information, to enable requested steering.
        """

        self._last_time_cmd_rcv = time.time()

        self.actuators['throttle'].get_value_out(message.linear.x)
        self.actuators['steering'].get_value_out(message.angular.z)
        self.send_servo_msg()

    def set_actuators_idle(self):
        """
        Generate steering information, to set both servos to idle.
        """
        self.actuators['throttle'].get_value_out(0)
        self.actuators['steering'].get_value_out(0)
        self.send_servo_msg()

    def send_servo_msg(self):
        """
        Sends converted steering information to servos.
        geometry_msgs/Twist converted data will be sent on topic /servos_absolute.
        """

        for actuator_name, servo_obj in self.actuators.items():
            self._servo_msg.servos[servo_obj.id - 8].servo = servo_obj.id
            self._servo_msg.servos[servo_obj.id - 8].value = servo_obj.value_out

        self.ros_pub_servo_array.publish(self._servo_msg)

    @property
    def is_controller_connected(self):
        """
        Checks if controller is still available/online.
        Returns true as long as a signal was received in the last {$IDLE_TIMEOUT} seconds.
        """
        return time.time() - self._last_time_cmd_rcv < IDLE_TIMEOUT

    def run(self):
        """
        Starts ROS node functionality and adds update rate.
        """
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if not self.is_controller_connected:
                """
                Reset of output steering signal to servo to idle value if controller is offline.
                This prevents runaway situations in case no further steering data is received.
                """
                self.set_actuators_idle()

            rate.sleep()


if __name__ == "__main__":
    controller = ServoController()
    controller.run()
