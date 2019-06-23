#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist


class ClawHandler:

    def __init__(self):
        self.__e_stop = False
        self.claw_values_pub = rospy.Publisher('claw_values', Int32MultiArray, queue_size=10)
        self.__trans_scaled_bounds = (1000, 2000)
        self.__rot_scaled_bounds = (1000, 2000)
        self.claw_values_msg = Int32MultiArray()
        self.claw_values_msg.data = [self.scale_trans_normed_to_pwm(0.0) for i in range(0, 2)]

    @staticmethod
    def __bound_number(number, bounds):
        if number < bounds[0]:
            number = bounds[0]
        elif number > bounds[1]:
            number = bounds[1]

        return number

    def scale_trans_normed_to_pwm(self, trans_normed):
        minimum = self.__trans_scaled_bounds[0]
        maximum = self.__trans_scaled_bounds[1]

        return self.__bound_number(int(minimum + ((trans_normed + 1.0) / 2.0) * (maximum - minimum)),
                                   [minimum, maximum])

    def scale_rot_normed_to_pwm(self, rot_normed):
        minimum = self.__rot_scaled_bounds[0]
        maximum = self.__rot_scaled_bounds[1]

        return self.__bound_number(int(minimum + ((rot_normed + 1.0) / 2.0) * (maximum - minimum)),
                                   [minimum, maximum])

    def claw_callback(self, msg):
        claw_trans_normed = msg.linear.x
        claw_rot_normed = msg.angular.y

        if self.__e_stop:
            claw_trans_normed = 0.0
            claw_rot_normed = 0.0

        claw_trans_scaled = self.scale_trans_normed_to_pwm(claw_trans_normed)
        claw_rot_scaled = self.scale_rot_normed_to_pwm(claw_rot_normed)

        self.claw_values_msg.data = [self.scale_trans_normed_to_pwm(claw_trans_scaled),
                                     self.scale_rot_normed_to_pwm(claw_rot_scaled)]

        self.claw_values_pub.publish(self.claw_values_msg)

    def e_stop_callback(self, msg):
        self.__e_stop = msg.data

    def start(self):
        rospy.Subscriber('target_claw', Twist, self.claw_callback)
        rospy.Subscriber('e_stop', Bool, self.e_stop_callback)

        rospy.init_node('claw_handler')
        rospy.spin()


if __name__ == "__main__":
    try:
        claw_handler = ClawHandler()
        claw_handler.start()
    except rospy.ROSInterruptException:
        pass

