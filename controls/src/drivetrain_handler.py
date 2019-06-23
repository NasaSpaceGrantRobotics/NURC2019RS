#!/usr/bin/env python3
"""
                  -->
    |-------------[1]-------------|
    |------^---------------^------|
    |-----[5]-------------[4]-----|
    |-----------------------------|
 ^ [2]---------------------------[0] ^
    |------^---------------^------|
    |-----[6]-------------[7]-----|
    |-----------------------------|
    |-------------[3]-------------|
                  -->
"""
import rospy
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist


class DrivetrainHandler:

    def __init__(self):
        self.__e_stop = False
        self.drive_values_pub = rospy.Publisher('drive_values', Int32MultiArray, queue_size=10)
        self.__scaled_bounds = (1000, 2000)
        self.drive_values_msg = Int32MultiArray()
        self.drive_values_msg.data = [self.scale_normed_to_pwm(0.0) for i in range(0, 8)]
        self.inverted = [False for i in range(0, 8)]


    @staticmethod
    def __bound_number(number, bounds):
        if number < bounds[0]:
            number = bounds[0]
        elif number > bounds[1]:
            number = bounds[1]

        return number

    @staticmethod
    def __bound_with_max_scaling(numberlist):
        max_mag = max(list(map(lambda el: abs(el), numberlist)))
        new_list = numberlist
        if max_mag > 1.0:
            new_list = list(map(lambda el: el / max_mag, numberlist))
        return new_list

    def scale_normed_to_pwm(self, normed):
        minimum = self.__scaled_bounds[0]
        maximum = self.__scaled_bounds[1]

        return self.__bound_number(int(minimum + ((normed + 1.0) / 2.0) * (maximum - minimum)),
                                   [minimum, maximum])

    def drive_callback(self, msg):
        trans_normed = [msg.linear.x, msg.linear.y, msg.linear.z]
        rot_normed = [msg.angular.x, msg.angular.y, msg.angular.z]
        motor_normed = [0.0 for i in range(0, 8)]

        if self.__e_stop:
            trans_normed = [0.0, 0.0, 0.0]
            rot_normed = [0.0, 0.0, 0.0]

        motor_normed[0] = trans_normed[1]
        motor_normed[2] = trans_normed[1]
        motor_normed[1] = trans_normed[0]
        motor_normed[3] = trans_normed[0]
        for i in range(4, 8):
            motor_normed[i] = trans_normed[2]

        # applying yaw rotation
        motor_normed[0] += rot_normed[2]
        motor_normed[2] -= rot_normed[2]
        motor_normed[1] -= rot_normed[2]
        motor_normed[3] += rot_normed[2]

        # applying roll rotation
        motor_normed[4] -= rot_normed[1]
        motor_normed[7] -= rot_normed[1]
        motor_normed[5] += rot_normed[1]
        motor_normed[6] += rot_normed[1]

        # applying pitch rotation
        motor_normed[4] += rot_normed[0]
        motor_normed[5] += rot_normed[0]
        motor_normed[6] -= rot_normed[0]
        motor_normed[7] -= rot_normed[0]

        for motor_index in range(0, len(self.inverted)):
            if self.inverted[motor_index]:
                motor_normed[motor_index] = -motor_normed[motor_index]

        motor_normed = self.__bound_with_max_scaling(motor_normed)

        motor_pwm = self.scale_normed_to_pwm(motor_normed)

        self.drive_values_drive.data = motor_pwm

        self.drive_values_pub.publish(self.drive_values_msg)

    def e_stop_callback(self, msg):
        self.__e_stop = msg.data

    def start(self):
        rospy.Subscriber('target_drive', Twist, self.drive_callback)
        rospy.Subscriber('e_stop', Bool, self.e_stop_callback)

        rospy.init_node('drivetrain_handler')
        rospy.spin()


if __name__ == "__main__":
    try:
        drive_handler = DrivetrainHandler()
        drive_handler.start()
    except rospy.ROSInterruptException:
        pass

