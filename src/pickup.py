#!/usr/bin/env python3

"""
The Pickup Node picks up the ball at its current position and then places it down at the angle of the 
specified apriltag. 
"""

import rospy
import tf
import math
from geometry_msgs.msg import Point
from std_msgs.msg import Bool, String, Float32


class Pickup():

    def __init__(self, apriltag_num, box: bool):
        rospy.loginfo("Initializing pickup node....")
        self.listener = tf.TransformListener()
        self.apriltag_num = apriltag_num
        self.boxes = box # if true, then it is initialized from the boxes node and 

        # Publishers
        self.point_publisher = rospy.Publisher("/arm_control/point", Point, queue_size=1)
        self.home_publisher = rospy.Publisher("/arm_control/home", Bool, queue_size=1)
        self.sleep_publisher = rospy.Publisher("/arm_control/sleep", Bool, queue_size=1)
        self.gripper_publisher = rospy.Publisher("/arm_control/gripper", String, queue_size=1)
        self.time_publisher = rospy.Publisher("/arm_control/time", Float32, queue_size=1)

        self.open = "open"
        self.close = "close"

        # wait for 3 sec to get connections properly set up
        rospy.sleep(3)
    
    def pickup_ball(self):
        # set the arm to the sleep position to start off 
        rospy.loginfo("sending the arm to sleep position")
        self.sleep_publisher.publish(True)
        # wait for the arm to execute the command
        rospy.sleep(2)

        # open the gripper and wait to execute
        rospy.loginfo("opening the gripper")
        self.gripper_publisher.publish(self.open)
        rospy.sleep(2)

        rospy.loginfo("changing trajectory time to {}".format(2.0))
        self.time_publisher.publish(Float32(data=2.0))
        rospy.sleep(2)

        try:
            ball_gripper_point, ball_gripper_rot = self.listener.lookupTransform('/px100/right_finger_link' ,'/ball', rospy.Time())
            ball_world_point, ball_world_rot = self.listener.lookupTransform('/world' ,'/ball', rospy.Time())
            print("transform between gripper link and ball is {}".format(ball_gripper_point))
            print("transform between world and ball is {}".format(ball_world_point))
            point = Point()
            point.x = ball_world_point[0]
            point.y = ball_world_point[1]
            point.z = ball_gripper_point[2]

            ball_angle = math.atan2(point.y, point.x)

            # # move the waist first and wait
            rospy.loginfo("Moving waist first by {}".format(ball_angle))
            self.point_publisher.publish(Point(0, ball_angle, 0))
            rospy.sleep(2)

            point.x = abs(ball_gripper_point[0])

            # execute the pickup motion 
            rospy.loginfo("starting the pickup motion by {}".format(point.x))
            if point.x < 0.25 and point.x > 0.1: 
                self.point_publisher.publish(Point(point.x, ball_angle, -0.05))
                rospy.sleep(2)
                self.gripper_publisher.publish(self.close)
                rospy.sleep(2)

                # return to sleep position with the ball and wait
                self.sleep_publisher.publish(True)
                rospy.sleep(3)

                picked_up = self.check_if_picked_up()

                if not self.boxes and picked_up: 
                    april_point, april_rot = self.listener.lookupTransform('/world' ,'/at{}'.format(self.apriltag_num), rospy.Time())
                    print("transform between world and at{} is {}".format(self.apriltag_num, april_point))

                    april_angle = math.atan2(april_point[1], april_point[0]) + 0.05

                    # place ball at the correct angle
                    # move the waist first and wait
                    rospy.loginfo("Moving waist first by {}".format(april_angle))
                    self.point_publisher.publish(Point(0, april_angle, 0))
                    rospy.sleep(2)

                    x = 0.15
                    z = -0.05
                    # place the ball down at a certain distance
                    rospy.loginfo("Placing ball down at {} from arm".format(x))
                    self.point_publisher.publish(Point(x, april_angle, z))
                    rospy.sleep(3)

                    self.gripper_publisher.publish(self.open)
                    rospy.sleep(3)

                    # move back to release
                    self.point_publisher.publish(Point(-0.05, april_angle, 0))
                    rospy.sleep(3)

                    self.sleep_publisher.publish(True)
                    rospy.sleep(2)
                    return True
                else: 
                    rospy.loginfo("Ball is too close or too far from the arm to pickup")
                    self.sleep_publisher.publish(True)
                    rospy.sleep(2)

                    return False

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return
    
    def check_if_picked_up(self):
        try: 
            ball_gripper_point, ball_gripper_rot = self.listener.lookupTransform('/px100/right_finger_link' ,'/ball', rospy.Time())
            rospy.loginfo("Checking if the ball has been picked up! {}, {}".format(ball_gripper_point[0], ball_gripper_point[1]))
            if abs(ball_gripper_point[0]) < 0.15 and abs(ball_gripper_point[1]) < 0.15: 
                return True
            else: 
                print("The ball has not been picked up, please put it within my reach and try again.")
                return False
        except: 
            return False 
    
if __name__ =='__main__':
    rospy.init_node('pickup', anonymous=True)
#     try:
#         Pickup()
#     except rospy.ROSInterruptException:
#         pass
    rospy.spin()
