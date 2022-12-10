#!/usr/bin/env python3

"""
The Push Node turns to the ball at a certain angle and pushes it forward
It looks for the ball transform and then publishes commands to the arm_controller. 
"""

import rospy 
import tf
import math
from geometry_msgs.msg import Point
from std_msgs.msg import Bool, String, Float32

class Push():
    def __init__(self, apriltag_num):
        rospy.loginfo("Initializing push node....")
        self.listener = tf.TransformListener()

        # Publishers
        self.point_publisher = rospy.Publisher("/arm_control/point", Point, queue_size=1)
        self.home_publisher = rospy.Publisher("/arm_control/home", Bool, queue_size=1)
        self.sleep_publisher = rospy.Publisher("/arm_control/sleep", Bool, queue_size=1)
        self.gripper_publisher = rospy.Publisher("/arm_control/gripper", String, queue_size=1)
        self.time_publisher = rospy.Publisher("/arm_control/time", Float32, queue_size=1)

        self.open = "open"
        self.close = "close"

        self.apriltag_num = apriltag_num

        # wait for 3 sec to get connections properly set up
        rospy.sleep(3)
        self.push_ball()
    
    def push_ball(self):
        # set the arm to the sleep position to start off 
        rospy.loginfo("sending the arm to sleep position")
        self.sleep_publisher.publish(True)
        # wait for the arm to execute the command
        rospy.sleep(2)
        
        # close the gripper and wait to execute
        self.gripper_publisher.publish(self.close)
        rospy.sleep(2)

        sec = 0.9
        rospy.loginfo("changing trajectory time to {}".format(sec))
        self.time_publisher.publish(Float32(data=sec))
        rospy.sleep(3)

        try:
            # get the ball transform using tf listener
            (trans,rot) = self.listener.lookupTransform('/px100/right_finger_link', '/ball', rospy.Time())
            print("transform between ball and gripper", trans)
            point = Point()
            point.x = trans[0]
            point.y = trans[1]
            point.z = trans[2]

            angle = math.atan(point.y / point.x)

            april_point, april_rot = self.listener.lookupTransform('/world' ,'/at{}'.format(self.apriltag_num), rospy.Time())
            print("transform between world and at{} is {}".format(self.apriltag_num, april_point))

            april_angle = math.atan2(april_point[1], april_point[0]) + 0.05

            # move the waist first 
            rospy.loginfo("Moving waist first by {}".format(april_angle))
            self.point_publisher.publish(Point(0, april_angle, 0))
            # wait for the command to execute
            rospy.sleep(2)

            # execute the pushing motion 
            # if point.x < -0.1: 
            rospy.loginfo("starting to push the ball")
            self.point_publisher.publish(Point(0.15, april_angle, -0.04))
            # else: 
            #     rospy.loginfo("Ball is too far away from arm to push")
            
            # wait for command to execute and return to sleep position
            rospy.sleep(2)
            rospy.loginfo("Returning to sleep position.")
            self.sleep_publisher.publish(True)

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return
    

if __name__ =='__main__':
    rospy.init_node('push', anonymous=True)
#     try:
#         Push()
#     except rospy.ROSInterruptException:
#         pass
    rospy.spin()