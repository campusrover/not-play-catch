#!/usr/bin/env python3

"""
This is a guessing game where the robot arm has to choose between two apriltag goals 
and then push the ball from under it. If it guesses correctly, then it will gain a point. 
If it guesses incorrectly, the user will gain a point. 
"""

import rospy 
import tf
import math
from geometry_msgs.msg import Point, Pose
from std_msgs.msg import Bool, String, Float32
import random

class Guess():

    def __init__(self):
        rospy.loginfo("initializing boxes node....")
        rospy.loginfo("please place the ball and the apriltag boxes in between the purple lines in front of the robot")
        self.listener = tf.TransformListener()
        self.point_publisher = rospy.Publisher("/arm_control/point", Point, queue_size=1)
        self.home_publisher = rospy.Publisher("/arm_control/home", Bool, queue_size=1)
        self.sleep_publisher = rospy.Publisher("/arm_control/sleep", Bool, queue_size=1)
        self.gripper_publisher = rospy.Publisher("/arm_control/gripper", String, queue_size=1)
        self.celebrate_publisher = rospy.Publisher("/arm_control/celebrate", Bool, queue_size=1)
        self.time_publisher = rospy.Publisher("/arm_control/time", Float32, queue_size=1)

        self.open = "open"
        self.close = "close"

        # wait for 3 sec to get connections properly set up
        rospy.sleep(3)

    def tag_lookup(self, num):
        try: 
            april_point, april_rot = self.listener.lookupTransform('/world' ,'/at{}'.format(num), rospy.Time())
            # print("transform between world and at{} is {}".format(num, april_point))
            return april_point
        except: 
            return None
    
    def ball_lookup(self):
        try: 
            ball_point, ball_rot = self.listener.lookupTransform('/world' ,'/ball', rospy.Time())
            print(ball_point)
            if ball_point[0] == 0 and ball_point[1] == 0: 
                print("I don't see the ball...")
                return False
            else: 
                print("I see the ball!")
                return True
        except: 
            print("No ball...")
            return False

    def execute(self):
        guess = random.randint(0,1)

        print("I'm guessing Apriltag Goal #{}! Let's see if I'm right!".format(guess))

        april_point = self.tag_lookup(guess)

        if april_point: 
            april_angle = math.atan2(april_point[1], april_point[0]) + 0.05

            # set the arm to the sleep position to start off 
            rospy.loginfo("sending the arm to sleep position")
            self.sleep_publisher.publish(True)
            # wait for the arm to execute the command
            rospy.sleep(2)

            # move the waist first 
            rospy.loginfo("Moving waist first by {}".format(april_angle))
            self.point_publisher.publish(Point(0, april_angle, 0))
            # wait for the command to execute
            rospy.sleep(2)

            rospy.loginfo("changing the trajectory time to 0.9")            
            self.time_publisher.publish(0.9)
            rospy.sleep(2)

            rospy.loginfo("starting to push the arm")
            self.point_publisher.publish(Point(0.15, april_angle, -0.05))
            rospy.sleep(2)

            rospy.loginfo("changing the trajectory time to 1.5")            
            self.time_publisher.publish(1.5)
            rospy.sleep(2)

            self.point_publisher.publish(Point(-0.1, april_angle, 0))
            rospy.sleep(5)

            if self.ball_lookup():
                print("I got it right!!")
                self.celebrate_publisher.publish(True)
                rospy.sleep(5)

                # return to sleep position with the ball and wait
                self.sleep_publisher.publish(True)
                rospy.sleep(3)
                return 1 
            else: 
                print("I got it wrong...")
                 # return to sleep position with the ball and wait
                self.sleep_publisher.publish(True)
                rospy.sleep(3)
                return -1
        else: 
            print("But I couldn't find the april tag that I guessed....")
            return 0


        

if __name__ =='__main__':
    rospy.init_node('catch', anonymous=True)
    # try:
    #     Guess()
    # except rospy.rosinterruptexception:
    #     pass
    rospy.spin()