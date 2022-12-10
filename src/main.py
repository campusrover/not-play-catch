#!/usr/bin/env python3

"""
The main node which initializes the other nodes when needed. 
It will interact with the user and wait for inputs to play the game with the arm. 
"""

import rospy
import pickup
import push 
import guess
import keyboard

class Main():
    def __init__(self):
        self.score = [0, 0] # [user, robot]

    def run(self):
        print("Let's play Guess or Kick!")
        while not rospy.is_shutdown(): 
            user_input = input("Please place the apriltag goals in between the purple lines and place the ball underneath one of them. \
                                    \nDon't worry, I won't cheat! \
                                    \nEnter y when you're done! \
                                    \nOr if you want for me to kick the ball right away, then place the goal outside the purple lines and the ball in between the purple lines!.\
                                    \nAnd then press k. ")
            if user_input == "y":
                print("Great, let's start!")
                result = guess.Guess().execute()

                if result == -1: 
                    self.score[0] = self.score[0] + 1
                    if self.score[0] == 2:
                        print("Looks like you won...but I can still win if I can score a penalty kick!\n \
                                Let's set up for one! Apriltag goal outside the purple lines and the ball between the purple lines.\n \
                                Then press enter to continue! ")
                        if input():
                            self.kick(0)
                        
                        score = [0,0]
                elif result == 1: 
                    self.score[1] = self.score[1] + 1
                    if self.score[1] == 2: 
                        print("I won! No extra work for me :)")
                        score = [0,0]
                else: 
                    print("There was some problem, please retry!")
            elif user_input == "k":
                apriltag_num = input("Apriltag 0 or 1? ")
                apriltag_num = int(apriltag_num)
                self.kick(apriltag_num)
            else: 
                print("okay, goodbye!")
                break
                
            print("Current Score (user vs arm) : {}".format(self.score))
    
    # initialize the kicking movement - picking up and pushing the ball. 
    def kick(self, apriltag_num):
        if pickup.Pickup(apriltag_num, False).pickup_ball():
                    push.Push(apriltag_num)
        else: 
            print("I couldn't get the ball, please retry!")

if __name__ =='__main__':
    rospy.init_node('main', anonymous=True)
    try:
        rospy.sleep(4)
        Main().run()
    except rospy.ROSInterruptException:
        pass
    rospy.spin()
