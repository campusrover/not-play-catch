# not-play-catch

This project was created to play catch with the arm. However due to certain limitations, it didn't work out the way as expected. 
Instead, the game Guess or Kick! was born, but I liked the name of play-catch so I just added a not in front of it... 

## Installation 
- You will need to download the Interbotix repository for their robotic arms: https://github.com/Interbotix/interbotix_ros_manipulators
- You will also need to download the repository arm_control: https://github.com/campusrover/arm_control
- And then download this repository. 

## Running the Code 
-	roslaunch arm_control command_center.launch
      - If you’re working with the actual arm, ensure that it is turned on and plugged into the computer before starting. Otherwise, if you’re working in simulation, then  uncomment <arg name=use_sim value=true /> line in the launch file. 
      -	If you want to just run this file and send commands to the arm, then you can also run: rosrun arm_control send_command.py   
-	roslaunch playcatch play-catch.launch 
