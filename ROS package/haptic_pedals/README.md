# Haptic feedback pedals ROS package

## Content

This package contains the following:
### Nodes
- zeromqClient      (communicates with the RaMstix and msd)
- msd               (Calculates the output voltage based on pedal and obstacle input)
- pedalDirection    (Converts the combined encoder positions to a twist)
- forceFeedback     (Calculates the magnitude of feedback based on obstacle location and platform velocity)
- scanToTwist       (Adapter node for using TurtleBot in Gazebo)
- dummyObstacle     (Generates fake obstacle values for testing)

### Messages

- EncoderValues
- Forces
- Movement

### Services

- RequestFeedback
- RequestForce

## Required packages

This ROS package is dependend on the following packages:

- turtlebot3
- turtlebot3_msgs
- turtlebot3_simulations
- zeromq_catkin

## Usage

**!! New accelerometers have been placed under the pedals, calibrate accelerometer values in the softeare before use !!**
**!! Do not turn on the powersupplies before the software, this will cause the pedals to behave unexpected !!**

To run the demo the following steps should be considered:

1. SSH into the RaMstix using ```SSH root@10.0.5.28``` (ww:ram)
2. cd into ramstix-io/files/ and run ```make all```
3. cd into build/bin/ramstix/unit-tests
4. run ```./zeromqserver_test```
5. on the computer run ```roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch```, this also includes roscore
6. rosrun all nodes from haptic_pedals (zeromqClient/msd/forceFeedback/pedalDirection/scanToTwist)
7. determine if the nodes are communicating 
8. position the pedals in their lowest position
9. if all software is running, turn on the powersupplies
10. Now the pedals should operate the TurtleBot in Gazebo
11. Place objects in the empty world in order to demonstrate the force feedback effect
