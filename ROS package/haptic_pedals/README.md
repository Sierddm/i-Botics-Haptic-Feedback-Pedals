# Haptic feedback pedals ROS package

This package contains the following:
### Nodes
- zeromqClient      (communicates with the RaMstix and msd)
- msd               (Calculates the output voltage based on pedal and obstacle input)
- pedalDirection    (Converts the combined encoder positions to a twist)
- forceFeedback     (Calculates the magnitude of feedback based on obstacle location and platform velocity)
- scanToTwist       (Adapter node for using TurtleBot in Gazebo)
- dummyObstacle     (Generates fake obstacle values for testing)
