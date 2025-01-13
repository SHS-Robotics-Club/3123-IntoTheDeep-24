## Software TODO List
This file maintains a list of high and low priority features that are needed or desired to
improve the robot's function.

### High Priority
The following are high priority improvements to the robot.
* Create build configuration and robot constants into separate files, probably at the top level.

### Low Priority
The following are low priority improvement to the robot.
* None identified.

### Completion Notes
* 24Nov24 Updated Robot class to have private subsystems that are made public by getter methods: i.e. getDrivetrain(), getClaw().
* 24Nov24 Added initial Claw code. Slows claw movement to perform in 0.5 second period.
* 03Jan25 Claw Arm Control - controls the motor that moves the claw up and down.
* 03Jan25 Claw Slide Control - lifts samples up to one of two baskets using two preset heights.
