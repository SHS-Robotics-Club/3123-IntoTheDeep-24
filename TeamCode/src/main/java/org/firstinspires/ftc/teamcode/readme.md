## TeamCode Module

Welcome!

This module, TeamCode, is the place where you will write/paste the code for your team's
robot controller App. This module is currently empty (a clean slate) but the
process for adding OpModes is straightforward.

## Creating your own OpModes

The preferred opmodes are base on OpMode rather than LinearOpMode. This means the required methods are:
* init() - Runs once when driver presses INIT.
* loop() - Runs repeatedly after driver presses PLAY.
The optional methods are:
* init_loop() - Runs repeated \ly after driver presses INIT but before PLAY.
* start() - Runs once when drivers presses PLAY.
* stop() - Runs once when driver presses STOP.

The easiest way to create your own OpMode is to copy a Sample OpMode and make it your own.

Sample opmodes exist in the FtcRobotController module.
To locate these samples, find the FtcRobotController module in the "Project/Android" tab.

In addition to the sample opmodes in
  ..\TeamCode/src/main/java/org/firstinspires/ftc/teamcode\teleops (and \autonomous)
additional opmode samples are available in
   FtcRobotController/java/org.firstinspires.ftc.robotcontroller/external/samples

When defining a new opmode, the four recommended groups are Competition, Development, Test, and Training.

### Robot Structure

The Robot class is where the robot subsystems such as drivetrain and claw,
along with utilities such as a shared free-running mission timer, are defined. In order to access
the methods defined for these, getter methods are used in the Robot class such as
robot.getDrivetrain() and robot.getClaw(). For example, the foloowing code initialized the drivetrain,
claw and mission timer.

    robot.getDrivetrain().init();
    robot.getClaw().init();
    robot.getMissionTimer().init();

#### Drivetrain
The drivetrain consists of four Mecanum wheels. Power is applied to these wheels using a power factor
that limits the maximum power.

#### Claw
The claw consists of two regular servos. Both servos are operated together to one of three positions:
init(), open() or close(). The time it takes to perform these actions is controlled by a constant.

