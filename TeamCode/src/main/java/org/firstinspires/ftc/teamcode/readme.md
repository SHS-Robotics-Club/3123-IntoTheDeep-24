## TeamCode Module

Welcome!

This module, TeamCode, is the place where you will write/paste the code for your team's
robot controller App. OpModes are in the teleop folder. The primary robot object class is in this folder. The subsystem objects classes code code (such as Claw, ClawArm and ClawLift) are in the subsystems folder, and there are utilitiy objects classes (such as MissionTimer) in the util folder. The structure of the OpModes and the robot object are described below.

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

The Robot object class is where the robot subsystems such as drivetrain and claw,
along with utilities such as a shared free-running mission timer, are defined. In order to access
the methods defined for each subsystem, getter methods are used in the Robot object class (such as
robot.getDrivetrain() and robot.getClaw() ). For example, the foloowing code initializes the drivetrain,
claw and mission timer.

    robot.getDrivetrain().init();
    robot.getClaw().init();
    robot.getMissionTimer().init();

#### Drivetrain
The drivetrain object consists of four Mecanum wheels, each with its own motor. Power is applied to these wheels using a power factor
that limits the maximum power.

#### Claw
The claw object consists of two regular servos. Both servos are operated together to one of three positions:
init(), open() or close(). The time it takes to perform these actions is controlled by a time constant to slow down the normal servo speed.

#### ClawArm
The claw arm object consists of a bar where the claw lift and claw are attached. A motor is used to rotate the bar that starts from a hard stop position and can be move using the X, A and B buttons on the gamepad #2, as well as the left joystick. Power is applied to the claw arm motor using a power factor that limits the maximum power.

#### ClawLift
The claw lift object consists of a goBILDA quad slide mechanism where the claw is attached at the top. A motor is used to activate the slide up and down using the dpad buttons on the gamepad #2, as well as the right joystick. Power is applied to the claw lift motor using a power factor that limits the maximum power.


