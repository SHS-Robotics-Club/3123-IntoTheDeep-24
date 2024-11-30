## Overview
The subsystems folder contains structures that are used by a robot such as drivetrain, slide lift and claw.

### Drivetrain
The drivetrain consist of four motors that may be controlled using a gamepad.
  PUBLIC METHODS:
     Drivetrain(hardwareMap) - constructor for instantiating a drivetrain
     void init() - initializes the components of the drivetrain
     void reportTelemetry() - reports drivetrain telemetry information
     void operate(powerX, powerY, powerRotate, powerFactor) - applies power to the motors
         [based on video "FTC Programming 9 of n: Deadline & Toggle Drive Power" by Brandon Pacewic
          see https://www.youtube.com/watch?v=06dOk1S6oEg]

## Claw
The Claw class defines all the claw components for a robot.
 Connections
     s_claw_l - Servo 0 por
     s_claw_r - Servo 1 port

 PUBLIC METHODS:
     Claw(hardwareMap, missonTimer) - constructor for instantiating a claw
     void init() - initializes the components of the claw
     void open() - opens claw (default)
     void close() - closes claw
     void reportTelemetry() - reports claw telemetry information
     void update() - updates claw positions during move
 PRIVATE METHODS:
     moveClaw(leftTarget, rightTarget) - configures claw when move requested

