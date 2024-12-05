package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Robot;

//@Disabled
// Possible Groups: Competition, Development, Test, Training
@TeleOp(name = "MecDriveClawArm", group = "Development")

/**
 * MecDriveClawArm
 * Controls drivetrain using left joystick X & Y for direction and right joystick X for rotation.
 * Uses button A for power level control, either low power or high power. High power by default.
 *
 * VERSION   DATE     WHO  DETAIL
 * 00.01.00  30Nov24  SEB  Initial release
 *
 */
public class MecDriveClawArm extends OpMode {

    public static final double DRIVETRAIN_X_POWER_CORRECTION = 1.1;
    public static final double DRIVETRAIN_LOW_POWER_FACTOR = 0.6;
    public static final double DRIVETRAIN_HIGH_POWER_FACTOR = 0.8;
    public static final double DRIVETRAIN_TURBO_POWER_FACTOR = 1.0;
    public static final double LONG_PRESS_THRESHOLD_MS = 500.0;
    public static final double MOTOR_ZERO_POWER = 0.0;
    private static final double TELEMETRY_UPDATE_PERIOD_MS = 500.0; // Update every 500ms
    private static final int CLAW_ARM_DRIVING_BACKWARD_POSITION = 14;
    private static final int  CLAW_ARM_DRIVING_FORWARD_POSITION = 100;
    private static final double JOYSTICK_DEADZONE = 0.1;

    // Define as instance of a Robot class as null
    private Robot robot;
    // Define local parameters
    private double drivetrainPowerDirX;
    private double drivetrainPowerDirY;
    private double drivetrainPowerRotate;
    private double drivetrainPowerFactor = DRIVETRAIN_LOW_POWER_FACTOR;
    private double buttonAPressStartTime = 0;
    private boolean buttonAPreviouslyPressed = false;
    private double nextTelemetryUpdateTime = 0.0;

    /**
     * Instantiates and initializes the subsystems and utilities
     * Runs once
     */
    @Override
    public void init(){

        // Instantiate a robot using the hardwareMap constructor
        robot = new Robot(hardwareMap, telemetry);

        // Make sure all subsystems are properly instantiated
        if (robot.getDrivetrain() == null || robot.getClaw() == null || robot.getClawArm() == null) {
            telemetry.addData("Error", "Subsystem initialization failed. Check hardware configuration.");
            telemetry.update();
            requestOpModeStop();
            return;
        }
        // Initialize robot subsystems
        robot.getDrivetrain().init();
        robot.getClaw().init();
        robot.getClawArm().init();

        // Initialize shared resources
        robot.getMissionTimer().init();

        // Set default telemetry
        telemetry.addData("Status", "Initialized");
        robot.getDrivetrain().reportTelemetry();
        telemetry.addData("Power Factor", drivetrainPowerFactor);
        robot.getClaw().reportTelemetry();
//        robot.getClawArm().reportTelemetry();
        telemetry.update(); // Send "Initialized" and powerFactor to the Driver Station
    }

    /**
     * Initialization complete. Wait here for PLAY.
     */
    @Override
    public void init_loop() {

        // Report telemetry while waiting
        telemetry.addData("Status", "Initialized");
        robot.getDrivetrain().reportTelemetry();
        telemetry.addData("Power Factor", drivetrainPowerFactor);
        robot.getClaw().reportTelemetry();
        robot.getClawArm().getTelemetry();
        telemetry.addData("Joystick Y", -gamepad2.left_stick_y);  // DEBUG //
        telemetry.addData("Button A", gamepad2.a); // DEBUG //
        telemetry.addData("Button B", gamepad2.b); // DEBUG //
        telemetry.update(); // Send "Initialized" and powerFactor to
    }

    @Override
    public void start() {

        // Open claw is the default position
        robot.getClaw().open();
    }

    /**
     * Main TeleOp operation loop
     * Wait here until STOP
     */
    @Override
    public void loop() {

        double currentTime = robot.getMissionTimer().getTimeMS();

        //**********DRIVETRAIN **********/
        /////
        //  Gamepad1 Button A
        //
        boolean buttonAPressed = gamepad1.a;

        if (buttonAPressed && !buttonAPreviouslyPressed) {  // Respond to drivetrain power factor change request
            // Button just pressed, record the time
            buttonAPressStartTime = currentTime;
        } else if (!buttonAPressed && buttonAPreviouslyPressed) {
            // Button just released, calculate the press duration
            double pressDuration = currentTime - buttonAPressStartTime;

            if (pressDuration >= LONG_PRESS_THRESHOLD_MS) {
                // Long press: set powerFactor to TURBO_POWER_FACTOR
                drivetrainPowerFactor = DRIVETRAIN_TURBO_POWER_FACTOR;
            } else {
                // Brief press: toggle between LOW_POWER_FACTOR and HIGH_POWER_FACTOR
                if (drivetrainPowerFactor == DRIVETRAIN_LOW_POWER_FACTOR) {
                    drivetrainPowerFactor = DRIVETRAIN_HIGH_POWER_FACTOR;
                } else if (drivetrainPowerFactor == DRIVETRAIN_HIGH_POWER_FACTOR ||
                        drivetrainPowerFactor == DRIVETRAIN_TURBO_POWER_FACTOR) {
                    drivetrainPowerFactor = DRIVETRAIN_LOW_POWER_FACTOR;
                }
            }
        }

        // Update button state
        buttonAPreviouslyPressed = buttonAPressed;

        // Sample gamepad1 joysticks
        drivetrainPowerDirX = gamepad1.left_stick_x * DRIVETRAIN_X_POWER_CORRECTION;
        drivetrainPowerDirY = -gamepad1.left_stick_y;
        drivetrainPowerRotate = gamepad1.right_stick_x;
        // Request power application to drivetrain
        robot.getDrivetrain().operate(drivetrainPowerDirX, drivetrainPowerDirY, drivetrainPowerRotate, drivetrainPowerFactor);

        //**********  CLAW  **********/
        // Open or close the claw based on gamepad input
        if (gamepad2.left_bumper) {
            robot.getClaw().open();
        } else if (gamepad2.right_bumper) {
            robot.getClaw().close();
        }
        // Update Claw subsystems
        robot.getClaw().update();

        //**********  CLAW ARM  **********/
        double gp2LeftJoystickY = -gamepad2.left_stick_y; // Manual control for arm (invert Y)

        if (Math.abs(gp2LeftJoystickY) < JOYSTICK_DEADZONE) { // Dead zone to avoid accidental movement

            if (gamepad2.a) {
                robot.getClawArm().setTargetPosition(CLAW_ARM_DRIVING_BACKWARD_POSITION); // Move arm
            } else if (gamepad2.b) {
                robot.getClawArm().setTargetPosition(CLAW_ARM_DRIVING_FORWARD_POSITION); // Move arm
            } else {
                robot.getClawArm().setManualPower(MOTOR_ZERO_POWER);
            }
        } else {
            robot.getClawArm().setManualPower(gp2LeftJoystickY);
        }

        //**********  TELEMETRY  **********/
        // Check if it's time to update telemetry
        if (robot.getMissionTimer().hasElapsed(nextTelemetryUpdateTime)) {
            // Drivetrain Telemetry    //********** TELEMETRY **********/
            // Report telemetry from subsystems
            robot.getDrivetrain().reportTelemetry();
            telemetry.addData("Power Factor", drivetrainPowerFactor);
            robot.getClaw().reportTelemetry();
            robot.getClawArm().getTelemetry();
            telemetry.update();
            // Calculate the next update time
            nextTelemetryUpdateTime += TELEMETRY_UPDATE_PERIOD_MS;
        }
    }
}
