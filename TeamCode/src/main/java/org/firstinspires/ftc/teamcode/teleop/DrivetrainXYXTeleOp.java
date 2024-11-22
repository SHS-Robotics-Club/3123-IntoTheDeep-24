package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.MissionTimer;

//@Disabled
// Possible Groups: Competition, Development, Test, Training
@TeleOp(group = "Development", name = "DrivetrainOnly")

/**
 * DrivetrainXYXTeleOp
 * Controls drivetrain using left joystick X & Y for direction and right joystick X for rotation.
 * Uses button A for power level control, either low power or high power. High power by default.
 */
public class DrivetrainXYXTeleOp extends OpMode {

    public static final double DRIVETRAIN_LOW_POWER_FACTOR = 0.6;
    public static final double DRIVETRAIN_HIGH_POWER_FACTOR = 0.8;
    public static final double DEBOUNCE_DELAY = 500.0;

    // Define as instance of a Robot class as null
    public Robot robot;
    // Define TeleOp utilities
    public MissionTimer missionTimer;

    // Define local parameters
    double drivetrainPowerDirX;
    double drivetrainPowerDirY;
    double drivetrainPowerRotate;
    double drivetrainPowerFactor;
    double previousPowerRequestTime;

    /**
     * Instantiates and initializes the subsystems and utilities
     * Runs once
     */
    @Override
    public void init(){

        // Instantiate a robot using the hardwareMap constructor
        robot = new Robot(hardwareMap);
        // Instantiate TeleOp utilities
        MissionTimer missionTimer = new MissionTimer();
        // Initialize robot
        robot.init();
        // Initialize TeleOp utilities
        missionTimer.init();
        // Initialize settings
        drivetrainPowerFactor = DRIVETRAIN_LOW_POWER_FACTOR;

        // Grab current time for use later
        previousPowerRequestTime = missionTimer.getTimeMS();

    }

    /**
     * Initialization complete. Wait here for PLAY.
     */
    public void init_loop() {

        robot.reportTelemetry();

    }

    /**
     * Main TeleOp operation loop
     * Wait here until STOP
     */
    @Override
    public void loop() {

        //  Sample gamepad1 button A
        if (gamepad1.a) {  // Respond to drivetrain power factor change request
            // Check to make sure that the debounce time has expired
            if (missionTimer.getTimeMS() > (previousPowerRequestTime + DEBOUNCE_DELAY)) {
                // Toggle drivetrain power factor
                drivetrainPowerFactor = (drivetrainPowerFactor == DRIVETRAIN_LOW_POWER_FACTOR) ?
                        DRIVETRAIN_HIGH_POWER_FACTOR : DRIVETRAIN_LOW_POWER_FACTOR;
                // Save off power change request time
                previousPowerRequestTime = missionTimer.getTimeMS();
            }

        }

        // Sample gamepad1 joysticks
        drivetrainPowerDirX = gamepad1.left_stick_x;
        drivetrainPowerDirY = gamepad1.left_stick_y;
        drivetrainPowerRotate = gamepad1.right_stick_x;
        // Request power application to drivetrain
        robot.drivetrain.operate(drivetrainPowerDirX, drivetrainPowerDirY, drivetrainPowerRotate, drivetrainPowerFactor);

    }

}
