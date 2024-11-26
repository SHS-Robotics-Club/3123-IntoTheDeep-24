package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.MissionTimer;

//@Disabled
// Possible Groups: Competition, Development, Test, Training
@TeleOp(name = "DrivetrainXYXTeleOp", group = "Development")

/**
 * DrivetrainXYXTeleOp
 * Controls drivetrain using left joystick X & Y for direction and right joystick X for rotation.
 * Uses button A for power level control, either low power or high power. High power by default.
 */
public class DrivetrainXYXTeleOp extends OpMode {

    public static final double DRIVETRAIN_LOW_POWER_FACTOR = 0.6;
    public static final double DRIVETRAIN_HIGH_POWER_FACTOR = 0.8;
    public static final double DRIVETRAIN_TURBO_POWER_FACTOR = 1.0;
    public static final double c = 500.0;

    // Define as instance of a Robot class as null
    public Robot robot;
    // Define TeleOp utilities
    private MissionTimer missionTimer;

    // Define local parameters
    private double drivetrainPowerDirX;
    private double drivetrainPowerDirY;
    private double drivetrainPowerRotate;
    private double drivetrainPowerFactor = DRIVETRAIN_LOW_POWER_FACTOR;
    private double buttonAPressStartTime = 0;
    private boolean buttonAPreviouslyPressed = false;

    /**
     * Instantiates and initializes the subsystems and utilities
     * Runs once
     */
    @Override
    public void init(){

        // Instantiate a robot using the hardwareMap constructor
        robot = new Robot(hardwareMap);
        // Instantiate TeleOp utilities
        missionTimer = new MissionTimer();
        
        // Initialize robot
        robot.init();
        // Initialize TeleOp utilities
        missionTimer.init();

        // Set default telemetry
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Power Factor", drivetrainPowerFactor);
        telemetry.update(); // Send "Initialized" and powerFactor to the Driver Station
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

        double currentTime = missionTimer.getTimeMS();

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

            if (pressDuration >= drivetrainPowerFactor) {
                // Long press: set powerFactor to TURBO_POWER_FACTOR
                drivetrainPowerFactor = DRIVETRAIN_TURBO_POWER_FACTOR;
            } else {
                // Brief press: toggle between LOW_POWER_FACTOR and HIGH_POWER_FACTOR
                if (drivetrainPowerFactor == DRIVETRAIN_LOW_POWER_FACTOR) {
                    drivetrainPowerFactor = DRIVETRAIN_HIGH_POWER_FACTOR;
                } else if (drivetrainPowerFactor == DRIVETRAIN_HIGH_POWER_FACTOR || drivetrainPowerFactor == DRIVETRAIN_TURBO_POWER_FACTOR) {
                    drivetrainPowerFactor = DRIVETRAIN_LOW_POWER_FACTOR;
                }
            }
        }

        // Update button state
        buttonAPreviouslyPressed = buttonAPressed;


        // Sample gamepad1 joysticks
        drivetrainPowerDirX = gamepad1.left_stick_x;
        drivetrainPowerDirY = gamepad1.left_stick_y;
        drivetrainPowerRotate = gamepad1.right_stick_x;
        // Request power application to drivetrain
        robot.drivetrain.operate(drivetrainPowerDirX, drivetrainPowerDirY, drivetrainPowerRotate, drivetrainPowerFactor);

        // Report telemetry
        robot.reportTelemetry();
        // Display telemetry
        telemetry.addData("Power Factor", drivetrainPowerFactor);
        telemetry.addData("Press Duration (ms)", currentTime - buttonAPressStartTime);
        telemetry.update();
    }

}
