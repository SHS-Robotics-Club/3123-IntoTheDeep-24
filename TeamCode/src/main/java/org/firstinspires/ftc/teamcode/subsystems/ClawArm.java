package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * The ClawArm class defines and controls the claw arm motor.
 *
 * Configuration
 *     m_claw_arm - claw arm motor config name
 *
 * PUBLIC METHODS:
 *     ClawArm(hardwareMap, telemetry) - constructor for instantiating a claw with try/catch
 *     void init() - initializes the claw arm motor
 *     void seManualPower() - Sets power level of the motor
 *     void setTargetPosition() - moves claw arm to a specified position
 *     void update() - Updates the arm's state. Call this in the loop method.
 *     void reportTelemetry() - reports claw arm telemetry information
 *     void update() - updates claw arm position during move
 *
 * VERSION   DATE     WHO  DETAIL
 * 00.01.00  30Nov24  SEB  Initial release
 *
 */public class ClawArm {

    // Constants
    private static final double CLAW_ARM_POWER_MAX = 1.0; // Maximum motor power
    private static final double CLAW_ARM_POWER_MIN = -1.0; // Minimum motor power
    private static final double CLAW_ARM_POWER_FACTOR = 0.6; // Scales motor power
    // goBILDA 5203 Series motor with a 19.2:1 gear ratio
    private static final int ENCODER_TICKS_PER_REVOLUTION = 537;
    private static final double GEAR_RATIO = 1.0; // Ratio between motor and arm shaft
    private static final double TICKS_PER_DEGREE = ENCODER_TICKS_PER_REVOLUTION * GEAR_RATIO / 360.0;

    // Arm stop positions in degrees
    private static final double CLAW_ARM_LOWER_STOP_POSITION = 10.0; // Minimum arm angle
    private static final double CLAW_ARM_UPPER_STOP_POSITION = 170.0; // Maximum arm angle
    private static final double CLAW_ARM_INIT_POSITION = 45.0;

    // Hardware
    private DcMotor m_claw_arm;
    private Telemetry telemetry;

    // Current state
    private boolean isManualControl = true;
    private double targetPosition = CLAW_ARM_INIT_POSITION; // Target position in degrees

    /**
     * Constructor
     *
     * @param hardwareMap Hardware map for accessing hardware
     * @param telemetry   Telemetry object for reporting
     */
    public ClawArm(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        try {
            // Instantiate claw arm motor as DcMotor class and use the configuration
            // to connect the port
            m_claw_arm = hardwareMap.get(DcMotor.class, "clawArmMotor");
        } catch (Exception e) {
            telemetry.addData("Error", "Claw arm motor initialization failed: " + e.getMessage());
            telemetry.update();
        }
    }

    /**
     * Initializes claw arm motor
     * Motor is set to zero power and power is based directly from setPower (no PID controller).
     */
    public void init() {

        if (m_claw_arm != null) {
            // Initialize motors to brake applies without encoders
            m_claw_arm.setDirection(DcMotorSimple.Direction.FORWARD);
            m_claw_arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m_claw_arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            m_claw_arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            // Initialize claw arm position
            // setTargetPosition(targetPosition);
        } else {
            telemetry.addData("Error", "Claw arm motor is not initialized.");
        }


    }
    /**
     * Set manual power to the arm motor, with safety limits.
     *
     * @param power Motor power (-1.0 to 1.0)
     */
    public void setManualPower(double power) {
        isManualControl = true;

        // Clamp power to range
        power = Math.max(CLAW_ARM_POWER_MIN, Math.min(CLAW_ARM_POWER_MAX, power * CLAW_ARM_POWER_FACTOR));

        // Check if the arm is near the stop positions
        double currentPosition = getCurrentPosition();
        if ((currentPosition <= CLAW_ARM_LOWER_STOP_POSITION && power < 0) ||
                (currentPosition >= CLAW_ARM_UPPER_STOP_POSITION && power > 0)) {
            // Prevent movement beyond limits
            m_claw_arm.setPower(0);
        } else {
            m_claw_arm.setPower(power);
        }
    }

    /**
     * Set the target position for the arm in degrees.
     *
     * @param position Target position in degrees
     */
    public void setTargetPosition(double position) {
        isManualControl = false;

        // Clamp position to range
        targetPosition = Math.max(CLAW_ARM_LOWER_STOP_POSITION, Math.min(CLAW_ARM_UPPER_STOP_POSITION, position));
        // Use DcMotor class methods to control claw arm motor
        m_claw_arm.setTargetPosition((int) (targetPosition * TICKS_PER_DEGREE));
        m_claw_arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        m_claw_arm.setPower(CLAW_ARM_POWER_MAX); // Use maximum power for motor PID position control
    }

    /**
     * Updates the arm's state. Call this in the loop method.
     */
    public void update() {
        // The value of isBusy is controlled internally by the motor controller for DcMotor types
        // Its value is determined internally by the motor controller firmware based on the current
        // encoder position, the target position, and the motor's RUN_TO_POSITION mode.
        if (!isManualControl && !m_claw_arm.isBusy()) {
            // Stop motor once the target is reached
            m_claw_arm.setPower(0);
            m_claw_arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /**
     * Reports telemetry data about the arm's state.
     */
    public void reportTelemetry() {
        telemetry.addData("-----  CLAW ARM", "  -----");
        telemetry.addData("Claw Arm", "Pos: %.2f° | Target: %.2f° | Manual: %b | Power Factor: %.2f",
                getCurrentPosition(), targetPosition, isManualControl, CLAW_ARM_POWER_FACTOR);
    }

    /**
     * Gets the current position of the arm in degrees.
     *
     * @return Current arm position in degrees
     */
    public double getCurrentPosition() {
        return m_claw_arm.getCurrentPosition() / TICKS_PER_DEGREE;
    }
}
