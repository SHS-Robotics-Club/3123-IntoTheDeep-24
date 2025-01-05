package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * The ClawArm class defines and controls the claw arm motor.
 *
 * Test Results:
 *     Init position: 0.0 (reset during init())
 *     Fully forward position: -90.5 degrees
 *     Vertical position: -28.83 degrees
 *     Driving forward position: -47.0 degrees
 *     Driving backward position: -11.0 degrees
 *
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
 * 00.01.01  03Dec24  SEB  Modify stop positions.
 * 00.02.00  02Jan25  SEB  New motor and external gearing.
 *
 */public class ClawArm {

    // Constants
    private static final double CLAW_ARM_POWER_MAX = 1.0; // Maximum motor power
    private static final double CLAW_ARM_POWER_MIN = -1.0; // Minimum motor power
    private static final double CLAW_ARM_POWER_FACTOR = 0.1; // Scales motor power
    // goBILDA 5203 Series motor with a 19.2:1 gear ratio
    public static final double ENCODER_TICKS_PER_REVOLUTION = 1425.1;  // goBILDA  5204-8002-0051
    public static final double GEAR_RATIO = 2.0; // Ratio between motor and arm shaft
    public static final double TICKS_PER_DEGREE = ENCODER_TICKS_PER_REVOLUTION * GEAR_RATIO / 360.0;  // 7.917
    // Arm stop positions in degrees
    private static final int CLAW_ARM_LOWER_STOP_POSITION = 0; // Minimum arm angle
    private static final int CLAW_ARM_UPPER_STOP_POSITION = 1200; // Maximum arm angle

    private static final int CLAW_ARM_INIT_POSITION = 50;

    private static final long SAFETY_TIMEOUT_MS = 5000;  // Loop safety timeout

    // Hardware
    private DcMotor m_claw_arm;
    private Telemetry telemetry;

    // Current state
    private double targetPosition = CLAW_ARM_INIT_POSITION; // Target position in degrees
    private double initialPosition = CLAW_ARM_INIT_POSITION; // Target position in degrees

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
            this.m_claw_arm = hardwareMap.get(DcMotor.class, "m_claw_arm");

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
            m_claw_arm.setDirection(DcMotorSimple.Direction.FORWARD);  // new motor 02Jan25
            m_claw_arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m_claw_arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m_claw_arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // Initialize claw arm position
            // setTargetPosition(initialPosition);
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

        // Clamp power to range
        // power = Math.max(CLAW_ARM_POWER_MIN, Math.min(CLAW_ARM_POWER_MAX, power * CLAW_ARM_POWER_FACTOR));
        power = Math.max(CLAW_ARM_POWER_MIN, Math.min(CLAW_ARM_POWER_MAX, power)) * CLAW_ARM_POWER_FACTOR;

        // Check if the arm is near the stop positions
        double currentPosition = getCurrentPositionEncoder();
        if ((currentPosition <= CLAW_ARM_LOWER_STOP_POSITION && power < 0) ||
                (currentPosition >= CLAW_ARM_UPPER_STOP_POSITION && power > 0)) {
            // Prevent movement beyond limits
            m_claw_arm.setPower(0);
        } else {
            m_claw_arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            m_claw_arm.setPower(power);
        }
    }

    /**
     * Set the target position for the arm in degrees.
     *
     * @param position Target position in degrees
     */
    public void setTargetPosition(int position) {

        // Clamp position to operating limits
        if (position < CLAW_ARM_LOWER_STOP_POSITION) {
            position = CLAW_ARM_LOWER_STOP_POSITION;
        } else if (position > CLAW_ARM_UPPER_STOP_POSITION) {
            position = CLAW_ARM_UPPER_STOP_POSITION;
        }

        // un motor to requested position
        m_claw_arm.setTargetPosition(position); // Set target position
        m_claw_arm.setMode(DcMotor.RunMode.RUN_TO_POSITION); // Move the motor to the target position
        m_claw_arm.setPower(CLAW_ARM_POWER_FACTOR); // Apply full power with power factor

        ElapsedTime runtime = new ElapsedTime();
        runtime.reset(); // Reset the timer before starting
        long timeout = SAFETY_TIMEOUT_MS; // Timeout value in seconds (for example, 5 seconds)

        // Wait until the motor reaches the target position
        while (m_claw_arm.isBusy() && runtime.milliseconds() < timeout) {
            // Update telemetry periodically to avoid excessive updates
            if (runtime.milliseconds() % 100 < 50) { // Update every 100ms
                telemetry.addData("Running to Position", "Target: %4d, Current: %4d",
                        position, m_claw_arm.getCurrentPosition());
                telemetry.update();
            }
        }

        // If the motor hasn't reached the target position within the timeout
        if (runtime.milliseconds() >= timeout) {
            telemetry.addData("Error", "Timeout reached while moving to position");
            telemetry.update();
        }
    }

    /**
     * Reports telemetry data about the arm's state.
     */
    public void getTelemetry() {
        telemetry.addData("-----  CLAW ARM", "  -----");
        telemetry.addData("Claw Arm", "Pos: %d° | Target: %.2f° | Factor: %.2f",
                getCurrentPositionEncoder(), targetPosition, CLAW_ARM_POWER_FACTOR);
        telemetry.addData("Target Encoder", (int) (targetPosition * TICKS_PER_DEGREE)); // DEBUG //
        telemetry.addData("Claw Arm Power", m_claw_arm.getPower()); // DEBUG //
    }

    /**
     * Gets the current position of the arm in degrees.
     *
     * @return Current arm position in degrees
     */
    public double getCurrentPositionDegrees() {
        return m_claw_arm.getCurrentPosition() / TICKS_PER_DEGREE;
    }

    /**
     * Gets the current position of the arm in degrees.
     *
     * @return Current arm position in degrees
     */
    public int getCurrentPositionEncoder() {
        return m_claw_arm.getCurrentPosition();
    }
}
