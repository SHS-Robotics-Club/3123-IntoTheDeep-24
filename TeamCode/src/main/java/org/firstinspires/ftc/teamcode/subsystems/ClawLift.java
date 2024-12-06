package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * The ClawLift class defines and controls the claw lift motor.
 *
 * Test Results:
 *     Init position:
 *
 * Configuration
 *     m_claw_lift - claw lift motor config name
 *
 * PUBLIC METHODS:
 *     ClawLift(hardwareMap, telemetry) - constructor for instantiating a claw lift try/catch
 *     void init() - initializes the claw lift motor
 *     void runManualPower() - Allows use of manual power control
 *     void runToPosition() - Moves claw lift to a specified position
 *     void reportTelemetry() - reports claw arm telemetry information
 *
 * PRIVATE METHODS:
 *     None
 *
 * VERSION   DATE     WHO  DETAIL
 * 00.01.00  06Dec24  SEB  Initial release
 *
 */public class ClawLift {

    // Constants
    private static final double CLAW_LIFT_POWER_MAX = 1.0; // Maximum motor power
    private static final double CLAW_LIFT_POWER_MIN = -1.0; // Minimum motor power
    private static final double CLAW_LIFT_POWER_FACTOR = 0.6; // Scales motor power
    private static final double CLAW_LIFT_POWER_ZERO = 0.0; // Scales motor power
    // goBILDA 5203 Series motor with a 19.2:1 gear ratio
    public static final int ENCODER_TICKS_PER_REVOLUTION = 537;
    public static final double GEAR_RATIO = 1.0; // Ratio between motor and arm shaft
    public static final double TICKS_PER_DEGREE = ENCODER_TICKS_PER_REVOLUTION * GEAR_RATIO / 360.0;
    // Arm stop positions in degrees
    private static final int CLAW_LIFT_UPPER_STOP_POSITION = (int) ((5 * ENCODER_TICKS_PER_REVOLUTION) + 0); // Minimum arm angle
    private static final int CLAW_LIFT_LOWER_STOP_POSITION = 0; // Minimum arm angle

    private static final long SAFETY_TIMEOUT_MS = 5000;  // Loop safety timeout

    // Hardware
    private DcMotor m_claw_lift;
    private Telemetry telemetry;

    // Current state
    private double manualPower = CLAW_LIFT_POWER_ZERO;
    private int targetPosition = 0; // Target position in degrees

    /**
     * Claw lift constructor
     *
     * @param hardwareMap Hardware map for accessing hardware
     * @param telemetry   Telemetry object for reporting
     */
    public ClawLift(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        try {
            // Instantiate claw arm motor as DcMotor class and use the configuration
            // to connect the port
            this.m_claw_lift = hardwareMap.get(DcMotor.class, "m_claw_lift");

        } catch (Exception e) {
            telemetry.addData("Error", "Claw lift motor initialization failed: " + e.getMessage());
            telemetry.update();
        }
    }

    /**
     * Initializes claw lift motor
     * Motor is set to zero power and power is set to manual control mode.
     */
    public void init() {

        if (m_claw_lift != null) {
            // Initialize motors to brake applies without PID controller
            m_claw_lift.setDirection(DcMotorSimple.Direction.REVERSE);
            m_claw_lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m_claw_lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m_claw_lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } else {
            telemetry.addData("Error", "Claw lift motor is not initialized.");
        }
    }

    /**
     * Set manual power to the claw lift motor, with safety limits.
     *
     * @param power Motor power (-1.0 to 1.0)
     */
    public void runManualPower(double power) {

        // Determine motor power level
        manualPower = power * CLAW_LIFT_POWER_FACTOR;

        // Check if the arm is near the stop positions
        int currentPosition = m_claw_lift.getCurrentPosition();
        if ((currentPosition <= CLAW_LIFT_LOWER_STOP_POSITION && manualPower < 0) ||
                (currentPosition >= CLAW_LIFT_UPPER_STOP_POSITION && manualPower > 0)) {
            // Prevent movement beyond limits
            m_claw_lift.setPower(CLAW_LIFT_POWER_ZERO);
        } else {
            // Ensure the motor is back in the normal operating mode
            m_claw_lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            // Run lift
            m_claw_lift.setPower(manualPower);
        }
    }

    /**
     * Run claw lift to target position with timeout. Stays in run to position mode.
     *
     * @param position Target position in ticks
     */
    public void runToPosition(int position) {

        // Clamp position to operating limits
        if (targetPosition < CLAW_LIFT_UPPER_STOP_POSITION) {
            targetPosition = CLAW_LIFT_UPPER_STOP_POSITION;
        } else if (targetPosition > CLAW_LIFT_UPPER_STOP_POSITION) {
            targetPosition = CLAW_LIFT_UPPER_STOP_POSITION;
        }

        // Run motor to requested position
        m_claw_lift.setTargetPosition(targetPosition); // Set target position
        m_claw_lift.setMode(DcMotor.RunMode.RUN_TO_POSITION); // Move the motor to the target position
        m_claw_lift.setPower(CLAW_LIFT_POWER_FACTOR); // Apply full power with power factor

        ElapsedTime runtime = new ElapsedTime();
        runtime.reset(); // Reset the timer before starting
        long timeout = SAFETY_TIMEOUT_MS; // Timeout value in seconds (for example, 5 seconds)

        // Wait until the motor reaches the target position or timeout
        while (m_claw_lift.isBusy() && runtime.milliseconds() < timeout) {
            // Update telemetry periodically to avoid excessive updates
            if (runtime.milliseconds() % 100 < 50) { // Update every 100ms
                telemetry.addData("Running to Position", "Target: %4d, Current: %4d",
                        targetPosition, m_claw_lift.getCurrentPosition());
                telemetry.update();
            }
        }

        // Report issue if timeout reached
        if (runtime.milliseconds() >= timeout) {
            telemetry.addData("Error", "Timeout reached while moving claw lift to position");
            telemetry.update();
        }
    }

    /**
     * Reports telemetry data about the claw lift
     */
    public void getTelemetry() {
        telemetry.addData("-----  CLAW LIFT", "  -----");
        telemetry.addData("Claw Lift", "Pos: %4d° | Target: %4d° | Factor: %.2f",
                m_claw_lift.getCurrentPosition(), targetPosition, CLAW_LIFT_POWER_FACTOR);
        telemetry.addData("Claw LIFT Power", m_claw_lift.getPower());
    }

}
