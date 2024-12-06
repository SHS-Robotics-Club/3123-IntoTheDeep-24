package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.MissionTimer;

/**
 * The Claw class defines all the claw components for a robot.
 *
 * Control Hub
 *     s_claw_l - Servo 0 port
 *     s_claw_r - Servo 1 port
 *
 * PUBLIC METHODS:
 *     Claw(hardwareMap, missonTimer) - constructor for instantiating a claw
 *     void init() - initializes the components of the claw
 *     void open() - opens claw (default)
 *     void close() - closes claw
 *     void reportTelemetry() - reports claw telemetry information
*      void update() - updates claw positions during move
 * PRIVATE METHODS:
 *     moveClaw(leftTarget, rightTarget) - configures claw when move requested
 *
 * VERSION   DATE     WHO  DETAIL
 * 00.01.00  24Nov24  SEB  Initial release
 * 00.01.01  28Nov24  SEB  Add try/catch in constructor. Check for null in init. Pass in telemetry.
 * 00.01.02  29Nov24  SEB  Adjusted claw positions. Verified claw motion softening.
 *
 */
public class Claw {

    // Left servo constants
    public static final double SERVO_LEFT_INIT_POSITION = 0.9;  // Claw position before START
    public static final double SERVO_LEFT_OPEN_POSITION = 0.62;  // Claw open position
    public static final double SERVO_LEFT_CLOSED_POSITION = 0.5;  // Power to motors before START
    // Right servo constants
    public static final double SERVO_RIGHT_INIT_POSITION = 0.72;  // Power to motors before START
    public static final double SERVO_RIGHT_OPEN_POSITION = 0.45;  // Power to motors before START
    public static final double SERVO_RIGHT_CLOSED_POSITION = 0.33;  // Maximum power to motors
    //
    public static final double CLAW_MOVE_DURATION_MS = 500;

    // Declare claw servo instances
    private Servo s_claw_l;
    private Servo s_claw_r;

    // Shared timer and motion tracking
    private MissionTimer missionTimer;
    private Telemetry telemetry;

    private double targetLeftPosition;
    private double targetRightPosition;
    private double startLeftPosition;
    private double startRightPosition;
    private double moveStartTime; // Start time of the motion
    private boolean isMoving = false;

    /**
     * - Claw Constructor -
     * Instantiates all the claw components
     * @param hardwareMap the central store for hardware configuration
     * @param missionTimer a project shared resource
     */
    public Claw(HardwareMap hardwareMap, Telemetry telemetry, MissionTimer missionTimer) {

        // Instantiate shared resource
        this.missionTimer = missionTimer;
        this.telemetry = telemetry;

        try {
            // Instantiate two servos for the claw
            this.s_claw_l = hardwareMap.get(Servo.class, "s_claw_l");
            this.s_claw_r = hardwareMap.get(Servo.class, "s_claw_r");
        } catch (Exception e) {
            telemetry.addData("Error", "Claw initialization failed: " + e.getMessage());
            telemetry.update();
        }
    }

    /**
     * Initializes all claw components
     * Servo directions are set as opposing and in the middle position
     */
    public void init() {

        if (s_claw_l != null && s_claw_r != null) {
            // Initialize left servo
            s_claw_l.setDirection(Servo.Direction.FORWARD);
            s_claw_l.setPosition(SERVO_LEFT_INIT_POSITION);
            // Initialize right servo
            s_claw_r.setDirection(Servo.Direction.REVERSE);
            s_claw_r.setPosition(SERVO_RIGHT_INIT_POSITION);
        } else {
            telemetry.addData("Error", "Claw servos are not initialized.");
        }
    }

    /**
     * Adjusts power levels for X, Y and rotate, then applies them to the motors.
     *
     *
     */
    public void open() {

        // Set claw servos to open positions
        moveClaw(SERVO_LEFT_OPEN_POSITION, SERVO_RIGHT_OPEN_POSITION);
    }

    /**
     * Adjusts power levels for X, Y and rotate, then applies them to the motors.
     * *
     */
    public void close() {

        // Set claw servos to open positions
        moveClaw(SERVO_LEFT_CLOSED_POSITION, SERVO_RIGHT_CLOSED_POSITION);
    }

    /**
     * Reports the current encoder position and power level for each motor.
     *
    */
    public void reportTelemetry() {

        // Send motor data as telemetry data
        telemetry.addData("-----  CLAW", "  -----");
        telemetry.addData("CLAW STATUS: ", "Left Position: %.2f, Right Position: %.2f, Moving: %b",
                s_claw_l.getPosition(), s_claw_r.getPosition(), isMoving);
    }

    /**
     * Starts a smooth move of the servos to the target positions.
     * @param leftTarget The target position for the left servo
     * @param rightTarget The target position for the right servo
     */
    private void moveClaw(double leftTarget, double rightTarget) {
        startLeftPosition = s_claw_l.getPosition();
        startRightPosition = s_claw_r.getPosition();
        targetLeftPosition = leftTarget;
        targetRightPosition = rightTarget;
        moveStartTime = missionTimer.getTimeMS(); // Capture the start time
        isMoving = true;
    }

    /**
     * Updates the positions of the servos during smooth movement.
     * Call this method periodically in the loop to ensure smooth motion.
     */
    public void update() {
        if (isMoving) {
            // Check if the movement duration has elapsed
            if (missionTimer.hasElapsed(moveStartTime + CLAW_MOVE_DURATION_MS)) {
                // Movement complete
                s_claw_l.setPosition(targetLeftPosition);
                s_claw_r.setPosition(targetRightPosition);
                isMoving = false;
            } else {
                // Smooth interpolation between start and target positions
                double elapsed = missionTimer.getTimeMS() - moveStartTime;
                double progress = elapsed / CLAW_MOVE_DURATION_MS;
                double newLeftPosition = startLeftPosition + (targetLeftPosition - startLeftPosition) * progress;
                double newRightPosition = startRightPosition + (targetRightPosition - startRightPosition) * progress;
                s_claw_l.setPosition(newLeftPosition);
                s_claw_r.setPosition(newRightPosition);
            }
        }
    }
}
