package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * The Drivetrain class defines all the drivetrain components for a robot.
 *
 * PUBLIC METHODS:
 *     Drivetrain(hardwareMap) - constructor for instantiating a drivetrain
 *     void init() - initializes the components of the drivetrain
 *     void reportTelemetry() - reports drivetrain telemetry information
 *     void operate(powerX, powerY, powerRotate, powerFactor) - applies power to the motors
 *         [based on video "FTC Programming 9 of n: Deadline & Toggle Drive Power" by Brandon Pacewic
 *          see https://www.youtube.com/watch?v=06dOk1S6oEg]
 *
 * VERSION   DATE     WHO  DETAIL
 * 00.01.00  11Nov24  SEB  Initial release
 * 00.01.01  24Nov24  SEB  Minor update.
 * 00.01.02  28Nov24  SEB  Add try/catch to constructor. Add null check to init. Pass in telemetry.
 * 00.01.03  29Nov24  SEB  Reverse motor directions (and added minus sign '-' in TeleOp) to fix
 *                         control directions.
 * 00.02.00  02Jan25  SEB  Reversed direction of all four motors due to replacing chains with
 *                         bevel gears.
 *
 */
public class Drivetrain {

    // Constants
    public static final double MOTOR_POWER_ZERO = 0.0;  // Power to motors before START
    public static final double ABSOLUTE_MAX_POWER = 1.0;  // Maximum power to motors

    // Declare drivetrain components (null) - do not instantiate here
    private DcMotor fL;
    private DcMotor fR;
    private DcMotor bL;
    private DcMotor bR;
    private double powerMax;
    // Telemetry object
    private Telemetry telemetry;

    /**
     * - Drivetrain Constructor -
     * Instantiates all the drivetrain components
     * @param hardwareMap the central store for hardware configuration
     */
    public Drivetrain(HardwareMap hardwareMap, Telemetry telemetry) {

        this.telemetry = telemetry;

        try {
            // Instantiate all four motors as DcMotor class and use the configuration
            // to connect the port to each existing motor
            fL = hardwareMap.get(DcMotor.class, "fL");
            fR = hardwareMap.get(DcMotor.class, "fR");
            bL = hardwareMap.get(DcMotor.class, "bL");
            bR = hardwareMap.get(DcMotor.class, "bR");

            // Define motor directions - typically left motors are forward but not always!
            fL.setDirection(DcMotorSimple.Direction.FORWARD);
            bL.setDirection(DcMotorSimple.Direction.FORWARD);
            fR.setDirection(DcMotorSimple.Direction.REVERSE);
            bR.setDirection(DcMotorSimple.Direction.REVERSE);
        } catch (Exception e) {
            telemetry.addData("Error", "Drivetrain initialization failed: " + e.getMessage());
            telemetry.update();
        }
    }

    /**
     * Initializes all drivetrain subsystems.
     * Motors are set to zero power and power is based directly from setPower (no PID controller).
     */
    public void init() {

        if (fL != null && fR != null && bL != null && bR != null) {
            // Initialize motors to brake applies without encoders
            fL.setPower(MOTOR_POWER_ZERO);  // SAFETY: Make sure motor is set to zero
            fL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            fL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            fL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);  // Encoder data collected but no PID
            fR.setPower(MOTOR_POWER_ZERO);  // SAFETY: Make sure motor is set to zero
            fR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            fR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            fR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);  // Encoder data collected but no PID
            bL.setPower(MOTOR_POWER_ZERO);  // SAFETY: Make sure motor is set to zero
            bL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            bL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            bL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);  // Encoder data collected but no PID
            bR.setPower(MOTOR_POWER_ZERO);  // SAFETY: Make sure motor is set to zero
            bR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            bR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            bR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);  // Encoder data collected but no PID
        } else {
            telemetry.addData("Error", "Drivetrain motors are not initialized.");
        }

    }

    /**
     * Reports the current encoder position and power level for each motor.
     *
     */
    public void reportTelemetry() {

        // Send motor data as telemetry data
        telemetry.addData("-----  DRIVETRAIN", "  -----");
        telemetry.addData("mFrontLeft", "Encoder: %2d, Power: %.2f",
                fL.getCurrentPosition(), fL.getPower());
        telemetry.addData("mBackLeft", "Encoder: %2d, Power: %.2f",
                bL.getCurrentPosition(), bL.getPower());
        telemetry.addData("mFrontRight", "Encoder: %2d, Power: %.2f",
                fR.getCurrentPosition(), fR.getPower());
        telemetry.addData("mBackRight", "Encoder: %2d, Power: %.2f",
                bR.getCurrentPosition(), bR.getPower());
    }

    /**
     * Adjusts power levels for X, Y and rotate, then applies them to the motors.
     *
     * @param powerX requested power in the X direction
     * @param powerY requested power in the Y direction
     * @param powerRotate requested power for rotation
     * @param powerFactor current power factor to reduce applied power
     */
    public void operate(double powerX, double powerY, double powerRotate, double powerFactor) {

        // Calculate maximum power and trim to a value of 1.0 as needed
        powerMax = Math.max( Math.abs(powerX) + Math.abs(powerY) + Math.abs(powerRotate), ABSOLUTE_MAX_POWER);
        // Set drivetrain power to left side motors
        fL.setPower( ((powerY + powerX + powerRotate) / powerMax) * powerFactor );
        bL.setPower( ((powerY - powerX + powerRotate) / powerMax) * powerFactor );
        // Set drivetrain power to right side motors
        fR.setPower( ((powerY - powerX - powerRotate) / powerMax) * powerFactor );
        bR.setPower( ((powerY + powerX - powerRotate) / powerMax) * powerFactor );
    }
}
