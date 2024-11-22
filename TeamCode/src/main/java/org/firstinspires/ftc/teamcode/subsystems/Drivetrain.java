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
 *
 */
public class Drivetrain {

    // Constants
    public static final double MOTOR_POWER_ZERO = 0.0;  // Power to motors before START
    public static final double ABSOLUTE_MAX_POWER = 1.0;  // Maximum power to motors

    // Declare drivetrain components (null) - do not instantiate here
    private DcMotor m_fl;
    private DcMotor m_fr;
    private DcMotor m_bl;
    private DcMotor m_br;
    private double powerMax;

    /**
     * - Drivetrain Constructor -
     * Instantiates all the drivetrain components
     * @param hardwareMap the central store for hardware configuration
     */
    public Drivetrain(HardwareMap hardwareMap) {

        // Instantiate all four motors as DcMotor class and use the configuration
        // to connect the port to each existing motor
        m_fl = hardwareMap.get(DcMotor.class, "m_fl");
        m_fr = hardwareMap.get(DcMotor.class, "m_fl");
        m_bl = hardwareMap.get(DcMotor.class, "m_fl");
        m_br = hardwareMap.get(DcMotor.class, "m_fl");

        // Define motor directions - typically left motors are forward but not always!
        m_fl.setDirection(DcMotorSimple.Direction.FORWARD);
        m_bl.setDirection(DcMotorSimple.Direction.FORWARD);
        m_fr.setDirection(DcMotorSimple.Direction.REVERSE);
        m_br.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    /**
     * Initializes all drivetrain subsystems.
     * Motors are set to zero power and power is based directly from setPower (no PID controller).
     */
    public void init() {
        // Initialize motors to brake applies without encoders
        m_fl.setPower(MOTOR_POWER_ZERO);  // SAFETY: Make sure motor is set to zero
        m_fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m_fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m_fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);  // Encoder data collected but no PID
        m_fr.setPower(MOTOR_POWER_ZERO);  // SAFETY: Make sure motor is set to zero
        m_fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m_fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m_fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);  // Encoder data collected but no PID
        m_bl.setPower(MOTOR_POWER_ZERO);  // SAFETY: Make sure motor is set to zero
        m_bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m_bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m_bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);  // Encoder data collected but no PID
        m_br.setPower(MOTOR_POWER_ZERO);  // SAFETY: Make sure motor is set to zero
        m_br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m_br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m_br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);  // Encoder data collected but no PID
    }

    /**
     * Reports the current encoder position and power level for each motor.
     *
     */
    public void reportTelemetry() {

        // Send motor data as telemetry data
        telemetry.addData("mFrontLeft", "Encoder: %2d, Power: %.2f",
                m_fl.getCurrentPosition(), m_fl.getPower());
        telemetry.addData("mBackLeft", "Encoder: %2d, Power: %.2f",
                m_bl.getCurrentPosition(), m_bl.getPower());
        telemetry.addData("mFrontRight", "Encoder: %2d, Power: %.2f",
                m_fr.getCurrentPosition(), m_fr.getPower());
        telemetry.addData("mBackRight", "Encoder: %2d, Power: %.2f",
                m_br.getCurrentPosition(), m_br.getPower());
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
        m_fl.setPower( ((powerY + powerX + powerRotate) / powerMax) * powerFactor );
        m_bl.setPower( ((powerY - powerX + powerRotate) / powerMax) * powerFactor );
        // Set drivetrain power to right side motors
        m_fl.setPower( ((powerY - powerX - powerRotate) / powerMax) * powerFactor );
        m_fl.setPower( ((powerY + powerX - powerRotate) / powerMax) * powerFactor );

    }
}
