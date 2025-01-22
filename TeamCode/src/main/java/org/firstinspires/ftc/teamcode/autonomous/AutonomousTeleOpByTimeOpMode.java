package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * The AutonomousTeleOpByTimeOpMode class teleop is a first attempt at autonomous operation.
 * It extends OpMode and uses the methods available to it.
 *
 * VERSION   DATE     WHO  DETAIL
 * 00.01.00  22Jan25  SEB  Initial release
 *
 */
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Autonomous TeleOp by Time (OpMode)", group = "Autonomous")
public class AutonomousTeleOpByTimeOpMode extends OpMode {
    private DcMotor leftFront, rightFront, leftRear, rightRear;

    private long endTime = 0;
    private boolean isMoving = false;
    private double leftFrontPower = 0;
    private double rightFrontPower = 0;
    private double leftRearPower = 0;
    private double rightRearPower = 0;
    private int step = 0;

    @Override
    public void init() {
        // Initialize motors
        leftFront = hardwareMap.get(DcMotor.class, "fL");
        rightFront = hardwareMap.get(DcMotor.class, "fR");
        leftRear = hardwareMap.get(DcMotor.class, "bL");
        rightRear = hardwareMap.get(DcMotor.class, "bR");

        // Set motor directions
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() {
        telemetry.addData("Status", "Waiting for Start");
    }

    @Override
    public void start() {
        telemetry.addData("Status", "Running Autonomous TeleOp");
        step = 0;
    }

    @Override
    public void loop() {
        if (!isMoving) {
            switch (step) {
                case 0:
                    runForwardTime(1.0, 1.0); // Move forward at full speed for 1 second
                    step++;
                    break;
                case 1:
                    strafeLeftTime(1.0, 5.0); // Strafe left at full speed for 5 seconds
                    step++;
                    break;
                case 2:
                    runReverseTime(1.0, 1.0); // Move backward at full speed for 1 second
                    step++;
                    break;
                default:
                    stopMotors();
                    telemetry.addData("Status", "Autonomous Complete");
                    break;
            }
        }

        if (isMoving && System.currentTimeMillis() >= endTime) {
            stopMotors();
            isMoving = false;
        }

        telemetry.addData("Status", isMoving ? "Moving" : "Idle");
        telemetry.addData("Step", step);
    }

    private void runForwardTime(double power, double seconds) {
        setMotorPower(power, power, power, power);
        scheduleMovement(seconds);
    }

    private void runReverseTime(double power, double seconds) {
        setMotorPower(-power, -power, -power, -power);
        scheduleMovement(seconds);
    }

    private void strafeLeftTime(double power, double seconds) {
        setMotorPower(-power, power, power, -power);
        scheduleMovement(seconds);
    }

    private void strafeRightTime(double power, double seconds) {
        setMotorPower(power, -power, -power, power);
        scheduleMovement(seconds);
    }

    private void setMotorPower(double leftFrontPower, double rightFrontPower, double leftRearPower, double rightRearPower) {
        this.leftFrontPower = leftFrontPower;
        this.rightFrontPower = rightFrontPower;
        this.leftRearPower = leftRearPower;
        this.rightRearPower = rightRearPower;

        leftFront.setPower(leftFrontPower);
        rightFront.setPower(rightFrontPower);
        leftRear.setPower(leftRearPower);
        rightRear.setPower(rightRearPower);
    }

    private void stopMotors() {
        setMotorPower(0, 0, 0, 0);
    }

    private void scheduleMovement(double seconds) {
        endTime = System.currentTimeMillis() + (long) (seconds * 1000);
        isMoving = true;
    }
}
