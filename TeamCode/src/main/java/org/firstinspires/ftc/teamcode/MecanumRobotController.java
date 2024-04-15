// This is the robot controller class. This class will contain all methods needed to control the robot
// so you can just instantiate this and use it in other files. Some examples include driving,
// getting the current heading, and moving any other motors on the robot.

// Remember to comment with B.E.R.P:
//      B: Behavior, what the method does.
//      E: Exceptions, what exceptions the method throws.
//      R: Return, what the method returns.
//      P: Parameters, the list of parameters and explanations of them.

package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


public class MecanumRobotController {
    public static final boolean DEFAULT_FIELD_CENTRIC = true;
    public static final boolean DEFAULT_SEND_TELEMETRY = true;
    // Need to find this.
    public static final double FORWARD_COUNTS_PER_INCH = 42.59;
    public static final double STRAFE_COUNTS_PER_INCH = 51.0;
    public static final double HEADING_CORRECTION_POWER = 1.0;
    public static final double MAX_CORRECTION_ERROR = 2.0;
    public static final double Kp = 0.05;
    public static final double Kd = 0.0005;
    public static final double Ki = 0.000005;

    private final DcMotor backLeft;
    private final DcMotor backRight;
    private final DcMotor frontLeft;
    private final DcMotor frontRight;
    private final IMU gyro;
    private final LinearOpMode robot;
    private final ElapsedTime runtime;

    private double wantedHeading;
    private double currentForward;
    private double currentStrafe;
    private double currentTurn;
    private double currentAngularVelocity;
    private double lastHeading;

    private double integralSum;
    private double lastError;
    private final ElapsedTime PIDTimer;
    private final ElapsedTime AngularVelocityTimer;

    // Create the controller with all the motors needed to control the robot. If another motor,
    // servo, or sensor is added, put that in here so the class can access it.
    public MecanumRobotController(DcMotor backLeft, DcMotor backRight,
                                  DcMotor frontLeft, DcMotor frontRight,
                                  IMU gyro, LinearOpMode robot) {
        this.backLeft = backLeft;
        this.backRight = backRight;
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;

        this.gyro = gyro;
        this.robot = robot;
        this.wantedHeading = getAngleImuDegrees();

        this.runtime = new ElapsedTime();
        this.runtime.reset();

        this.PIDTimer = new ElapsedTime();
        this.AngularVelocityTimer = new ElapsedTime();
        this.runtime.reset();
    }

    // Overloaded constructor to create the robot controller without a LinearOpMode.
    // You cannot use distanceDrive or turnTo without a LinearOpMode.
    public MecanumRobotController(DcMotor backLeft, DcMotor backRight,
                                  DcMotor frontLeft, DcMotor frontRight,
                                  IMU gyro) {
        this(backLeft, backRight, frontLeft, frontRight, gyro, null);
    }

    // Behavior: Moves the robot using a given forward, strafe, and turn power.
    // Params:
    //      - double forward: The forward power for the robot.
    //      - double strafe: The strafe power for the robot.
    //      - double turn: The turn power fo the robot.
    //      - double headingCorrectionPower: The speed of heading correction.
    //      - boolean isTelemetry: Whether or not this method will send telemetry.
    private void move(double forward, double strafe, double turn, double headingCorrectionPower,
                      boolean isTelemetry) {
        double currentHeading = getAngleImuDegrees();
        currentAngularVelocity = Math.abs(normalize(currentHeading - lastHeading) / Math.max(AngularVelocityTimer.seconds(), 1));
        AngularVelocityTimer.reset();
        if (currentAngularVelocity > 4 || turn != 0) {
            wantedHeading = currentHeading;
        } else {
            double error = normalize(wantedHeading - currentHeading);
            double derivative = (error - lastError) / PIDTimer.seconds();
            integralSum += error * runtime.seconds();

            double headingCorrection = -((Kp * error) + (Kd * derivative) + (Ki * integralSum));
            headingCorrection = normalize(headingCorrection);
            lastError = error;
            PIDTimer.reset();

            turn = headingCorrectionPower * headingCorrection;
        }

        // Set fields so the robot knows what its current forward, strafe, and turn is in other methods.
        currentForward = forward;
        currentStrafe = strafe;
        currentTurn = turn;
        lastHeading = currentHeading;

        if (isTelemetry && robot != null) {
            sendTelemetry();
        }

        double backLeftPower = Range.clip((forward - strafe - turn) / 3, -1.0, 1.0);
        double backRightPower = Range.clip((forward - strafe + turn) / 3, -1.0, 1.0);
        double frontLeftPower = Range.clip((forward + strafe - turn) / 3, -1.0, 1.0);
        double frontRightPower = Range.clip((forward + strafe + turn) / 3, -1.0, 1.0);

        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);
        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
    }

    // Behavior: Overloaded method of move. This sets the default of isTelemetry.
    // Params:
    //      - double forward: The forward power for the robot.
    //      - double strafe: The strafe power for the robot.
    //      - double turn: The turn power fo the robot.
    //      - double headingCorrectionPower: The speed of heading correction.
    private void move(double forward, double strafe, double turn, double headingCorrectionPower) {
        move(forward, strafe, turn, headingCorrectionPower, DEFAULT_SEND_TELEMETRY);
    }

    // TODO: Find exact values for distance and implement it in COUNTS_PER_INCH to make this method precise.
    // Behavior: Drives the robot a given distance in a given direction without turning it.
    // Exceptions:
    //      - Throws RuntimeException when a LinearOpMode object was not provided in the constructor.
    // Params:
    //      - double distance: The distance to drive the robot in inches.
    //      - double direction: The direction, in degrees, that the robot will drive in. This is
    //                          based on the direction the robot was initialized in.
    //      - double speed: The speed at which the robot will move.
    //      - boolean isFieldCentric: Determines whether the direction is based from the robot
    //                                or the field. If its field centric, the robot will always
    //                                move the same direction for the same inputted direction,
    //                                no matter what direction the robot is facing.
    public void distanceDrive(double distance, double direction, double speed, boolean isFieldCentric)
            throws RuntimeException {
        if (robot == null) {
            throw new RuntimeException("Tried to run distanceDrive but LinearOpMode object not given!");
        }
        double currentHeading = getAngleImuDegrees();
        // This still needs testing.
        double forward;
        double strafe;
        if (isFieldCentric) {
            forward = Math.cos((direction - currentHeading) * (Math.PI / 180));
            strafe = Math.sin((direction - currentHeading) * (Math.PI / 180));
        } else {
            forward = Math.cos(direction * (Math.PI / 180));
            strafe = Math.sin(direction * (Math.PI / 180));
        }

        int forwardCounts = (int)(forward * distance * FORWARD_COUNTS_PER_INCH);
        int strafeCounts = (int)(strafe * distance * STRAFE_COUNTS_PER_INCH);

        int backLeftTarget = backLeft.getCurrentPosition() - forwardCounts + strafeCounts;
        int backRightTarget = backRight.getCurrentPosition() - forwardCounts + strafeCounts;
        int frontLeftTarget = frontLeft.getCurrentPosition() - forwardCounts - strafeCounts;
        int frontRightTarget = frontRight.getCurrentPosition() - forwardCounts - strafeCounts;

        backLeft.setTargetPosition(backLeftTarget);
        backRight.setTargetPosition(backRightTarget);
        frontLeft.setTargetPosition(frontLeftTarget);
        frontRight.setTargetPosition(frontRightTarget);

        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        double correctionPower = HEADING_CORRECTION_POWER;
        if (Math.abs(direction - currentHeading) < 90) correctionPower *= -1;

        while ((backLeft.isBusy() || backRight.isBusy() || frontLeft.isBusy() || frontRight.isBusy())
                && robot.opModeIsActive()) {
            move(speed, 0.0, 0.0, correctionPower);
        }

        // Stop movement and switch modes
        move(0, 0, 0, 0);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // Behavior: Overloaded method of distanceDrive. This sets the default of isFieldCentric.
    // Exceptions:
    //      - Throws RuntimeException when a LinearOpMode object was not provided in the constructor.
    // Params:
    //      - double distance: The distance to drive the robot in inches.
    //      - double direction: The direction, in degrees, that the robot will drive in. This is
    //                          based on the direction the robot was initialized in.
    //      - double speed: The speed at which the robot will move.
    public void distanceDrive(double distance, double direction, double speed) throws RuntimeException {
        distanceDrive(distance, direction, speed, DEFAULT_FIELD_CENTRIC);
    }

    // Behavior: Drives the robot continuously based on forward, strafe, and turn power.
    // Params:
    //      - double forwardPower: The power at which the robot will move forward.
    //      - double strafePower: The power at which the robot will strafe.
    //      - double turn: The power at which the robot will turn.
    //      - boolean isFieldCentric: Determines whether the forward direction is based on the
    //                                direction of the robot, or the direction the robot was
    //                                initialized in.
    public void continuousDrive(double forwardPower, double strafePower, double turn, boolean isFieldCentric) {
        double currentHeading = getAngleImuDegrees();
        double forward;
        double strafe;
        if (isFieldCentric) {
            forward = ((forwardPower * Math.cos(currentHeading * (Math.PI / 180))) +
                    (strafePower * Math.sin(currentHeading * (Math.PI / 180))));
            strafe = ((forwardPower * Math.sin(currentHeading * (Math.PI / 180))) -
                    (strafePower * Math.cos(currentHeading * (Math.PI / 180))));
        } else {
            forward = forwardPower;
            strafe = strafePower;
        }

        move(forward, strafe, turn, HEADING_CORRECTION_POWER);
    }

    // Behavior: Overloaded method of continuousDrive. This sets the default of isFieldCentric.
    // Params:
    //      - double forwardPower: The power at which the robot will move forward.
    //      - double strafePower: The power at which the robot will strafe.
    //      - double turn: The power at which the robot will turn.
    public void continuousDrive(double forwardPower, double strafePower, double turn) {
        continuousDrive(forwardPower, strafePower, turn, DEFAULT_FIELD_CENTRIC);
    }

    // Behavior: Turns the robot to a given angle
    // Params:
    //      - double degrees: The angle to turn the robot to in degrees.
    //      - double speed: The speed at which the robot should turn.
    public void turnTo(double angle, double speed) {
        wantedHeading = angle;
        while (Math.abs(getAngleImuDegrees() - wantedHeading) > MAX_CORRECTION_ERROR && robot.opModeIsActive()) {
            move(0, 0, 0, speed);
        }
        // Stop the robot
        move(0, 0, 0, 0);
    }

    // Behavior: Normalizes a given degree value to the range (-180, 180]
    // Returns: The normalized degrees as a double.
    // Params:
    //      - double degrees: The degrees to be normalized.
    public static double normalize(double degrees) {
        double normalizedAngle = degrees;
        while (normalizedAngle > 180) normalizedAngle -= 360;
        while (normalizedAngle <= -180) normalizedAngle += 360;
        return normalizedAngle;
    }

    // Behavior: Get the current IMU heading in degrees.
    // Returns: The current robots angle in degrees on the range (-180, 180]
    public double getAngleImuDegrees() {
        return normalize(
                gyro.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
    }

    // Behavior: Sends various information to the telemetry to be read. The information sent is
    //           the strafe movement of the robot, the forward movement of the robot, the turn
    //           movement of the robot, the wanted heading of the robot, the current heading
    //           of the robot, and the current runtime.
    // Params:
    //      - Telemetry telemetry: The telemetry to send the information to.
    public void sendTelemetry(Telemetry telemetry) {
        telemetry.addData("Forward", currentForward);
        telemetry.addData("Strafe", currentStrafe);
        telemetry.addData("Turn", currentTurn);
        telemetry.addData("", "");
        telemetry.addData("Current Heading", getAngleImuDegrees());
        telemetry.addData("Wanted Heading", wantedHeading);
        telemetry.addData("", "");
        telemetry.addData("Runtime", runtime.seconds());
        telemetry.addData("Angular Velocity", currentAngularVelocity);

        telemetry.update();
    }

    // Behavior: Overloaded method of sendTelemetry() that sends the information to the provided
    //           LinearOpMode's telemetry.
    // Exceptions:
    //      - Throws RuntimeException when a LinearOpMode object was not provided in the constructor.
    public void sendTelemetry() throws RuntimeException {
        if (robot == null) {
            throw new RuntimeException(
                    "Tried to run sendTelemetry (no parameters) but LinearOpMode object not given!");
        }

        sendTelemetry(robot.telemetry);
    }

    // Behavior: Stops all movement except heading correction for the given duration of time.
    // Exceptions:
    //      - Throws RuntimeException when a LinearOpMode object was not provided in the constructor.
    // Params:
    //      - double time: The amount of time the robot should stop for in seconds.
    public void sleep(double time) throws RuntimeException {
        if (robot == null) {
            throw new RuntimeException("Tried to run sleep but LinearOpMode object not given!");
        }

        double startTime = runtime.seconds();
        while (robot.opModeIsActive() && (runtime.seconds() - startTime) < time) {
            move(0, 0, 0, HEADING_CORRECTION_POWER);
        }

        // Stop the robot
        move(0, 0, 0, 0);
    }
}