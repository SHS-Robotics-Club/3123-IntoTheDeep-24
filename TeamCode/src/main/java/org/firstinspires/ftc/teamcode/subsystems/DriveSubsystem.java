package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.bosch.BHI260IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import java.lang.Math;

import org.firstinspires.ftc.teamcode.*;

//TODO: Test odometry and insert correct values for odometry constants
//TODO: Modify drive functions to make use of new odometry logic rather than IMU

//TODO: Fix deadzone logic to use quadratic equation

public class DriveSubsystem extends SubsystemBase {

    private HardwareMap hardwareMap;

    private final DcMotorEx frontLeftMotor;
    private final DcMotorEx frontRightMotor;
    private final DcMotorEx backLeftMotor;
    private final DcMotorEx backRightMotor;

    private final BHI260IMU imu;

    private double speedMultiplier = 1.0;
    private boolean fieldCentric = true;

    public DriveSubsystem(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        
        imu = hardwareMap.get(BHI260IMU.class, Constants.DriveConstants.IMU_NAME);
        imu.initialize(Constants.DriveConstants.IMU_PARAMETERS);

        frontLeftMotor = hardwareMap.get(DcMotorEx.class, Constants.DriveConstants.FRONT_LEFT_MOTOR_NAME);
        frontRightMotor = hardwareMap.get(DcMotorEx.class, Constants.DriveConstants.FRONT_RIGHT_MOTOR_NAME);
        backLeftMotor = hardwareMap.get(DcMotorEx.class, Constants.DriveConstants.BACK_LEFT_MOTOR_NAME);
        backRightMotor = hardwareMap.get(DcMotorEx.class, Constants.DriveConstants.BACK_RIGHT_MOTOR_NAME);

        frontLeftMotor.setDirection(Constants.DriveConstants.FRONT_LEFT_MOTOR_DIRECTION);
        frontRightMotor.setDirection(Constants.DriveConstants.FRONT_RIGHT_MOTOR_DIRECTION);
        backLeftMotor.setDirection(Constants.DriveConstants.BACK_LEFT_MOTOR_DIRECTION);
        backRightMotor.setDirection(Constants.DriveConstants.BACK_RIGHT_MOTOR_DIRECTION);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        resetGyro();
        resetEncoders();
    }
    
    public void drive(double forward, double strafe, double turn) {
        updatePose();
        if(fieldCentric) driveFieldCentric(forward, strafe, turn); else driveRobotCentric(forward, strafe, turn);
    }

    private void driveFieldCentric(double forward, double strafe, double turn) {
        turn = -turn;

        forward = Math.abs(forward) >= Constants.DriveConstants.DEADZONE ? forward : 0;
        strafe = Math.abs(strafe) >= Constants.DriveConstants.DEADZONE ? strafe : 0;
        turn = Math.abs(turn) >= Constants.DriveConstants.DEADZONE ? turn : 0;

        double gyroRadians = Math.toRadians(-getHeading());
        double fieldCentricStrafe = strafe * Math.cos(gyroRadians) - forward * Math.sin(gyroRadians);
        double fieldCentricDrive = strafe * Math.sin(gyroRadians) + forward * Math.cos(gyroRadians);

        frontLeftMotor.setPower(Range.clip((fieldCentricDrive + fieldCentricStrafe + turn), -1, 1) * speedMultiplier);
        frontRightMotor.setPower(Range.clip((fieldCentricDrive - fieldCentricStrafe - turn), -1, 1) * speedMultiplier);
        backLeftMotor.setPower(Range.clip((fieldCentricDrive - fieldCentricStrafe + turn), -1, 1) * speedMultiplier);
        backRightMotor.setPower(Range.clip((fieldCentricDrive + fieldCentricStrafe - turn), -1, 1) * speedMultiplier);
    }

    private void driveRobotCentric(double forward, double strafe, double turn) {
        turn = -turn;

        forward = Math.abs(forward) >= Constants.DriveConstants.DEADZONE ? forward : 0;
        strafe = Math.abs(strafe) >= Constants.DriveConstants.DEADZONE ? strafe : 0;
        turn = Math.abs(turn) >= Constants.DriveConstants.DEADZONE ? turn : 0;

        frontLeftMotor.setPower(Range.clip((forward + strafe + turn), -1, 1) * speedMultiplier);
        frontRightMotor.setPower(Range.clip((forward - strafe - turn), -1, 1) * speedMultiplier);
        backLeftMotor.setPower(Range.clip((forward - strafe + turn), -1, 1) * speedMultiplier);
        backRightMotor.setPower(Range.clip((forward + strafe - turn), -1, 1) * speedMultiplier);
    }

    public void setSpeedMultiplier(double multiplier) {
        speedMultiplier = Range.clip(multiplier, 0, 1);
    }

    public void resetGyro() {
        imu.resetYaw();
    }

    public void setFieldCentricOnOff(){
        fieldCentric = !fieldCentric; 
    }

    public void resetEncoders() {
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private double getHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

//Odometry Stuff

    /*
    Left Pod: frontLeftMotor
    Center Pod: frontRightMotor
    Right Pod: backLeftMotor
    */

    private class Pose {
        public double x;
        public double y;
        public double theta;

        public Pose(double x, double y, double theta) {
            this.x = x;
            this.y = y;
            this.theta = theta;
        }
    }

    private int[] lastTicks = new int[3];
    public Pose pose = new Pose(0, 0, 0);

    public Pose getPose() {
        return pose;
    }

    public void resetPose() {
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        pose.x = 0;
        pose.y = 0;
        pose.theta = 0;

        lastTicks[0] = frontLeftMotor.getCurrentPosition(); //Change later!
        lastTicks[1] = frontRightMotor.getCurrentPosition(); //Change later!
        lastTicks[2] = backLeftMotor.getCurrentPosition(); //Change later!
    }

    public void updatePose() {
        int[] ticks = new int[3];
        ticks[0] = frontLeftMotor.getCurrentPosition(); //Change later!
        ticks[1] = frontRightMotor.getCurrentPosition(); //Change later!
        ticks[2] = backLeftMotor.getCurrentPosition(); //Change later!

        int deltaLeft = ticks[0] - lastTicks[0];
        int deltaCenter = ticks[1] - lastTicks[1];
        int deltaRight = ticks[2] - lastTicks[2];
        lastTicks = ticks;

        double leftDist = -deltaLeft * Constants.OdometryConstants.WHEEL_CIRCUMFERENCE / Constants.OdometryConstants.TICKS_PER_REVOLUTION;
        double centerDist = -deltaCenter * Constants.OdometryConstants.WHEEL_CIRCUMFERENCE / Constants.OdometryConstants.TICKS_PER_REVOLUTION;
        double rightDist = deltaRight * Constants.OdometryConstants.WHEEL_CIRCUMFERENCE / Constants.OdometryConstants.TICKS_PER_REVOLUTION;
        double deltaTheta = (rightDist - leftDist) / Constants.OdometryConstants.WIDTH;

        double distY = (leftDist + rightDist) / 2;
        double distX = centerDist;
        double avgTheta = (pose.theta + deltaTheta) / 2; 

        double cos = Math.cos(avgTheta);
        double sin = Math.sin(avgTheta);

        pose.x += distX * sin + distY * cos;
        pose.y += -distX * cos + distY * sin;
        pose.theta = normalizeRadians(pose.theta + deltaTheta);
    }

    public static double normalizeRadians(double radians){
        double temp = (radians + Math.PI) / (2.0 * Math.PI);
        return (temp - Math.floor(temp) - 0.5)  * 2.0 * Math.PI;
    }

}