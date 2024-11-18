package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.drivetrain.MechDrive;
import org.firstinspires.ftc.teamcode.roadrunner.Drawing;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Climb;
import org.firstinspires.ftc.teamcode.subsystems.Imu;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.ThreeDeadWheelLocalizer;
import org.firstinspires.ftc.teamcode.utils.DriverHubHelp;
import org.firstinspires.ftc.teamcode.utils.GamepadEvents;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name ="Aurabot TeleOp")
public class TeleOp extends LinearOpMode {
    private GamepadEvents controller;
    private MechDrive robot;
    private Limelight limelight;
    private Imu imu;
    private ThreeDeadWheelLocalizer deadwheels;
    private DriverHubHelp screen;
    private Arm arm;
    private double armPos;
    private Claw claw;
    private double clawPos;
    private Lift lift;
    private double liftPower;
    boolean reverseArm;
    private double[] armPositions = {0.6, 0.3, 0.1};
    private int armPositionIndex = 0;
    private boolean isReversing = false;
    private Climb climb;
    private boolean fieldCentric;

    public void runOpMode() throws InterruptedException{

        controller = new GamepadEvents(gamepad1);
        robot = new MechDrive(hardwareMap);
        limelight = new Limelight(hardwareMap);
//        imu = new Imu(hardwareMap);
        IMU imu = hardwareMap.get(IMU.class, "imu");
        screen = new DriverHubHelp();
        deadwheels = new ThreeDeadWheelLocalizer(hardwareMap);
        arm = new Arm(hardwareMap);
        deadwheels = new ThreeDeadWheelLocalizer(hardwareMap);
        arm = new Arm(hardwareMap);
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        claw = new Claw(hardwareMap);
        lift = new Lift(hardwareMap, "lift", "lift");
        reverseArm = false;
        climb = new Climb(hardwareMap,"climb");
        fieldCentric = false;
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        ));
        imu.initialize(parameters);
        waitForStart();
        while(opModeIsActive())
        {
            if(controller.y.onPress())
            {
                fieldCentric = !fieldCentric;
            }
            if(fieldCentric)
            {
                telemetry.addData("Field Centric: ", fieldCentric);
                double forward = controller.left_stick_y;
                double strafe = controller.left_stick_x;
                double rotate = -controller.right_stick_x;
                clawPos = 0.6;
                YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();

                //drive and limelight
                String[] distance =limelight.getDistanceInInches();
                telemetry.addData("Limelight Distance: ", distance[0] + ", " + distance[1]);
                drive.updatePoseEstimate();
                robot.fieldCentricDrive(forward, strafe, rotate);
                telemetry.addData("x", screen.roundData(drive.pose.position.x));
                telemetry.addData("y", screen.roundData(drive.pose.position.y));
                telemetry.addData("Yaw (deg)", screen.roundData(Math.toDegrees(drive.pose.heading.toDouble())));


                //arm
                if(controller.right_bumper.onPress())
                {
                    if(!isReversing)
                    {
                        armPos = armPositions[armPositionIndex];
                        arm.setPosition(armPos);
                        armPositionIndex++;
                        if(armPositionIndex >= armPositions.length)
                        {
                            armPositionIndex--;
                            isReversing = true;
                        }
                    }else {
                        armPos = armPositions[armPositionIndex];
                        arm.setPosition(armPos);
                        armPositionIndex--;
                        if(armPositionIndex < 0)
                        {
                            armPositionIndex++;
                            isReversing = false;
                        }
                    }


                }
                if(controller.left_bumper.onPress())
                {

                    armPos = armPositions[armPositionIndex];
                    arm.setPosition(armPos);
                    armPositionIndex--;
                    if(armPositionIndex < 0)
                    {
                        armPositionIndex = armPositions.length-1;
                    }

                }
                telemetry.addData("Arm Position", armPos);


                //claw
                if (controller.a.onPress()) {
                    claw.release();
                } else if (controller.b.onPress()) {
                    claw.close(clawPos);
                }
                telemetry.addData("Claw Position", clawPos);


                //lift
                liftPower = controller.right_trigger.getTriggerValue() - controller.left_trigger.getTriggerValue();
                lift.moveLift(liftPower);
                telemetry.addData("Lift Power", liftPower);

                //climb

                //           climb.moveClimb(controller.right_trigger.getTriggerValue() - controller.left_trigger.getTriggerValue());
//            if(controller.dpad_up.onPress())
//            {
//                climb.moveClimb(0.7);
//            }else if(controller.dpad_down.onPress())
//            {
//                climb.moveClimb(0);
//            }
                telemetry.update();
                controller.update();
            }

            telemetry.addData("Field Centric: ", fieldCentric);
            double forward = controller.left_stick_y;
            double strafe = controller.left_stick_x;
            double rotate = -controller.right_stick_x;
            clawPos = 0.6;
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();

            //drive and limelight
            String[] distance =limelight.getDistanceInInches();
            telemetry.addData("Limelight Distance: ", distance[0] + ", " + distance[1]);
            drive.updatePoseEstimate();
            robot.drive(forward, strafe, rotate);
            telemetry.addData("x", screen.roundData(drive.pose.position.x));
            telemetry.addData("y", screen.roundData(drive.pose.position.y));
            telemetry.addData("Yaw (deg)", screen.roundData(Math.toDegrees(drive.pose.heading.toDouble())));


            //arm
            if(controller.right_bumper.onPress())
            {
                if(!isReversing)
                {
                    armPos = armPositions[armPositionIndex];
                    arm.setPosition(armPos);
                    armPositionIndex++;
                    if(armPositionIndex >= armPositions.length)
                    {
                        armPositionIndex--;
                        isReversing = true;
                    }
                }else {
                    armPos = armPositions[armPositionIndex];
                    arm.setPosition(armPos);
                    armPositionIndex--;
                    if(armPositionIndex < 0)
                    {
                        armPositionIndex++;
                        isReversing = false;
                    }
                }


            }
            if(controller.left_bumper.onPress())
            {

                armPos = armPositions[armPositionIndex];
                arm.setPosition(armPos);
                armPositionIndex--;
                if(armPositionIndex < 0)
                {
                    armPositionIndex = armPositions.length-1;
                }

            }
            telemetry.addData("Arm Position", armPos);


            //claw
            if (controller.a.onPress()) {
                claw.release();
            } else if (controller.b.onPress()) {
                claw.close(clawPos);
            }
            telemetry.addData("Claw Position", clawPos);


            //lift
            liftPower = controller.right_trigger.getTriggerValue() - controller.left_trigger.getTriggerValue();
            lift.moveLift(liftPower);
            telemetry.addData("Lift Power", liftPower);

            //climb

 //           climb.moveClimb(controller.right_trigger.getTriggerValue() - controller.left_trigger.getTriggerValue());
//            if(controller.dpad_up.onPress())
//            {
//                climb.moveClimb(0.7);
//            }else if(controller.dpad_down.onPress())
//            {
//                climb.moveClimb(0);
//            }
            telemetry.update();
            controller.update();
        }


    }

}
