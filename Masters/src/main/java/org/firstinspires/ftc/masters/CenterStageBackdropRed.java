package org.firstinspires.ftc.masters;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.masters.drive.SampleMecanumDrive;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.Date;
import java.util.List;

@Config
@Autonomous(name = "Center Stage Backdrop Red", group = "competition")
public class CenterStageBackdropRed extends LinearOpMode {
    private OpenCvCamera webcam;

    private static final int CAMERA_WIDTH  = 640; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 360; // height of wanted camera resolution

    TelemetryPacket packet = new TelemetryPacket();

    enum State {
        PURPLE_DEPOSIT_PATH,
        PURPLE_DEPOSIT,

        UNTURN,
        BACKUP_FROM_SPIKES,

        YELLOW_DEPOSIT_PATH,
        YELLOW_DEPOSIT_STRAIGHT,
        YELLOW_DEPOSIT,
        BACK,
        PARK,
        STOP
    }

    int resetInt = 0;
    ElapsedTime depositTime = new ElapsedTime();
    SampleMecanumDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "frontWebcam"), cameraMonitorViewId);
        PropFindRight myPipeline;
        webcam.setPipeline(myPipeline = new PropFindRight(telemetry,packet));
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        PropFindRight.pos propPos = null;

        drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(new Vector2d(12, -58.5), Math.toRadians(90)); //Start position for roadrunner
        drive.setPoseEstimate(startPose);

        State currentState;

        Trajectory purpleDepositPathL = drive.trajectoryBuilder(startPose,false)
                .lineToSplineHeading(new Pose2d(new Vector2d(12, -32.5), Math.toRadians(150)))
                .build();

        Trajectory purpleDepositPathR = drive.trajectoryBuilder(startPose,false) // misses every time
                .lineToSplineHeading(new Pose2d(new Vector2d(13, -31), Math.toRadians(60)))
                .build();

        Trajectory purpleDepositPathC = drive.trajectoryBuilder(startPose,false)
                .lineToSplineHeading(new Pose2d(new Vector2d(12, -37), Math.toRadians(90)))
                .build();

        Trajectory backUpFromSpikes = drive.trajectoryBuilder(purpleDepositPathC.end(),false)
                .back(4)
                .build();

        Trajectory yellowDepositPath = drive.trajectoryBuilder(backUpFromSpikes.end(),false)
                .splineToLinearHeading(new Pose2d(new Vector2d(46, -36), Math.toRadians(180)), Math.toRadians(60))
                .build();

        Trajectory yellowDepositPathC = drive.trajectoryBuilder(backUpFromSpikes.end(),false)
                .splineToLinearHeading(new Pose2d(new Vector2d(46, -26), Math.toRadians(180)), Math.toRadians(60))
                .build();

        Trajectory yellowDepositPathL = drive.trajectoryBuilder(backUpFromSpikes.end(),false)
                .splineToLinearHeading(new Pose2d(new Vector2d(46, -16), Math.toRadians(180)), Math.toRadians(60))
                .build();

        Trajectory yellowDepositPathR = drive.trajectoryBuilder(backUpFromSpikes.end(),false)
                .splineToLinearHeading(new Pose2d(new Vector2d(45, -30), Math.toRadians(180)), Math.toRadians(60))
                .build();

        Trajectory park;

        int target=0;

        drive.closeClaw();

        propPos = myPipeline.position;
        telemetry.addData("Position", propPos);
        telemetry.update();

        waitForStart();

        long startTime = new Date().getTime();
        long time = 0;

        while (time < 50 && opModeIsActive()) {
            time = new Date().getTime() - startTime;
            propPos = myPipeline.position;
            telemetry.addData("Position", propPos);
        }

        currentState = State.PURPLE_DEPOSIT_PATH;
        if (propPos == PropFindRight.pos.LEFT) {
            drive.followTrajectoryAsync(purpleDepositPathL);
        } else if (propPos == PropFindRight.pos.RIGHT) {
            drive.followTrajectoryAsync(purpleDepositPathR);
        } else {
            drive.followTrajectoryAsync(purpleDepositPathC);
        }

        while (opModeIsActive() && !isStopRequested()) {
            drive.update();
            drive.backSlidesMove(target);
            switch (currentState) {
                case PURPLE_DEPOSIT_PATH:
                    if (!drive.isBusy()) {
                        currentState = State.PURPLE_DEPOSIT;
                    } else {
                        drive.intakeToGround();
                    }
                    break;
                case PURPLE_DEPOSIT:

                        drive.openClaw();
                        drive.closeHook();
/*
                            if (propPos == PropFindRed.pos.RIGHT) {
                                drive.turn(Math.toRadians(30));
                            } else
                            if (propPos == PropFindRed.pos.LEFT) {
                                drive.turn(Math.toRadians(-60));
                            }*/

                            currentState = State.UNTURN;


                    break;
                case UNTURN:
                    if (!drive.isBusy()) {
                        drive.followTrajectoryAsync(backUpFromSpikes);
                        currentState = State.BACKUP_FROM_SPIKES;

                    }
                    break;
                case BACKUP_FROM_SPIKES:
                    if (!drive.isBusy()) {
                        if (propPos == PropFindRight.pos.LEFT){
                            drive.followTrajectoryAsync(yellowDepositPathL);
                        } else if (propPos == PropFindRight.pos.RIGHT){
                            drive.followTrajectoryAsync(yellowDepositPathR);
                        } else if (propPos == PropFindRight.pos.MID){
                            drive.followTrajectoryAsync(yellowDepositPathC);
                        }

                        currentState = State.YELLOW_DEPOSIT_PATH;
                        //currentState = State.STOP;
                    } else {
                        target= CSCons.OuttakePosition.LOW.getTarget();
                        drive.intakeToTransfer();
                        drive.outtakeToBackdrop();
                    }
                    break;
                case YELLOW_DEPOSIT_PATH:
                    if (!drive.isBusy() ) {
                        currentState = State.YELLOW_DEPOSIT_STRAIGHT;
                    }
                    break;
                case YELLOW_DEPOSIT_STRAIGHT:
                    if(!drive.isBusy()){
                        Trajectory straight= drive.trajectoryBuilder(drive.getPoseEstimate())
                                .back(3).build();
                        drive.followTrajectoryAsync(straight);

                        currentState= State.YELLOW_DEPOSIT;
                    }
                    break;
                case YELLOW_DEPOSIT:

                    if(resetInt == 0){
                        depositTime.reset();
                        resetInt++;
                    }
                    if(resetInt == 1){
                        if (!drive.isBusy()) {

                            park = drive.trajectoryBuilder(drive.getPoseEstimate(), false)
                                    .back(4)
                                    .build();
                            //april tag alignment
                            //if april tag is aligned drop and
                            if (depositTime.milliseconds() > 500) {
                                drive.dropPixel();
                            }
                            if (depositTime.milliseconds() > 700) {
                                drive.followTrajectoryAsync(park);
                                currentState = State.BACK;

                            }
                        }
                    }
                    break;
                case BACK:
                    if (!drive.isBusy()) {
                        drive.outtakeToTransfer();
                        target = 0;
                        park = drive.trajectoryBuilder(drive.getPoseEstimate(),false)
                                .splineToLinearHeading(new Pose2d(new Vector2d(50, -56), Math.toRadians(180)), Math.toRadians(0))
                                .build();
                        drive.followTrajectoryAsync(park);
                        currentState= State.PARK;
                    }
                    break;
                case PARK:
                   break;
            }
        }
    }
}