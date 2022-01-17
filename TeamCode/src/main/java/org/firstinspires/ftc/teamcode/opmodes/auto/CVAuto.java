package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.core.robot.vision.robot.TseDetector;
import org.firstinspires.ftc.teamcode.core.thread.EventThread;
@Autonomous
@Config
public class CVAuto extends LinearOpMode {
    public static int zeroOrOneRedorBlue = 0;
    public EventThread eventThread = new EventThread();
    @Override
    public void runOpMode(){
        telemetry.addLine("Starting");
        telemetry.update();
        TseDetector webcam = new TseDetector(hardwareMap, "webcam", true);
        waitForStart();
        telemetry.addLine("Running");
        telemetry.update();
        while (opModeIsActive()) {
            telemetry.addData("webcamOutput", webcam.run(zeroOrOneRedorBlue == 0));
            telemetry.update();
        }
        eventThread.interrupt();
        requestOpModeStop();
    }
}
