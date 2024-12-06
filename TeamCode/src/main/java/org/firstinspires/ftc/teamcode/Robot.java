package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.ClawArm;
import org.firstinspires.ftc.teamcode.subsystems.ClawLift;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.utils.MissionTimer;

/**
 * This class consists of all the subsystems and utilities used to form a robot.
 *
 * PUBLIC METHODS:
 *     Robot(hardwareMap) - constructor for instantiating a robot
 *     Drivetrain getDrivetrain() - allows access to Drivetrain subsystem
 *     Claw getClaw - Allows access to claw subsystem
 *     MissionTimer getMissionTimer - allows access to Mission Timmer utility
 *     void reportTelemetry() - reports robot telemetry information
 *
 * VERSION   DATE     WHO  DETAIL
 * 00.01.00  11Nov24  SEB  Initial release
 * 00.01.01  26Nov24  SEB  Add claw to subsystems. Uddate robot to use private subsystems.
 * 00.01.02  28Nov24  SEB  Remove types in constructor. Add telemetry to constructor and other methods.
 * 00.01.03  03Dec24  SEB  Add clawArm.
 *
 */
public class Robot {

    // Declare robot subsystems as null instance
    private Drivetrain drivetrain;
    private Claw claw;
    private ClawArm clawArm;
    private ClawLift clawLift;
    private MissionTimer missionTimer;

    /**
     * - Robot Constructor -
     * Uses HardwareMap to import the robot subsystems
     */
    public Robot(HardwareMap hardwareMap, Telemetry telemetry) {

        try {
            // Shared resources
            missionTimer = new MissionTimer();
            // Instantiate robot subsystems
            drivetrain = new Drivetrain(hardwareMap, telemetry);  // Drivetrain is four motors
            claw = new Claw(hardwareMap, telemetry, missionTimer);  // Claw is two servos
            clawArm = new ClawArm(hardwareMap, telemetry);  // Claw arm is a single motor
            clawLift = new ClawLift(hardwareMap, telemetry);  // Claw arm is a single motor
        } catch (Exception e) {
            telemetry.addData("Error", "Robot initialization failed: " + e.getMessage());
            telemetry.update();
            throw new RuntimeException("Robot initialization failed", e);
        }
    }

    /**
     * Allows public access to the drivetrain subsystem
     */
    public Drivetrain getDrivetrain() {
        return drivetrain;
    }

    /**
     * Allows public access to the claw subsystem
     */
    public Claw getClaw() {
        return claw;
    }

    /**
     * Allows public access to the claw subsystem
     */
    public ClawArm getClawArm() {
        return clawArm;
    }

    /**
     * Allows public access to the claw subsystem
     */
    public ClawLift getClawLift() {
        return clawLift;
    }

    /**
     * Allows public access to the missionTimer utility
     */
    public MissionTimer getMissionTimer() {
        return missionTimer;
    }
}
