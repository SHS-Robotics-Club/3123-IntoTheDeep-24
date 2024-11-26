
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
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
 *
 */
public class Robot {

    // Declare robot subsystems as null instance
    private Drivetrain drivetrain;
    private Claw claw;
    private MissionTimer missionTimer;

    /**
     * - Robot Constructor -
     * Uses HardwareMap to import the robot subsystems
     */
    public Robot(HardwareMap hardwareMap) {

        // Shared resources
        MissionTimer missionTimer = new MissionTimer();
        // Instantiate robot subsystems
        Drivetrain drivetrain = new Drivetrain(hardwareMap);
        Claw claw = new Claw(hardwareMap, missionTimer);
    }

    /**
     * Allows public access to the drivetrain subsystem
     */
    public void getDrivetrain() {
        return drivetrain;
    }

    /**
     * Allows public access to the claw subsystem
     */
    public void getClaw() {
        return claw;
    }

        /**
     * Allows public access to the missionTimer utility
     */
    public void getMissionTimer() {
        return missionTimer;
    }
}
