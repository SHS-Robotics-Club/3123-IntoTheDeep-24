
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.utils.MissionTimer;

/**
 * This class consists of all the subsystems and utilities used to form a robot.
 *
 */
public class Robot {

    // Declare robot subsystems as null instance
    public Drivetrain drivetrain;

    /**
     * - Robot Constructor -
     * Uses HardwareMap to import the drivetrain
     */
    public Robot(HardwareMap hardwareMap) {

        // Instantiate robot subsystems
        Drivetrain drivetrain = new Drivetrain(hardwareMap);

    }

    /**
     * Initializes robot
     */
    public void init() {
        drivetrain.init();
    }

    /**
     * Reports robot subsystem telemetry
     */
    public void reportTelemetry() {
        drivetrain.reportTelemetry();
    }
}
