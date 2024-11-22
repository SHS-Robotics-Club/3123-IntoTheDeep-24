package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.util.ElapsedTime;
import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.MILLISECONDS;

public class MissionTimer {

    // Declare timer
    private ElapsedTime missionTimer;

    /**
     * - Constructor -
     * Instantiates a new ElapsedTime timer of class MissionTimer
     */
    public MissionTimer() {

        // Instantiate a MissionTimer
        ElapsedTime missionTimer = new ElapsedTime(MILLISECONDS);
    }

    /**
     * Resets the missionTimer to a value of zero
     */
    public void init() {
        missionTimer.reset();
    }

    /**
     * Reads the current timer value
     * @return
     */
    public double getTimeMS() {

        // Get current time
        return missionTimer.time();
    }

}
