package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.util.ElapsedTime;
import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.MILLISECONDS;

/**
 * This class uses an ElapsedTime to create a mission timer.
 *
 * PUBLIC METHODS:
 *     MissionTimer() - constructor for instantiating a mission timer
 *     void init() - initializes the mission timer
 *     double getTimeMS() - returns current mission timer in milliseconds
 *
 * VERSION   DATE     WHO  DETAIL
 * 00.01.00  11Nov24  SEB  Initial release
 *
 */
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
