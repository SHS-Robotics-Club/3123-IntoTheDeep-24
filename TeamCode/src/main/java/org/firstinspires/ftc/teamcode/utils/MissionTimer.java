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
 * 00.01.01  25Nov24  SEB  Added hasElapsed() and isWithinInterval() methods.
 *                         Also minor edits.
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
        missionTimer = new ElapsedTime(MILLISECONDS);
    }

    /**
     * Resets the missionTimer to a value of zero
     */
    public void init() {
        missionTimer.reset();
    }

    /**
     * Reads the current timer value
     * @return time since last reset in milliseconds
     */
    public double getTimeMS() {

        // Get current time
        return missionTimer.milliseconds();
    }

    /**
     * Reads the current timer value
     * @return time since last reset in seconds
     */
    public double getTimeSec() {

        // Get current time
        return missionTimer.seconds();
    }

    /**
     * Determines if the specified mission time has been reached
     * @param milliseconds target mission time in milliseconds
     * @return true is mission time has been reached, otherwise false
     */
    public boolean hasElapsed(double milliseconds) {
        return missionTimer.milliseconds() >= milliseconds;
    }

    /**
     * Determines if the current mission time is within the provided mission time limits
     * @param startMS starting mission time limit
     * @param endMS ending mission time limit
     * @return true if current mission time is within provided limits, otherwise false
     */
    public boolean isWithinInterval(double startMs, double endMs) {
        double elapsed = missionTimer.milliseconds();
        return elapsed >= startMs && elapsed <= endMs;
    }    

}
