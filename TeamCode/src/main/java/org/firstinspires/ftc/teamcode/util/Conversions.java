package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.teamcode.RC;

public class Conversions {

    private static double deadzone = 0.4;
    private static double nearzero = .0000001;

    public Conversions(){}


    public static double servoNormalize(int pulse){
        double normalized = (double)pulse;
        return (normalized - 750.0) / 1500.0; //convert mr servo controller pulse width to double on _0 - 1 scale
    }
    /**
     * returns the minimum difference (in absolute terms) between two angles,
     * preserves the sign of the difference
     *
     * @param angle1
     * @param angle2
     * @return
     */
    public static double diffAngle(double angle1, double angle2){
        return Math.abs(angle1 - angle2) < Math.abs(angle2-angle1) ? Math.abs(angle1 - angle2) : Math.abs(angle2-angle1);
    }

    public static double diffAngle2(double angle1, double angle2){

        double diff = angle1 - angle2;

        //allow wrap around

        if (Math.abs(diff) > 180)
        {
            if (diff > 0) {
                diff -= 360;
            } else {
                diff += 360;
            }
        }
        return diff;
    }

    public static double wrap360(double angle){
        double tmp;
        tmp = wrapAngle(angle);
        if (tmp < 0.0) return 360+tmp;
        return tmp;
    }

    public static double wrap360(double angleCurrent, double angleChange){

        return Math.abs(wrap360(angleCurrent + angleChange));
        //added abs() to fix case where you could get a negative zero. eg wrap360(-460, 100)
    }

    public static boolean between(double value, double minValue, double maxValue){
        return (value>=minValue && value<=maxValue);
    }

    public static boolean between360(double value, double leftAngle, double rightAngle){
        if(wrap360(value) == wrap360(leftAngle) || wrap360(value) == wrap360(rightAngle)){
            return true;
        }

        double shift = -diffAngle2(0,leftAngle);

        leftAngle = wrap360(leftAngle, shift);
        rightAngle = wrap360(rightAngle, shift);
        value = wrap360(value, shift);

        return (between(value, leftAngle, rightAngle));
    }

    public static boolean between360NonDirectional(double value, double leftAngle, double rightAngle){
        if(wrap360(value) == wrap360(leftAngle) || wrap360(value) == wrap360(rightAngle)){
            return true;
        }

        double shift = -diffAngle2(0,leftAngle);

        leftAngle = wrap360(leftAngle, shift);
        rightAngle = wrap360(rightAngle, shift);
        value = wrap360(value, shift);

        return (between(value, leftAngle, rightAngle));
    }

    public static double wrapAngle(double angle){
        return (angle) % 360;
    }
    /**
     * Apply an angular adjustment to a base angle with result wrapping around at 360 degress
     *
     * @param angle1
     * @param angle2
     * @return
     */


    public static double wrapAngle(double angle1, double angle2){
        return (angle1 + angle2) % 360;
    }

    public static double wrapAngleMinus(double angle1, double angle2){
        return 360-((angle1 + angle2) % 360);
    }

    public double getBearingTo(double x, double y, double poseX, double poseY){
        double diffx = x-poseX;
        double diffy = y-poseY;
        return ( Math.toDegrees(Math.atan2( diffy, diffx)) - 90  + 360 ) % 360;
    }

    public double getBearingOpposite(double x, double y, double poseX, double poseY){
        double diffx = x-poseX;
        double diffy = y-poseY;
        return ( Math.toDegrees(Math.atan2( diffy, diffx)) + 90 + 360 ) % 360;
    }

    public double getDistanceTo(double x, double y, double poseX, double poseY){

        double dx = x - poseX;
        double dy = y - poseY;
        return Math.sqrt(dx*dx + dy*dy);

    }

    public static long futureTime(float seconds){
        return System.nanoTime() + (long) (seconds * 1e9);
    }

    public static boolean notdeadzone(double value){
        if (value> -deadzone && value < deadzone) return false;
        else return true;
    }

    public static boolean nearZero(double value){
        if (value> -nearzero && value < nearzero) return true;
        else return false;
    }

    public static double nextCardinal(double currentAngle, boolean right, double hop){
        double tmp;
        if (right) {
            tmp = (currentAngle + hop) % 360;
            tmp = Math.floor(tmp/90) + 1;
            if (tmp>=4) tmp = 0;
        }
        else{
            tmp = (currentAngle - hop);
            tmp = Math.floor(tmp/90);
            if (tmp<0) tmp = 3;
        }
        return tmp*90;
    }

}
