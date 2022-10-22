/* Copyright (c) 2022 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.Reno.poc;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.Reno.HardwareRobot;

@Autonomous(name="POC: Auto Drive Calibration", group="Concept")
//@Disabled
public class ConceptAutoDriveCalibration extends LinearOpMode {

    /* Declare OpMode members. */
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;
    //private BNO055IMU       imu         = null;      // Control/Expansion Hub IMU
    private HardwareRobot robot   = new HardwareRobot();

    private double          robotHeading  = 0;
    private double          headingOffset = 0;
    private double          headingError  = 0;

    // These variable are declared here (as class members) so they can be updated in various methods,
    // but still be displayed by sendTelemetry()
    private double  targetHeading = 0;
    private double  driveSpeed    = 0;
    private double  turnSpeed     = 0;
    private double  leftSpeed     = 0;
    private double  rightSpeed    = 0;
    private int     leftFrontTarget    = 0;
    private int     rightFrontTarget   = 0;
    private int     leftBackTarget    = 0;
    private int     rightBackTarget   = 0;
    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final double     COUNTS_PER_MOTOR_REV    = 537.7 ;   // eg: GoBILDA 312 RPM Yellow Jacket
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);

    // These constants define the desired driving/control characteristics
    // They can/should be tweaked to suit the specific robot drive train.
    static final double     DRIVE_SPEED             = 0.5;     // Max driving speed for better distance accuracy.
    static final double     TURN_SPEED              = 0.02;     // Max Turn speed to limit turn rate
    static final double     HEADING_THRESHOLD       = 1.0 ;    // How close must the heading get to the target before moving to next step.
                                                               // Requiring more accuracy (a smaller number) will often make the turn take longer to get into the final position.
    // Define the Proportional control coefficient (or GAIN) for "heading control".
    // We define one value when Turning (larger errors), and the other is used when Driving straight (smaller errors).
    // Increase these numbers if the heading does not corrects strongly enough (eg: a heavy robot or using tracks)
    // Decrease these numbers if the heading does not settle on the correct value (eg: very agile robot with omni wheels)
    static final double     P_TURN_GAIN            = 0.02;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_GAIN           = 0.03;     // Larger is more responsive, but also less stable


    private ElapsedTime     runtime = new ElapsedTime();

    private ConceptPidMotorController motorController = null;

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);
        // Initialize the drive system variables.
        leftFrontDrive = robot.leftDriveFront;//hardwareMap.get(DcMotor.class, "left_drive");
        rightFrontDrive = robot.rightDriveFront;//hardwareMap.get(DcMotor.class, "right_drive");
        leftBackDrive = robot.leftDriveBack;
        rightBackDrive = robot.rightDriveBack;

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        robot.setDriveForward();

        // Ensure the robot is stationary.  Reset the encoders and set the motors to BRAKE mode
        robot.resetEncoder();
        // Wait for the game to start (Display Gyro value while waiting)
        while (opModeInInit()) {
            //telemetry.addData("gyro Cali status", imu.isGyroCalibrated());
            //telemetry.addData("sys Cali status", imu.isSystemCalibrated());
            telemetry.addData(">", "Robot Heading = %4.0f", getRawHeading());
            telemetry.addData(">", "Robot Roll = %4.0f", getRawRoll());
            telemetry.addData(">", "Robot Pitch = %4.0f", getRawPitch());
            telemetry.update();
        }



        // Set the encoders for closed loop speed control, and reset the heading.
        robot.enableEncoder();
        resetHeading();
        motorController = new ConceptPidMotorController(this.getRawHeading());
        waitForStart();

        runtime.reset();

        driveStraight(DRIVE_SPEED, 24.0, 0.0);
        sleep(2000);

        turnToHeading( 0, -90.0 );
        telemetry.addData("turn -90 ", this.getRawHeading());
        telemetry.update();
        sleep(2000);

        driveStraight(DRIVE_SPEED, 24.0, 0.0);
        sleep(2000);
        turnToHeading( 0, -180);
        telemetry.addData("turn -90 ", this.getRawHeading());
        telemetry.update();
        sleep(2000);
        driveStraight(DRIVE_SPEED, 24.0, 0.0);


        sleep(1000);  // Pause to display last telemetry message.
    }

    /*
     * ====================================================================================================
     * Driving "Helper" functions are below this line.
     * These provide the high and low level methods that handle driving straight and turning.
     * ====================================================================================================
     */

    // **********  HIGH Level driving functions.  ********************

    /**
    *  Method to drive in a straight line, on a fixed compass heading (angle), based on encoder counts.
    *  Move will stop if either of these conditions occur:
    *  1) Move gets to the desired position
    *  2) Driver stops the opmode running.
    *
    * @param driveSpeed MAX Speed for forward/rev motion (range 0 to +1.0) .
    * @param distance   Distance (in inches) to move from current position.  Negative distance means move backward.
    * @param heading      Absolute Heading Angle (in Degrees) relative to last gyro reset.
    *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
    *                   If a relative angle is required, add/subtract from the current robotHeading.
    */
    public void driveStraight(double driveSpeed,
                              double distance,
                              double heading) {

        robot.resetEncoder();
        robot.enableEncoder();

        if(distance < 0)
        {
            robot.setDriveBackward();
        }
        else
        {
            robot.setDriveForward();
        }

        if (opModeIsActive()) {

            robot.setTargetPosition(Math.abs(distance));

            //robot.tankDrive(driveSpeed, driveSpeed);
            //turnAndDrive(driveSpeed, heading);
            robot.drive(driveSpeed, 0);
            while (opModeIsActive() && robot.isBusyDriving()) {

                // Determine required steering to keep on heading
                turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    turnSpeed *= -1.0;

                // Apply the turning correction to the current driving speed.
                robot.drive(driveSpeed, turnSpeed);

                // Display drive status for the driver.
                sendTelemetry(true);
            }

            // keep looping while we are still active, and BOTH motors are running.
            //while (opModeIsActive() && robot.isBusyDriving())
            //{
                //turnAndDrive(driveSpeed, heading);
            //}

            //robot.tankDrive(driveSpeed, driveSpeed);
            robot.stop();
            robot.resetEncoder();
        }
    }


    public void turnToHeading(double driveSpeed, double heading)
    {
        double requestHeading = heading;
        // Normalize the heading to be within +/- 180 degrees
        heading -= this.getRawHeading();
        while (heading > 180)  heading -= 360;
        while (heading <= -180) heading += 360;

        robot.disableEncoder();
        robot.setDriveForward();
        /*
        if(heading < 0) {
            robot.setDriveBackward();
        }
        else
        {
            robot.setDriveForward();
        }
        */

        motorController = new ConceptPidMotorController(0.02, 0.0, 0.03);
        motorController.setGoal(heading);
        motorController.setFeedbackValue(this.getRawHeading());
        telemetry.addData("Error:Steer",  "%5.1f:%5.1f", motorController.getError(), heading);
        telemetry.update();

        int count = 0;
        while (opModeIsActive() && ((Math.abs(motorController.getError()) > HEADING_THRESHOLD)))
        {
            double turnSpeed = driveSpeed;

                motorController.setFeedbackValue(this.getRawHeading());
                turnSpeed = motorController.getValue();

                //turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);
            if(motorController.getError() < 0 && turnSpeed > 0) {
                turnSpeed *= -1;
            }
            if(motorController.getError() > 0 && turnSpeed < 0) {
                turnSpeed *= -1;
            }

                robot.drive(0, turnSpeed);

                telemetry.addData("Angle Target:Current", "%5.2f:%5.0f", requestHeading, this.getRawHeading());
                telemetry.addData("PID", "%5.2f - %5.0f - %5.2f", motorController.getProportional(),
                        motorController.getIntegral(), motorController.getDerivative());
                telemetry.addData("Error: turn Speed",  "%5.1f:%5.1f", motorController.getError(), turnSpeed);
                telemetry.addData("", robot.getMotorStatus());
                telemetry.addData("count", count++);
                telemetry.update();
        }
        robot.stop();
        sleep(250);

    }

    /**
     *  Method to obtain & hold a heading for a finite amount of time
     *  Move will stop once the requested time has elapsed
     *  This function is useful for giving the robot a moment to stabilize it's heading between movements.
     *
     * @param maxTurnSpeed      Maximum differential turn speed (range 0 to +1.0)
     * @param heading    Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     * @param holdTime   Length of time (in seconds) to hold the specified heading.
     */
    public void holdHeading(double maxTurnSpeed, double heading, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();

        // keep looping while we have time remaining.
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            moveRobot(0, turnSpeed);

            // Display drive status for the driver.
            sendTelemetry(false);
        }

        // Stop all motion;
        moveRobot(0, 0);
    }

    // **********  LOW Level driving functions.  ********************

    /**
     * This method uses a Proportional Controller to determine how much steering correction is required.
     *
     * @param desiredHeading        The desired absolute heading (relative to last heading reset)
     * @param proportionalGain      Gain factor applied to heading error to obtain turning power.
     * @return                      Turning power needed to get to required heading.
     */
    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        targetHeading = desiredHeading;  // Save for telemetry

        // Get the robot heading by applying an offset to the IMU heading
        robotHeading = getRawHeading() - headingOffset;

        // Determine the heading current error
        headingError = targetHeading - robotHeading;

        // Normalize the error to be within +/- 180 degrees
        while (headingError > 180)  headingError -= 360;
        while (headingError <= -180) headingError += 360;

        // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
        return Range.clip(headingError * proportionalGain, -1, 1);
    }

    /**
     * This method takes separate drive (fwd/rev) and turn (right/left) requests,
     * combines them, and applies the appropriate speed commands to the left and right wheel motors.
     * @param drive forward motor speed
     * @param turn  clockwise turning motor speed.
     */
    public void moveRobot(double drive, double turn) {
        driveSpeed = drive;     // save this value as a class member so it can be used by telemetry.
        turnSpeed  = turn;      // save this value as a class member so it can be used by telemetry.

        leftSpeed  = drive - turn;
        rightSpeed = drive + turn;

        // Scale speeds down if either one exceeds +/- 1.0;
        double max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
        if (max > 1.0)
        {
            leftSpeed /= max;
            rightSpeed /= max;
        }

        leftFrontDrive.setPower(leftSpeed);
        rightFrontDrive.setPower(rightSpeed);
        leftBackDrive.setPower(leftSpeed);
        rightBackDrive.setPower(rightSpeed);
    }

    /**
     *  Display the various control parameters while driving
     *
     * @param straight  Set to true if we are driving straight, and the encoder positions should be included in the telemetry.
     */
    private void sendTelemetry(boolean straight) {

        if (straight) {
            telemetry.addData("Motion", "Drive Straight");
            telemetry.addData("Target Pos L:R",  "%7d:%7d",      leftFrontTarget,  rightFrontTarget);
            telemetry.addData("Actual Pos L:R",  "%7d:%7d",      leftFrontDrive.getCurrentPosition(),
                    rightFrontDrive.getCurrentPosition());
        } else {
            telemetry.addData("Motion", "Turning");
        }

        telemetry.addData("Angle Target:Current", "%5.2f:%5.0f", targetHeading, robotHeading);
        telemetry.addData("Error:Steer",  "%5.1f:%5.1f", headingError, turnSpeed);
        telemetry.addData("Wheel Speeds L:R.", "%5.2f : %5.2f", leftSpeed, rightSpeed);
        telemetry.update();
    }

    /**
     * read the raw (un-offset Gyro heading) directly from the IMU
     */
    public double getRawHeading() {
        return robot.getRawHeading();
    }

    public double getRawRoll() {
       return robot.getRawRoll();
    }

    public double getRawPitch() {
        return robot.getRawPitch();
    }

    /**
     * Reset the "offset" heading back to zero
     */
    public void resetHeading() {
        // Save a new heading offset equal to the current raw heading.
        headingOffset = getRawHeading();
        robotHeading = 0;
    }
}
