package org.firstinspires.ftc.teamcode.Jeremy.AutonomousBot.FollowTheLine;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;


@Autonomous(name="FTL: main switch", group="FTL")
@Disabled
public class FTLmain extends LinearOpMode {

    /* Declare OpMode members. */
    private double          headingError  = 0;

    private DcMotor         leftDrive   = null;
    private DcMotor         rightDrive  = null;
    private IMU imu         = null;
    NormalizedColorSensor colorSensor1;
    NormalizedColorSensor colorSensor2;
    private DistanceSensor distanceSensor1;
    private DistanceSensor distanceSensor2;
    ElapsedTime runtime = new ElapsedTime();

    private double  targetHeading = 0;
    private double  driveSpeed    = 0;
    private double turnSpeed = 0;
    private double  leftSpeed     = 0;
    private double  rightSpeed    = 0;
    private int     leftTarget    = 0;
    private int     rightTarget   = 0;
    double power = 0;
    double tpower = 0;
    double leftPow = 0.1;
    double rightPow = 0.1;
    
    static final double whiteThreshold = 0.16;  // spans between 0.0 - 1.0 from dark to light

    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ; //no external gearing
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    static final double     DRIVE_SPEED             = 0.5;     // Max driving speed for better distance accuracy.
    static final double     TURN_SPEED              = 0.03;     // Max Turn speed to limit turn rate

    static final double     HEADING_THRESHOLD       = 1.0 ;    // How close must the heading get to the target before moving to next step.

    static final double     P_TURN_GAIN            = 0.02;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_GAIN           = 0.03;     // Larger is more responsive, but also less stable
    ElapsedTime getOut = new ElapsedTime();

    @Override
    public void runOpMode() {

        // initialize the drive variables.
//            Robot robot = new Robot(hardwareMap, new Movement()); stoopid i hate java
        // TO MAX: once you've moved all the code:
        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        imu = hardwareMap.get(IMU.class, "imu");
        colorSensor1 = hardwareMap.get(NormalizedColorSensor.class, "sensor_color1");
        colorSensor2 = hardwareMap.get(NormalizedColorSensor.class, "sensor_color2");
        distanceSensor1 = hardwareMap.get(DistanceSensor.class, "sensor_distance");
        distanceSensor2 = hardwareMap.get(DistanceSensor.class, "sensor_distance2");


        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        imu.resetYaw();

        if (colorSensor1 instanceof SwitchableLight) {
            ((SwitchableLight) colorSensor1).enableLight(true);
        }
        if (colorSensor2 instanceof SwitchableLight) {
            ((SwitchableLight) colorSensor1).enableLight(true);
        }


        colorSensor1.setGain(15);
        colorSensor2.setGain(15);


        while (opModeInInit()) {

            if (getBrightness1() > whiteThreshold && getBrightness2() > whiteThreshold) {
                telemetry.addData("Status", "In between line, ready to go");
            } else if (getBrightness1() < whiteThreshold && getBrightness2() < whiteThreshold) {
                telemetry.addData("Status", "above white threshold on both sensors");

            } else if (getBrightness1() < whiteThreshold && getBrightness2() > whiteThreshold) {
                telemetry.addData("Status", "above white threshold on color sensor 1");
            } else if (getBrightness1() > whiteThreshold && getBrightness2() < whiteThreshold) {
                telemetry.addData("Status", "above white threshold on color sensor 2");
            } else {
                telemetry.addData("Status", "error of some sort");
            }

            doAllTelemetry();
            telemetry.update();

            imu.resetYaw();
        }
        power = 0.2;
        tpower = -0.1;
        leftPow = 0.1;
        rightPow = 0.1;
        boolean manual = false;
        //with gyro motion
        waitForStart();

        while (opModeIsActive()) {
            doAllTelemetry();

            telemetry.addData("power", "%4.2f", power);
            telemetry.addData("turning power", "%4.2f", tpower);

            telemetry.update();
            if (rightSensorCheck() && leftSensorCheck()) {
                setSpeed(0);
                telemetry.addData("Switching", "Manual");
                telemetry.update();
                sleep(1000);
                manual = true;
innerLoop:
                while (opModeIsActive() && manual == true) {
                    leftPow = 0.1;
                    rightPow = 0.1;
                    if (rightSensorCheck() && leftSensorCheck()) {
                        manual = false;
                        telemetry.addData("Switching", "Automatic");
                        telemetry.update();
                        sleep(1000);
                        break innerLoop;
                    } while(rightSensorCheck() && opModeIsActive() && !leftSensorCheck() && manual == true) {
                        encoderDrive(0.2, rightPow, -rightPow, 0.20);
                        rampRight(0.25, 10);
                    } while (leftSensorCheck() && opModeIsActive() && !rightSensorCheck() && manual == true) {
                        encoderDrive(0.2, -leftPow, leftPow, 0.20);
                        rampLeft(0.25, 10);
                    } if (!leftSensorCheck() && !rightSensorCheck() && opModeIsActive() && manual == true) {
                        encoderDrive(0.1, 3, 3, 0.5);
                    }
               }


            } else if (getBrightness1() < whiteThreshold && getBrightness2() > whiteThreshold) {
                checkLeftOld();
            } else if (getBrightness2() < whiteThreshold && getBrightness1() > whiteThreshold) {
                checkRightOld();

//Current issue is the color sensors sensing weird. This might be fixed with either putting the 
//color sensors back where it was so it doesnt reflext the light unevenly, hiding the wires so that they read evenly,reorganizing wire managments,
//or using something to cover the top consistantly. Once the sensors read equally guage the white threshold and then 
//either use the currently implimented code or go back to the code above for smoother, less sharp turns.
//                } else if (getBrightness1() < whiteThreshold) {
//                    encoderDrive(0.2, -1, 0, 0.50);
//                } else if (getBrightness2() < whiteThreshold) {
//                    encoderDrive(0.2, 0, -1, 0.50);

//                } else if (getBrightness1() < whiteThreshold) {
//                    checkLeftOld();
//                } else if (getBrightness2() < whiteThreshold) {
//                    checkRightOld();

//

                } else {
                    setSpeed(power);
                }

        }
    }
    private void doAllTelemetry() {
        NormalizedRGBA colors1 = colorSensor1.getNormalizedColors();
        telemetry.addData("Light Level (0 to 1)Left Sensor",  "%4.2f", colors1.alpha);
        NormalizedRGBA colors2 = colorSensor2.getNormalizedColors();
        telemetry.addData("Light Level (0 to 1) Right Sensor",  "%4.2f", colors2.alpha);

        telemetry.addData("------------------------", "--------------");

        telemetry.addData("Heading- Target : Current", "%5.2f : %5.0f", targetHeading, getHeading());

        telemetry.addData("Wheel Speeds L : R", "%5.2f : %5.2f", leftSpeed, rightSpeed);
        telemetry.addData("Error  : Steer Pwr",  "%5.1f : %5.1f", headingError, turnSpeed);
        telemetry.addData(">", "Robot Heading = %4.0f", getHeading());
        telemetry.addData("Heading- Target : Current", "%5.2f : %5.0f", targetHeading, getHeading());
        telemetry.addData("Error  : Steer Pwr",  "%5.1f : %5.1f", headingError, turnSpeed);
    }
    double getBrightness1() {
        NormalizedRGBA colors = colorSensor1.getNormalizedColors();
        return colors.alpha;
    }
    double getBrightness2() {
        NormalizedRGBA colors = colorSensor2.getNormalizedColors();
        return colors.alpha;
    }

    private boolean rightSensorCheck() {
        if (distanceSensor2.getDistance(DistanceUnit.INCH) < 10) {
            return true;
        }
        if (distanceSensor2.getDistance(DistanceUnit.INCH) > 10) {
            return false;
        }
        return true;
    }
    private boolean leftSensorCheck() {
        if (distanceSensor1.getDistance(DistanceUnit.INCH) < 10) {
            return true;
        }
        if (distanceSensor1.getDistance(DistanceUnit.INCH) > 10) {
            return false;
        }
        return true;
    }
    private void checkLeftDime() {
            encoderDrive(0.1, -1, 0, 0.50);
    }
    private void checkRightDime() {
            encoderDrive(0.1, 0, -1, 0.50);
    }
    private void turnTo(double heading) {
        turnToHeading(P_TURN_GAIN, heading);
    }
    private void setSpeed(double speed) {
        leftDrive.setPower(speed);
        rightDrive.setPower(speed);
    }

    private void move(double distance, double turn) {
        moveRobot(distance, turn);
    }
    private void checkLeft() {
        while (getBrightness1() < whiteThreshold && opModeIsActive()) {
            rightDrive.setPower(0.01);
            leftDrive.setPower(tpower);
        }
    }
    private void checkRight() {
        while (getBrightness2() < whiteThreshold && opModeIsActive()) {
            leftDrive.setPower(0.01);
            rightDrive.setPower(tpower);
        }
    }
    private void checkLeftOld() {
        while (getBrightness1() < whiteThreshold && opModeIsActive()) {
            rightDrive.setPower(1);
            leftDrive.setPower(0.25);
        }
    }
    private void checkRightOld() {
        while (getBrightness2() < whiteThreshold && opModeIsActive()) {
            leftDrive.setPower(1);
            rightDrive.setPower(0.25);
        }
    }
    //checkBoth() is basically just checkRight + checkLeft.
    private void checkBoth() {
        // Check if we need to correct to the left.
        boolean hasTurnedLeft = false;
        if (getBrightness1() < whiteThreshold && getBrightness2() > whiteThreshold) {
            leftDrive.setPower(0.01);
            rightDrive.setPower(tpower);
            hasTurnedLeft = true;
        }
        if (hasTurnedLeft) {
            return;
        }
        // Check if we need to correct to the right.
        if (getBrightness2() < whiteThreshold && getBrightness1() > whiteThreshold) {
            rightDrive.setPower(0.01);
            leftDrive.setPower(tpower);
        }
    }
    private void checkLineRight(double turn, double adjustment) {
        getBrightness2();
        while (getBrightness2() > whiteThreshold) {
            move(0, -turn);
        }
        move(0, -adjustment);
    }
    private void rampPower(double increment, double topSpeed) {
        if (true) {
            // Keep stepping up until we hit the max value.
            power += increment ;
            if (power >= topSpeed ) {
                power = topSpeed;
            }
        }

    }

    private void rampLeft(double increment, double topSpeed) {
        if (true) {
            // Keep stepping up until we hit the max value.
            leftPow += increment ;
            if (leftPow >= topSpeed ) {
                leftPow = topSpeed;
            }
        }

    }

    private void rampRight(double increment, double topSpeed) {
        if (true) {
            // Keep stepping up until we hit the max value.
            rightPow += increment ;
            if (rightPow >= topSpeed ) {
                rightPow = topSpeed;
            }
        }

    }


    /**
     *  Drive in a straight line, on a fixed compass heading (angle), based on encoder counts.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the desired position
     *  2) Driver stops the OpMode running.
     *
     * @param maxDriveSpeed MAX Speed for forward/rev motion (range 0 to +1.0) .
     * @param distance   Distance (in inches) to move from current position.  Negative distance means move backward.
     * @param heading      Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from the current robotHeading.
     */
    public void driveStraight(double maxDriveSpeed,
                              double distance,
                              double heading) {

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            int moveCounts = (int)(distance * COUNTS_PER_INCH);
            leftTarget = leftDrive.getCurrentPosition() + moveCounts;
            rightTarget = rightDrive.getCurrentPosition() + moveCounts;

            // Set Target FIRST, then turn on RUN_TO_POSITION
            leftDrive.setTargetPosition(leftTarget);
            rightDrive.setTargetPosition(rightTarget);

            leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Set the required driving speed  (must be positive for RUN_TO_POSITION)
            // Start driving straight, and then enter the control loop
            maxDriveSpeed = Math.abs(maxDriveSpeed);
            moveRobot(maxDriveSpeed, 0);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (leftDrive.isBusy() && rightDrive.isBusy())) {

                // Determine required steering to keep on heading
                turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    turnSpeed *= -1.0;

                // Apply the turning correction to the current driving speed.
                moveRobot(driveSpeed, turnSpeed);

                // Display drive status for the driver.
                sendTelemetry(true);
            }

            // Stop all motion & Turn off RUN_TO_POSITION
            moveRobot(0, 0);
            leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /**
     *  Spin on the central axis to point in a new direction.
     *  <p>
     *  Move will stop if either of these conditions occur:
     *  <p>
     *  1) Move gets to the heading (angle)
     *  <p>
     *  2) Driver stops the OpMode running.
     *
     * @param maxTurnSpeed Desired MAX speed of turn. (range 0 to +1.0)
     * @param heading Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *              0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *              If a relative angle is required, add/subtract from current heading.
     */
    public void turnToHeading(double maxTurnSpeed, double heading) {

        // Run getSteeringCorrection() once to pre-calculate the current error
        getSteeringCorrection(heading, P_DRIVE_GAIN);

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && (Math.abs(headingError) > HEADING_THRESHOLD)) {

            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -turnSpeed, turnSpeed);

            // Pivot in place by applying the turning correction
            moveRobot(0, turnSpeed);

            // Display drive status for the driver.
            sendTelemetry(false);
        }

        // Stop all motion;
        moveRobot(0, 0);
    }

    /**
     *  Obtain & hold a heading for a finite amount of time
     *  <p>
     *  Move will stop once the requested time has elapsed
     *  <p>
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
     * Use a Proportional Controller to determine how much steering correction is required.
     *
     * @param desiredHeading        The desired absolute heading (relative to last heading reset)
     * @param proportionalGain      Gain factor applied to heading error to obtain turning power.
     * @return                      Turning power needed to get to required heading.
     */
    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        targetHeading = desiredHeading;  // Save for telemetry

        // Determine the heading current error
        headingError = targetHeading - getHeading();

        // Normalize the error to be within +/- 180 degrees
        while (headingError > 180)  headingError -= 360;
        while (headingError <= -180) headingError += 360;

        // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
        return Range.clip(headingError * proportionalGain, -1, 1);
    }

    /**
     * Take separate drive (fwd/rev) and turn (right/left) requests,
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

        leftDrive.setPower(leftSpeed);
        rightDrive.setPower(rightSpeed);
    }

    /**
     *  Display the various control parameters while driving
     *
     * @param straight  Set to true if we are driving straight, and the encoder positions should be included in the telemetry.
     */
    private void sendTelemetry(boolean straight) {

        if (straight) {
            telemetry.addData("Motion", "Drive Straight");
            telemetry.addData("Target Pos L:R",  "%7d:%7d",      leftTarget,  rightTarget);
            telemetry.addData("Actual Pos L:R",  "%7d:%7d",      leftDrive.getCurrentPosition(),
                    rightDrive.getCurrentPosition());
        } else {
            telemetry.addData("Motion", "Turning");
        }

        telemetry.addData("Heading- Target : Current", "%5.2f : %5.0f", targetHeading, getHeading());
        telemetry.addData("Error  : Steer Pwr",  "%5.1f : %5.1f", headingError, turnSpeed);
        telemetry.addData("Wheel Speeds L : R", "%5.2f : %5.2f", leftSpeed, rightSpeed);
        telemetry.update();
    }

    /**
     * read the Robot heading directly from the IMU (in degrees)
     */
    public double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = leftDrive.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightTarget = rightDrive.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            leftDrive.setTargetPosition(newLeftTarget);
            rightDrive.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            leftDrive.setPower(Math.abs(speed));
            rightDrive.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (leftDrive.isBusy() && rightDrive.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Running to", " %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Currently at", " at %7d :%7d",
                        leftDrive.getCurrentPosition(), rightDrive.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            leftDrive.setPower(0);
            rightDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move.
        }

    }
}


