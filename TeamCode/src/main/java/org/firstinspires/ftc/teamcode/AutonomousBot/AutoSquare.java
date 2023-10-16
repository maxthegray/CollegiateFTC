

/* Copyright (c) 2018 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode.AutonomousBot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;




@Autonomous(name="ChairTrack: Colorsensor", group="Robot")
//@Disabled
public class AutoSquare extends LinearOpMode {
    private DistanceSensor sensorDistanceFront;
    private DistanceSensor sensorDistanceLeft;

    /* Declare OpMode members. */
    private DcMotor         leftDrive   = null;
    private DcMotor         rightDrive  = null;
    private ElapsedTime     runtime = new ElapsedTime();

    NormalizedColorSensor colorSensor;

    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = .75;
    static final double     TURN_SPEED              = 0.5;
    private int i;

    static final double GROUND_BRIGHTNESS = 0.85;  // spans between 0.0 - 1.0 from dark to light


    @Override
    public void runOpMode() {

        // Initialize the drive system variables.
        leftDrive = hardwareMap.get(DcMotor.class, "leftBackMotor");
        rightDrive = hardwareMap.get(DcMotor.class, "rightBackMotor");
        sensorDistanceLeft = hardwareMap.get(DistanceSensor.class, "sensor_distance2");
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point 4n opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Starting at", "%7d :%7d",
                leftDrive.getCurrentPosition(),
                rightDrive.getCurrentPosition());
        telemetry.addData("Starting at", "%7d :%7d");
        getBrightness();
        telemetry.update();

        // you can use this as a regular DistanceSensor.
//        sensorDistanceFront = hardwareMap.get(DistanceSensor.class, "sensor_distance");


        telemetry.update();

        if (colorSensor instanceof SwitchableLight) {
            ((SwitchableLight) colorSensor).enableLight(true);
        }

        // Wait for the game to start (driver presses PLAY)


        while (opModeInInit()) {
            getBrightness();
        }
        waitForStart();
        colorSensor.setGain(0);


        while (allowedToMove()) {
            getBrightness();
            if (tapeIsntThere()) {
                moveForward(4);
            } else {
                goAroundStool();
                moveForward(84);
            }
        }

    }

    private boolean allowedToMove() {
        return opModeIsActive();
    }
//Color Stuff
    private boolean shouldTurnLeftcolor() {
        return !tapeIsntThere() && !wallOnTheLeft();
    }

    private boolean shouldTurnRightcolor() {
        return !tapeIsntThere();}

    private boolean tapeIsntThere() {
        return getBrightness() > GROUND_BRIGHTNESS;
    }
    private void goAroundStool() {
        moveForward(18);
        turnLeftcolor(7);
        moveForward(18);
        turnLeftcolor(7);
    }
    private void turnRightcolor() {
        encoderDrive(TURN_SPEED, 7, -7, 0.25);
    }

    private void turnLeftcolor(int amount) {
        encoderDrive(TURN_SPEED, -amount, amount, 0.25);
    }
    double getBrightness() {
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        telemetry.addData("Light Level (0 to 1)",  "%4.2f", colors.alpha);
        telemetry.update();

        return colors.alpha;
    }
    //Not Color Stuff
    private boolean shouldMoveForward() {
        return enoughDistanceInFront() && wallOnTheLeft();
   }

    private boolean shouldTurnLeft() {
        return enoughDistanceInFront() && !wallOnTheLeft();
    }

    private boolean shouldTurnRight() {
        return !shouldMoveForward();
    }


    private boolean enoughDistanceInFront() {
        return sensorDistanceFront.getDistance(DistanceUnit.INCH) >= 16;
    }
    private boolean wallOnTheLeft() {
        return sensorDistanceLeft.getDistance(DistanceUnit.INCH) <= 6;
    }
//
    private void moveForward(int distance) {
        encoderDrive(DRIVE_SPEED, distance, distance, 0.25);
    }
//    private void turnRight90Degrees() {
//        encoderDrive(TURN_SPEED, 7, -7, 0.25);
//    }
//
//    private void turnLeft90Degrees() {
//        encoderDrive(TURN_SPEED, -7, 7, 0.25);
//    }
//


    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the OpMode running.
     */
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = leftDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = rightDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
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
                telemetry.addData("Running to",  " %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Currently at",  " at %7d :%7d",
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
