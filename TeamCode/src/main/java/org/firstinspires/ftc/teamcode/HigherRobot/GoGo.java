
package org.firstinspires.ftc.teamcode.HigherRobot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "Drive round and stuff", group = "1stSemester")
public class GoGo extends LinearOpMode {

    static final int CYCLE_MS = 500;     // period of each cycle

    // Define class members
    Servo servo;

    TouchSensor touchSensor;
    DistanceSensor sensorDistance;

    private DcMotor arm1 = null;
    private DcMotor arm2 = null;


    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private ElapsedTime runtime = new ElapsedTime();


    static final double     COUNTS_PER_MOTOR_REV    = 2240 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;
    int stage = 0;

    int leftParkCo = 0;
    int rightParkCo = 0;
    double pos = 0.15;



    @Override
    public void runOpMode() {

        servo = hardwareMap.get(Servo.class, "claw");

        arm1 = hardwareMap.get(DcMotor.class, "arm1");
        arm2 = hardwareMap.get(DcMotor.class, "arm2");

        leftDrive = hardwareMap.get(DcMotor.class, "left");
        rightDrive = hardwareMap.get(DcMotor.class, "right");

        touchSensor = hardwareMap.get(TouchSensor.class, "sensor_touch");
        sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor");

        arm1.setDirection(DcMotorSimple.Direction.FORWARD);
        arm2.setDirection(DcMotorSimple.Direction.FORWARD);

        leftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        while(opModeInInit()) {
            telemetry.addData("Servo Position", "%5.2f", servo.getPosition());
            telemetry.addData("Wheels Starting at",  "%7d :%7d",
                    leftDrive.getCurrentPosition(),
                    rightDrive.getCurrentPosition());
            telemetry.addData("Distance", "%5.2f", sensorDistance.getDistance(DistanceUnit.INCH));
            telemetry.update();
        }
        waitForStart();
        telemetry.addData("servo", servo.getPosition());
        telemetry.update();
        int stage = 1;
        int counter = 0;
        while (opModeIsActive()) {
            while (stage == 1 && opModeIsActive()) {
                  if (sensorDistance.getDistance(DistanceUnit.INCH) < 17 && counter == 0) {
                      leftDrive.setPower(0.2);
                      rightDrive.setPower(-0.5);
                      counter += 1;
                      sleep(3250);
                      leftDrive.setPower(0.3);
                      rightDrive.setPower(0.3);
                      leftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
                      rightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
                  }

                  if (touchSensor.isPressed() && opModeIsActive()) {
                      leftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
                      rightDrive.setDirection(DcMotorSimple.Direction.FORWARD);
                      leftDrive.setPower(0);
                      rightDrive.setPower(0);
                      stage = 2;
                  }
                  else {
                      leftDrive.setPower(5);
                      rightDrive.setPower(5);
                  }
                }

            while (stage == 2 && opModeIsActive()) {
                if (gamepad1.square) {
                    clawOpen(0.05);
                }
                if (gamepad1.circle) {
                    clawClose(0.02);
                }
                if (gamepad1.triangle) {
                    while (gamepad1.triangle && opModeIsActive()) {
                        armUp(0.4);
                    }
                }
                if (gamepad1.cross) {
                    while (gamepad1.cross && opModeIsActive()) {
                        armDown(0.4);
                    }
                }
                if (gamepad1.dpad_right) {
                    while (gamepad1.dpad_right && opModeIsActive()) {
                        leftDrive.setPower(0.7);
                        rightDrive.setPower(-0.6);
                    }
                }
                if (gamepad1.dpad_left) {
                    while (gamepad1.dpad_left) {
                        leftDrive.setPower(-0.6);
                        rightDrive.setPower(0.7);
                    }
                }
                if (gamepad1.dpad_up) {
                    while (gamepad1.dpad_up) {
                        leftDrive.setPower(0.7);
                        rightDrive.setPower(0.7);
                    }
                }
                if (gamepad1.dpad_down) {
                    while (gamepad1.dpad_down) {
                        leftDrive.setPower(-0.8);
                        rightDrive.setPower(-0.8);
                    }
                }
                    arm1.setPower(0);
                    arm2.setPower(0);
                    leftDrive.setPower(0);
                    rightDrive.setPower(0);
                    telemetry.addData("going down", "false");
                    telemetry.update();
                }
            }
            telemetry.update();

        }

    private void armDown(double speed) {
        arm1.setPower(speed);
        arm2.setPower(speed);
        telemetry.addData("going down", "true");

    }

    private void armUp(double speed) {
        arm1.setPower(-speed);
        arm2.setPower(-speed);
        telemetry.addData("going up", "true");
    }

    private void clawOpen(double inc) {
        while (gamepad1.square) {
            if (pos <= 0.40) {
                pos += inc;
            }
            servo.setPosition(pos);
            telemetry.addData("claw", "opening");
            telemetry.addData("Servo Position", "%5.2f", servo.getPosition());
            telemetry.update();
            sleep(50);
        }
    }
    private void clawClose(double inc) {
        while(gamepad1.circle) {
            if (pos >= 0.01) {
                pos -= inc;
            }
            servo.setPosition(pos);
            telemetry.addData("claw", "closing");
            telemetry.addData("Servo Position", "%5.2f", servo.getPosition());
            telemetry.update();
            sleep(50);
        }

    }
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