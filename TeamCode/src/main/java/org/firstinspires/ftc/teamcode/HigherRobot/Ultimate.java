
package org.firstinspires.ftc.teamcode.HigherRobot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "We are so cool", group = "1stSemester")
public class Ultimate extends LinearOpMode {

    static final int CYCLE_MS = 500;     // period of each cycle

    // Define class members
    Servo servo;

    TouchSensor touchSensor;

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



    @Override
    public void runOpMode() {

        servo = hardwareMap.get(Servo.class, "claw");

        arm1 = hardwareMap.get(DcMotor.class, "arm1");
        arm2 = hardwareMap.get(DcMotor.class, "arm2");

        leftDrive = hardwareMap.get(DcMotor.class, "left");
        rightDrive = hardwareMap.get(DcMotor.class, "right");

        touchSensor = hardwareMap.get(TouchSensor.class, "sensor_touch");

        arm1.setDirection(DcMotorSimple.Direction.FORWARD);
        arm2.setDirection(DcMotorSimple.Direction.REVERSE);

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
            telemetry.update();
        }
        waitForStart();
        telemetry.addData("servo", servo.getPosition());
        telemetry.update();

        while (opModeIsActive()) {
            if (touchSensor.isPressed()) {
                telemetry.addData("Touch Sensor", "Is Pressed");
            }
            if (gamepad1.a) {
                armUp(0.25, 2600);
                clawOpen();
            }
            else if (gamepad1.b) {
                armDown(0.25, 2600);
                clawClose();
            }
            if (gamepad1.dpad_right) {
                encoderDrive(TURN_SPEED,   12, -12, 1.0);
            }
            else if (gamepad1.dpad_left) {
                encoderDrive(TURN_SPEED, -12, 12, 1);
            }

        }
    }

    private void armDown(double speed, long time) {
        arm1.setPower(speed);
        arm2.setPower(speed);
        telemetry.addData("going", "true");
        telemetry.update();
        sleep(time);
        arm1.setPower(0);
        arm2.setPower(0);
        telemetry.addData("going", "false");
    }

    private void armUp(double speed, long time) {
        arm1.setPower(-speed);
        arm2.setPower(-speed);
        telemetry.addData("going", "true");
        telemetry.update();
        sleep(time);
        arm1.setPower(0);
        arm2.setPower(0);
        telemetry.addData("going", "false");
    }

    private void clawOpen() {
        servo.setPosition(0.45);
        telemetry.addData("claw", "open");
        telemetry.addData("Servo Position", "%5.2f", servo.getPosition());
        telemetry.update();
    }
    private void clawClose() {
        servo.setPosition(0.10);
        telemetry.addData("claw", "close");
        telemetry.addData("Servo Position", "%5.2f", servo.getPosition());
        telemetry.update();
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