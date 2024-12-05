
package org.firstinspires.ftc.teamcode.HigherRobot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Autonomous(name = "Centerstage", group = "2ndSemester")
public class Centerstage extends LinearOpMode {

    // Define class members
    Servo wristServo;
    Servo clawServo;
    Servo plane;
    private DcMotor armMotor1 = null;
    private DcMotor armMotor2 = null;
    private DcMotor hangMotor = null;



    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private ElapsedTime runtime = new ElapsedTime();


    static final double COUNTS_PER_MOTOR_REV = 2240;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // No External Gearing.
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    double wristpos = 0.0;


    @Override
    public void runOpMode() {

        wristServo = hardwareMap.get(Servo.class, "wrist1");
        plane = hardwareMap.get(Servo.class, "plane");
        clawServo = hardwareMap.get(Servo.class, "claw");
        wristServo.setDirection(Servo.Direction.FORWARD);
        plane.setDirection(Servo.Direction.REVERSE);

        armMotor1 = hardwareMap.get(DcMotor.class, "arm1");
        armMotor2 = hardwareMap.get(DcMotor.class, "arm2");
//        arm1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        arm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armMotor1.setDirection(DcMotorSimple.Direction.FORWARD);
        armMotor2.setDirection(DcMotorSimple.Direction.FORWARD);

        hangMotor = hardwareMap.get(DcMotor.class, "hangMotor");
        hangMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftDrive = hardwareMap.get(DcMotor.class, "left");
        rightDrive = hardwareMap.get(DcMotor.class, "right");

        leftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        while (opModeInInit()) {
            telemetry.addData("wrist1 Position", wristServo.getPosition());
            telemetry.addData("plane pos", plane.getPosition());
            plane.setPosition(0.5);
        }

        waitForStart();
        while (opModeIsActive()) {

            if (gamepad2.circle) {
                clawOpen(0.05);
            }
            else if (gamepad2.square) {
                clawClose(0.05);
            }

            //wrist funcs
            double wristPower = (Range.clip(gamepad2.right_stick_y, -0.25, 0.25))/100 ;

            wristServo.setPosition(wristServo.getPosition() + wristPower);

            if(gamepad1.left_bumper) {
                launchPlane();
            }
            //Hang Funcs
            if(gamepad1.triangle) {
                hangUp(0.5);
            }
            else if(gamepad1.cross) {
                hangUp(-1);
            }
            else {
                hangUp(0);
            }
            //Arm Things
            double armPower = Range.clip(gamepad2.left_stick_y, -0.25, 0.25) ;



            armMotor1.setPower(armPower);
            armMotor2.setPower(armPower);


//            double rad = 30;
//            while(gamepad2.left_stick_button) {
//                rad += 0.30;
//                sleep(1);
//                armMotor1.setPower(armPower);
//                armMotor2.setPower(armPower);
//            }

//            if (gamepad1.triangle) {
//                armUp(0.001);
//            }
//            else if (gamepad1.cross) {
//                armDown(0.001);
//            }
//            else {
//                arm1.setPower(0);
//                arm2.setPower(0);
//                armRampSpeed = 0.2;
//                arm1.setDirection(DcMotorSimple.Direction.FORWARD);
//                arm2.setDirection(DcMotorSimple.Direction.FORWARD);


            //Driving Funcs
            double leftPower;
            double rightPower;

            double drive = -gamepad1.left_stick_y;
            double turn  =  gamepad1.left_stick_x;
            leftPower    = Range.clip(drive + turn, -0.5, 0.5) ;
            rightPower   = Range.clip(drive - turn, -0.5, 0.5) ;

            leftDrive.setPower(leftPower);
            rightDrive.setPower(rightPower);



            telemetry.addData("Arm Motor 1 Pos", armMotor1.getCurrentPosition());
            telemetry.addData("Arm Motor 2 Pos", armMotor2.getCurrentPosition());

            telemetry.addData("Claw Servo Position", clawServo.getPosition());
            telemetry.addData("WristPower", clawServo.getPosition());

            telemetry.addData("Wrist Servo Position", wristServo.getPosition());



            telemetry.update();
        }

    }

    private void hangUp(double power) {
        hangMotor.setPower(power);
    }
    private void clawOpen(double inc) {

        if (clawServo.getPosition() < 0.8) {
            clawServo.setPosition(clawServo.getPosition() + inc);
            sleep(50);
        }
    }
    private void clawClose(double inc) {
        if (clawServo.getPosition() > 0.15) {
            clawServo.setPosition(clawServo.getPosition() - inc);
            sleep(50);
        }

    }
    private void wristUp(double inc) {
        while (gamepad2.square) {
            if (wristpos <= 0.90) {
                wristpos += inc;
            }
            wristServo.setPosition(wristpos);

            telemetry.addData("Servo Position", "%5.2f", wristServo.getPosition());
            telemetry.update();
            sleep(50);
        }
    }
    private void wristDown(double inc) {
        while(gamepad2.circle) {
            if (wristpos >= 0.01) {
                wristpos -= inc;
            }
            wristServo.setPosition(wristpos);

            telemetry.addData("Servo Position", "%5.2f", wristServo.getPosition());
            telemetry.update();
            sleep(50);
        }

    }
    private void launchPlane() {
        if (plane.getPosition() == 1) {
            plane.setPosition(0.5);
        } else {
            plane.setPosition(1);
        }
        sleep(100);

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