package org.firstinspires.ftc.teamcode.CollegiateLibs;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;



public class Base {

    private static LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.
    public static CRServo claw;
    public static Servo bucket;
    public static DcMotor frontLeftMotor;
    public static DcMotor backLeftMotor;
    public static DcMotor frontRightMotor;
    public static DcMotor backRightMotor;
    public static DcMotor wristMotor;
    public static DcMotor armMotor;
    public static DcMotor bucketLift;

    public static double RSy1 = myOpMode.gamepad1.right_stick_y;
    public static double RSy2 = myOpMode.gamepad2.right_stick_y;
    public static double LSy1 = myOpMode.gamepad1.left_stick_y;
    public static double LSy2 = myOpMode.gamepad2.left_stick_y;
    public static double RT1 = myOpMode.gamepad1.right_trigger;
    public static double RT2 = myOpMode.gamepad2.right_trigger;
    public static double LT1 = myOpMode.gamepad1.left_trigger;
    public static double LT2 = myOpMode.gamepad2.left_trigger;

    public static boolean DpadUp1 = myOpMode.gamepad1.dpad_up;
    public static boolean DpadUp2 =myOpMode.gamepad2.dpad_up;
    public static boolean DpadDown1 = myOpMode.gamepad1.dpad_down;
    public static boolean DpadDown2 = myOpMode.gamepad2.dpad_down;

    public static void init() {
        claw = myOpMode.hardwareMap.crservo.get("claw");

        bucket = myOpMode.hardwareMap.servo.get("bucket");

        frontLeftMotor = myOpMode.hardwareMap.dcMotor.get("frontLeftMotor");
        backLeftMotor = myOpMode.hardwareMap.dcMotor.get("backLeftMotor");
        frontRightMotor = myOpMode.hardwareMap.dcMotor.get("frontRightMotor");
        backRightMotor = myOpMode.hardwareMap.dcMotor.get("backRightMotor");

        wristMotor = myOpMode.hardwareMap.dcMotor.get("bucketWrist");
        armMotor = myOpMode.hardwareMap.dcMotor.get("ArmMotor");
        bucketLift = myOpMode.hardwareMap.dcMotor.get("testTetrix");

        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wristMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public static void drive() {
        double y = -myOpMode.gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = -myOpMode.gamepad1.left_stick_x * 2; // Counteract imperfect strafing
        double rx = -myOpMode.gamepad1.right_stick_x;
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = -(y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);
    }
    public static void Slide(double racialSlur, double jesusChrist) {
        armMotor.setPower(racialSlur - jesusChrist);
    }
    public static void bucketSwitch(boolean upBind, boolean downBind) {
        if (upBind) {
            bucket.setPosition(0.6);
        } else if (downBind) {
            bucket.setPosition(0.4);
        }

    }
    public static void spinClaw(double power) {
        claw.setPower(power);
    }
    public static void liftBucket(double power) {
        bucketLift.setPower(power);
    }
    public static void rotateWrist(double power) {
        wristMotor.setPower(power);
    }
}