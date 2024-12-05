package org.firstinspires.ftc.teamcode.CollegiateLibs;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public class robotcore {

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
}