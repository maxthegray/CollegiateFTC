package org.firstinspires.ftc.teamcode.YR2;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.CollegiateLibs.Base;
import org.firstinspires.ftc.teamcode.CollegiateLibs.robotcore;

import org.firstinspires.ftc.teamcode.CollegiateLibs.GoBildaPinpointDriver;

@TeleOp
public class BasicDrive1 extends LinearOpMode {



    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration

        //        Servo claw = hardwareMap.servo.get("claw");CRServo


        IMU imu = hardwareMap.get(IMU.class, "imu");


        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.




        GoBildaPinpointDriver odo = null; // Declare OpMode member for the Odometry Computer

        //odo stuff
        double oldTime = 0;


        telemetry.addData("Status", "Initialized");
        telemetry.addData("X offset", odo.getXOffset());
        telemetry.addData("Y offset", odo.getYOffset());
        telemetry.addData("Device Version Number:", odo.getDeviceVersion());
        telemetry.addData("Device Scalar", odo.getYawScalar());
        telemetry.update();



        waitForStart();
        odo.resetPosAndIMU(); //resets the position to 0 and recalibrates the IMU
        odo.recalibrateIMU(); //recalibrates the IMU without resetting position
        imu.resetYaw();



        if (isStopRequested()) return;

        while (opModeIsActive()) {
            Base.init();

            Base.drive();

            Base.liftBucket(Base.RSy1);

            Base.rotateWrist(Base.LSy2/2);

            Base.spinClaw(Base.RSy2);

            Base.Slide(Base.RT2, Base.LT2);

            Base.bucketSwitch(Base.DpadUp2, Base.DpadDown2);



            telemetry.addData("arm motor power", Base.armMotor.getPower());
            telemetry.addData("FL power", Base.frontLeftMotor.getPower());
            telemetry.addData("BL power", Base.backLeftMotor.getPower());
            telemetry.addData("FR power", Base.frontRightMotor.getPower());
            telemetry.addData("BR power", Base.backRightMotor.getPower());
            telemetry.addData("claw power", Base.claw.getPower());

            telemetry.addData("Status", odo.getDeviceStatus());

            telemetry.addData("Pinpoint Frequency", odo.getFrequency()); //prints/gets the current refresh rate of the Pinpoint
            telemetry.addData("PosX", odo.getPosX());

            telemetry.addData("PosY", odo.getPosY()); //prints/gets the current refresh rate of the Pinpoint


            telemetry.update();

        }

        }


}
