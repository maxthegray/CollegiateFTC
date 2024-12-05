package org.firstinspires.ftc.teamcode.YR2;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class Race extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
// Declare our motors
        // Make sure your ID's match your configuration
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightMotor");



        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        double y = 0; // forward amount
        double x = 0; // strafe, *somethign to align
        double rx = 0; // just in case, rotation

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = -(y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;
        if (isStopRequested()) return;

        while (opModeIsActive()) {

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]


            y = 1;
            x = 0.01;
            denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            frontLeftPower = (y + x + rx) / denominator;
            backLeftPower = (y - x + rx) / denominator;
            frontRightPower = -(y - x - rx) / denominator;
            backRightPower = (y + x - rx) / denominator;
            frontLeftMotor.setPower(frontLeftPower*3);
            backLeftMotor.setPower(backLeftPower*3);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);
            sleep(3500);
            y = 0;
            x = 0;
            denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            frontLeftPower = (y + x + rx) / denominator;
            backLeftPower = (y - x + rx) / denominator;
            frontRightPower = -(y - x - rx) / denominator;
            backRightPower = (y + x - rx) / denominator;
            frontLeftMotor.setPower(frontLeftPower*3);
            backLeftMotor.setPower(backLeftPower*3);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);
            sleep(250);
            y = 0;
            x = -1;
            denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            frontLeftPower = (y + x + rx) / denominator;
            backLeftPower = (y - x + rx) / denominator;
            frontRightPower = -(y - x - rx) / denominator;
            backRightPower = (y + x - rx) / denominator;
            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);
            sleep(1000);
            y = 0;
            x = 0;
            denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            frontLeftPower = (y + x + rx) / denominator;
            backLeftPower = (y - x + rx) / denominator;
            frontRightPower = -(y - x - rx) / denominator;
            backRightPower = (y + x - rx) / denominator;
            frontLeftMotor.setPower(frontLeftPower*3);
            backLeftMotor.setPower(backLeftPower*3);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);
            sleep(250);
            x = -0.1;
            y = -1;
            denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            frontLeftPower = (y + x + rx) / denominator;
            backLeftPower = (y - x + rx) / denominator;
            frontRightPower = -(y - x - rx) / denominator;
            backRightPower = (y + x - rx) / denominator;
            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);
            sleep(10000);



            telemetry.addData("FL power", frontLeftMotor.getPower());
            telemetry.addData("BL power", backLeftMotor.getPower());
            telemetry.addData("FR power", frontRightMotor.getPower());
            telemetry.addData("BR power", backRightMotor.getPower());
            telemetry.update();
        }
    }
}