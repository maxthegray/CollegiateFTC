package org.firstinspires.ftc.teamcode.YR2;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class Proofs extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
// Declare our motors
        // Make sure your ID's match your configuration
        DcMotor liftMotor = hardwareMap.dcMotor.get("ArmMotor");

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {


            liftMotor.setPower(gamepad1.right_stick_y);




        }
    }
}