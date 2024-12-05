package org.firstinspires.ftc.teamcode.YR2;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.CollegiateLibs.core;
import org.firstinspires.ftc.teamcode.CollegiateLibs.Arm;


@TeleOp
public class BasicDrive extends LinearOpMode {



    @Override
    public void runOpMode() throws InterruptedException {


        if (isStopRequested()) return;
        core.init();

        while (opModeIsActive()) {

            Arm.SynchronizeSlideAndWrist();

            core.drive();

            core.liftBucket(core.RSy1);

            core.rotateWrist(core.LSy2/2);

            core.spinClaw(core.RSy2);

            core.Slide(core.RT2, core.LT2);

            core.bucketSwitch(core.DpadUp2, core.DpadDown2);



            telemetry.addData("arm motor power", core.armMotor.getPower());
            telemetry.addData("FL power", core.frontLeftMotor.getPower());
            telemetry.addData("BL power", core.backLeftMotor.getPower());
            telemetry.addData("FR power", core.frontRightMotor.getPower());
            telemetry.addData("BR power", core.backRightMotor.getPower());
            telemetry.addData("claw power", core.claw.getPower());


            telemetry.update();

        }

        }


}
