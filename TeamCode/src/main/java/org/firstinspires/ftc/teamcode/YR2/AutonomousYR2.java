package org.firstinspires.ftc.teamcode.YR2;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.CollegiateLibs.GoBildaPinpointDriver;

import java.util.Locale;

@TeleOp
public class AutonomousYR2 extends LinearOpMode {

    @Override

    public void runOpMode() throws InterruptedException {
// Declare our motors
        // Make sure your ID's match your configuration
        GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Computer

        double oldTime = 0;
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        odo.setOffsets(10, 9); //these are tuned for 3110-0002-0001 Product Insight #1

        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        odo.resetPosAndIMU();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("X offset", odo.getXOffset());
        telemetry.addData("Y offset", odo.getYOffset());
        telemetry.addData("Device Version Number:", odo.getDeviceVersion());
        telemetry.addData("Device Scalar", odo.getYawScalar());
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();
        resetRuntime();


        // run until the end of the match (driver presses STOP)


            double newTime = getRuntime();
            double loopTime = newTime - oldTime;
            double frequency = 1 / loopTime;
            oldTime = newTime;


            Pose2D pos = odo.getPosition();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Position", data);

            Pose2D vel = odo.getVelocity();
            String velocity = String.format(Locale.US, "{XVel: %.3f, YVel: %.3f, HVel: %.3f}", vel.getX(DistanceUnit.MM), vel.getY(DistanceUnit.MM), vel.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Velocity", velocity);

            DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
            DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
            DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
            DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightMotor");



            frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);


            while (opModeIsActive()) {

                while (odo.getPosX() > -1000) {
                    frontLeftMotor.setPower(0.25);
                    backLeftMotor.setPower(0.25);
                    frontRightMotor.setPower(0.25);
                    backRightMotor.setPower(0.25);
                    telemetry.addData("Status", odo.getDeviceStatus());

                    telemetry.addData("Pinpoint Frequency", odo.getFrequency()); //prints/gets the current refresh rate of the Pinpoint
                    telemetry.addData("PosX", odo.getPosX());

                    telemetry.addData("PosY", odo.getPosY()); //prints/gets the current refresh rate of the Pinpoint

                    telemetry.addData("REV Hub Frequency: ", frequency); //prints the control system refresh rate
                    telemetry.update();
                    odo.update();
                }
                while (odo.getPosY() < 311) {
                    frontLeftMotor.setPower(-0.25);
                    backLeftMotor.setPower(0.25);
                    frontRightMotor.setPower(0.25);
                    backRightMotor.setPower(-0.25);
                    telemetry.addData("Status", odo.getDeviceStatus());

                    telemetry.addData("Pinpoint Frequency", odo.getFrequency()); //prints/gets the current refresh rate of the Pinpoint
                    telemetry.addData("PosX", odo.getPosX());

                    telemetry.addData("PosY", odo.getPosY()); //prints/gets the current refresh rate of the Pinpoint

                    telemetry.addData("REV Hub Frequency: ", frequency); //prints the control system refresh rate
                    telemetry.update();
                    odo.update();
                }
                if (odo.getPosY() >= 311 && odo.getPosX() <= -1000) {
                    //grab a puck with the claw and extend the arm

                }

                    frontLeftMotor.setPower(0);
                    backLeftMotor.setPower(0);
                    frontRightMotor.setPower(0);
                    backRightMotor.setPower(0);

                    odo.update();



                odo.update();
                // Denominator is the largest motor power (absolute value) or 1
                // This ensures all the powers maintain the same ratio,
                // but only if at least one is out of the range [-1, 1]


                telemetry.addData("Status", odo.getDeviceStatus());

                telemetry.addData("Pinpoint Frequency", odo.getFrequency()); //prints/gets the current refresh rate of the Pinpoint
                telemetry.addData("PosX", odo.getPosX());

                telemetry.addData("PosY", odo.getPosY()); //prints/gets the current refresh rate of the Pinpoint

                telemetry.addData("REV Hub Frequency: ", frequency); //prints the control system refresh rate
                telemetry.update();

            }
        }
    }