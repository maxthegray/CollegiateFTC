//package org.firstinspires.ftc.teamcode.AutonomousBot;
//
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.hardware.IMU;
//import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
//public class Robot {
//    public final DcMotor leftDrive;
//    public final DcMotor rightDrive;
//    public final IMU imu;
//    public final NormalizedColorSensor colorSensor1;
//    public final NormalizedColorSensor colorSensor2;
//
//    private final MovementStrategy movement;
//
//    public Robot(HardwareMap hardwareMap, MovementStrategy movement) {
//        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
//        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
//        imu = hardwareMap.get(IMU.class, "imu");
//        colorSensor1 = hardwareMap.get(NormalizedColorSensor.class, "sensor_color1");
//        colorSensor2 = hardwareMap.get(NormalizedColorSensor.class, "sensor_color2");
//        this.movement = movement;
//    }
//
//    public void move() {
//        movement.move(this);
//    }
//}
