package org.firstinspires.ftc.teamcode.CollegiateLibs;

public class Arm {
    static double speed = core.RT1 - core.LT1;
    static double wristSpeed = speed/4;
    static double slideSpeed = speed/2;
    public static void SynchronizeSlideAndWrist() {
        core.wristMotor.setPower(wristSpeed);
        core.armMotor.setPower(slideSpeed);
    }
}
