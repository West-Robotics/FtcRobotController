package org.firstinspires.ftc.teamcode.Technofeathers.Bibot.Teleop.Subsystems;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    public DcMotor intake;
    public void setUpIntake(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotor.class, "intake");
        //intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setZeroPowerBehavior(BRAKE);
    }
    public void rotateForwards() {
        intake.setPower(1);
    }
    public void rotateBackwards() {
        intake.setPower(-1);
    }
    public void off() {
        intake.setPower(0);
    }
}
