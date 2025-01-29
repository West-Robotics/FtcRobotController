package org.firstinspires.ftc.teamcode.Technofeathers.Gamble.Teleop.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class VerticalLift {
    public DcMotor verticalLeftLift;
    public DcMotor verticalRightLift;
    public void setupMotors(HardwareMap hardwareMap) {
        verticalLeftLift = hardwareMap.get(DcMotor.class,  "verticalLeftLift");
        verticalRightLift = hardwareMap.get(DcMotor.class, "verticalRightLift");

        verticalLeftLift.setDirection(DcMotorSimple.Direction.FORWARD);
        verticalRightLift.setDirection(DcMotorSimple.Direction.REVERSE);

        verticalLeftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        verticalRightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        verticalLeftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset the motor encoder
        verticalRightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset the motor encoder

        verticalLeftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // Turn the motor back on when we are done
        verticalRightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // Turn the motor back on when we are done
    }
    public void setLiftPower(double power) {
        verticalLeftLift.setPower(power);
        verticalRightLift.setPower(power);
    }
}

