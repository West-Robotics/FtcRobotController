package org.firstinspires.ftc.teamcode.Technofeathers.Gamble.Teleop.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class HorizontalLift {
    public DcMotor horizontalLeftLift;
    public DcMotor horizontalRightLift;

    public void setupMotors(HardwareMap hardwareMap) {
        horizontalLeftLift = hardwareMap.get(DcMotor.class, "horizontalLeftLift");
        horizontalRightLift = hardwareMap.get(DcMotor.class, "horizontalRightLift");

        horizontalLeftLift.setDirection(DcMotorSimple.Direction.FORWARD);
        horizontalRightLift.setDirection(DcMotorSimple.Direction.FORWARD);

        horizontalLeftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        horizontalRightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        horizontalLeftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset the motor encoder
        horizontalRightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset the motor encoder

        horizontalLeftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // Turn the motor back on when we are done
        horizontalRightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // Turn the motor back on when we are done
    }

    public void setLiftPower(double horizontalPower) {
        horizontalLeftLift.setPower(horizontalPower);
        horizontalRightLift.setPower(horizontalPower);
    }
}