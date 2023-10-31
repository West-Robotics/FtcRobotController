package org.firstinspires.ftc.teamcode.griddle;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp(name = "Griddle TeleOp")

public class GriddleTeleOp extends OpMode {
    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private Gamepad gamepad;
    
    @Override
    public void init() {
        leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");
        rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");
        gamepad = gamepad1;
        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {
        leftMotor.setPower(-gamepad.left_stick_y);
        rightMotor.setPower(-gamepad.left_stick_y);
    }
}
