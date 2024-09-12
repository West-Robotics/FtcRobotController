package org.firstinspires.ftc.teamcode.Technofeathers.Teleop;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Controller;

@TeleOp(name = "autotesting")
public class autotesting extends OpMode {

    DcMotor frontright;
    DcMotor frontleft;
    DcMotor backleft;
    DcMotor backright;

    public Controller controller1;

    @Override
    public void init(){
        frontright = hardwareMap.get(DcMotor.class,"frontRight");
        frontleft = hardwareMap.get(DcMotor.class,"frontLeft");
        backleft = hardwareMap.get(DcMotor.class,"backLeft");
        backright = hardwareMap.get(DcMotor.class,"backRight");

        controller1 = new Controller(gamepad1);

        backright.setDirection(DcMotorSimple.Direction.REVERSE);
        frontright.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop(){
        controller1.update();
        frontright.setPower(controller1.right_stick_y);
        backright.setPower(controller1.right_stick_y);
        backleft.setPower(controller1.right_stick_y);
        frontleft.setPower(controller1.right_stick_y);

        if (controller1.A()){
            backleft.setPower(1);
        }
        if (controller1.B()){
            backright.setPower(1);
        }
        if (controller1.Y()){
            frontright.setPower(1);
        }
        if (controller1.X()){
            frontleft.setPower(1);
        } else {
            frontleft.setPower(0);
            backleft.setPower(0);
            frontright.setPower(0);
            backright.setPower(0);
        }
    }
}
