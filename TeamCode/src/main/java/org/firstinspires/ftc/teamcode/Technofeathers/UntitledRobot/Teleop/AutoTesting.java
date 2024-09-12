package org.firstinspires.ftc.teamcode.Technofeathers.UntitledRobot.Teleop;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Controller;

@TeleOp(name = "AutoTesting")
public class AutoTesting extends OpMode {

    DcMotor frontRight;
    DcMotor frontLeft;
    DcMotor backLeft;
    DcMotor backRight;

    public Controller controller1;

    @Override
    public void init(){
        frontRight = hardwareMap.get(DcMotor.class,"backRight");
        frontLeft = hardwareMap.get(DcMotor.class,"backLeft");
        backLeft = hardwareMap.get(DcMotor.class,"frontLeft");
        backRight = hardwareMap.get(DcMotor.class,"frontRight");

        controller1 = new Controller(gamepad1);
    }

    @Override
    public void loop(){
        controller1.update();
        if (controller1.A()){
            backLeft.setPower(1);
        }
        if (controller1.B()){
            backRight.setPower(1);
        }
        if (controller1.Y()){
            frontRight.setPower(1);
        }
        if (controller1.X()){
            frontLeft.setPower(1);
        }
    }
}
