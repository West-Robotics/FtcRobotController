package org.firstinspires.ftc.teamcode.Technofeathers.UntitledRobot.Teleop;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Controller;
import org.firstinspires.ftc.teamcode.Technofeathers.TechnofeathersDrive;

@TeleOp(name = "AutoTesting")
public class AutoTesting extends OpMode {

    DcMotor frontRight;
    DcMotor frontLeft;
    DcMotor backLeft;
    DcMotor backRight;
    TechnofeathersDrive drive = new TechnofeathersDrive();

    public Controller controller1;

    @Override
    public void init(){

        /*
        frontRight = hardwareMap.get(DcMotor.class,"backRight");
        frontLeft = hardwareMap.get(DcMotor.class,"backLeft");
        backLeft = hardwareMap.get(DcMotor.class,"frontLeft");
        backRight = hardwareMap.get(DcMotor.class,"frontRight");
        */
        controller1 = new Controller(gamepad1);
        drive.setupMotors(hardwareMap);


    }

    @Override
    public void loop(){
        controller1.update();

        drive.drive(controller1.left_stick_x, controller1.left_stick_y, controller1.right_stick_x);


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
        } else{
            backLeft.setPower(0);
            backRight.setPower(0);
            frontLeft.setPower(0);
            frontRight.setPower(0);
        }
    }
}
