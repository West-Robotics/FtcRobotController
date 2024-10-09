package org.firstinspires.ftc.teamcode.Technofeathers.UntitledRobot.Teleop;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Controller;
import org.firstinspires.ftc.teamcode.Technofeathers.TechnofeathersDrive;

@TeleOp(name = "AutoTesting")
public class AutoTesting extends OpMode {

    //DcMotor frontRight;
    //DcMotor frontLeft;
    //DcMotor backLeft;
    //DcMotor backRight;
    TechnofeathersDrive drive = new TechnofeathersDrive();

    public Controller controller1;

    public Servo grabber;
    public Servo pivot;
    public Servo newgrabber;

    @Override
    public void init(){


        controller1 = new Controller(gamepad1);
        drive.setupMotors(hardwareMap);
        grabber = hardwareMap.get(Servo.class,"grabber");
        pivot = hardwareMap.get(Servo.class,"pivot");
        newgrabber = hardwareMap.get(Servo.class,"newgrabber");

    }

    @Override
    public void loop(){
        controller1.update();

        drive.drive(controller1.left_stick_x, -controller1.left_stick_y, controller1.right_stick_x);
        telemetry.addData("Side: ", controller1.left_stick_x);
        telemetry.addData("Forward: ", controller1.left_stick_y);
        telemetry.addData("Turn: ", controller1.right_stick_x);
        telemetry.addData("pivot", pivot.getPosition());
        telemetry.addData("grabber", grabber.getPosition());
        telemetry.addData("newgrabber",newgrabber.getPosition());
        telemetry.update();

        if (controller1.AOnce()){
            grabber.setPosition(0.15);
        }
        if (controller1.BOnce()){
            grabber.setPosition(0.24);

        }
        if (controller1.XOnce()){
            pivot.setPosition(0.86);
        }
        if (controller1.YOnce()){
            pivot.setPosition(0.55);
        }

        if (controller1.dpadDown()){
            newgrabber.setPosition(0.35);
        }
        if (controller1.dpadUp()){
            newgrabber.setPosition(0.55);
        }
    }
}
