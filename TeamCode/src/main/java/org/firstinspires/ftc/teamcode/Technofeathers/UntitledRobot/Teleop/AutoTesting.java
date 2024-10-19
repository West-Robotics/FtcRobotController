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
    public DcMotor arm;
    TechnofeathersDrive drive = new TechnofeathersDrive();

    public Controller controller1;

    public Servo grabber;
    public Servo pivot;
    public Servo newgrabber;
    public boolean buttonA;
    public boolean buttonY;


    @Override
    public void init(){

        arm = hardwareMap.get(DcMotor.class,"arm");
        controller1 = new Controller(gamepad1);
        drive.setupMotors(hardwareMap);
        grabber = hardwareMap.get(Servo.class,"grabber");
        pivot = hardwareMap.get(Servo.class,"pivot");
        arm.setDirection(DcMotorSimple.Direction.FORWARD);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        buttonA = true;
        buttonY = true;
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
        telemetry.update();


        if (controller1.leftBumper()){
            if (buttonA) {
                grabber.setPosition(0.15);
                buttonA = false;
            } else{
                grabber.setPosition(0.24);
                buttonA = true;
            }
        }

        if (controller1.YOnce()){
            if (buttonY){
                pivot.setPosition(0.86);
                buttonY = false;
            } else{
                pivot.setPosition(0.55);
                buttonY = true;
            }
        }
        if (controller1.XOnce()){

        }
        if (controller1.AOnce()){
            pivot.setPosition(0.38);
        }
        if (controller1.BOnce()){

        }

    }
}
