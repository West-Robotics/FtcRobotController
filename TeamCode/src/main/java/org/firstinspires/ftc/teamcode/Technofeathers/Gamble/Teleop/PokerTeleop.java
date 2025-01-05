package org.firstinspires.ftc.teamcode.Technofeathers.Gamble.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;



import org.firstinspires.ftc.teamcode.Controller;
import org.firstinspires.ftc.teamcode.Technofeathers.Gamble.Teleop.Subsystems.HorizontalLift;
import org.firstinspires.ftc.teamcode.Technofeathers.Gamble.Teleop.Subsystems.VerticalLift;
import org.firstinspires.ftc.teamcode.Technofeathers.TechnofeathersDrive;

@TeleOp
public class PokerTeleop extends OpMode {

    public Controller controller1;
    public Controller controller2;
    public TechnofeathersDrive drive;
    public HorizontalLift horizontalLift;
    public VerticalLift verticalLift = new VerticalLift();

    public Servo diffyRotatorLeft;
    public Servo diffyRotatorRight;
    public Servo linkageServoLeft;
    public Servo linkageServoRight;

    public Servo pivotSlide;
    public Servo pivotClaw;

    public Servo grabber;
    public Servo horizontalgrabber;

    public boolean buttonA;

    public boolean buttonY;

    public boolean buttonBforcontroller1;

    @Override
    public void init(){
        drive = new TechnofeathersDrive();
        controller1 = new Controller(gamepad1);
        controller2 = new Controller(gamepad2);
        drive.setupMotors(hardwareMap);
        //horizontalLift.setupMotors(hardwareMap);
        verticalLift.setupMotors(hardwareMap);
        diffyRotatorLeft = hardwareMap.get(Servo.class, "diffyRotatorLeft");
        diffyRotatorRight = hardwareMap.get(Servo.class, "diffyRotatorRight");
        linkageServoLeft = hardwareMap.get(Servo.class, "linkageServoLeft");
        linkageServoRight = hardwareMap.get(Servo.class, "linkageServoRight");
        pivotSlide = hardwareMap.get(Servo.class, "diffyRotatorLeft");
        pivotClaw = hardwareMap.get(Servo.class, "diffyRotatorLeft");
        grabber = hardwareMap.get(Servo.class, "diffyRotatorLeft");
        horizontalgrabber = hardwareMap.get(Servo.class,"diffyRotaterLeft");
        buttonA = false;
        buttonY = false;
        buttonBforcontroller1 = false;

    }

    @Override
    public void loop(){
        controller1.update();
        controller2.update();
        drive.drive(controller1.left_stick_x, -controller1.left_stick_y/1.25, controller1.right_stick_x/1.25);
        //horizontalLift.setLiftPower(controller2.left_stick_y); //power for horizontal lift, left stick
        verticalLift.setLiftPower(controller2.left_stick_y/2); //power for vertical lift, right stick

        if (controller1.BOnce()){
            if (buttonBforcontroller1){
                horizontalgrabber.setPosition(0.5);
                buttonBforcontroller1 = false;
            } else{
                horizontalgrabber.setPosition(0.75);
                buttonBforcontroller1 = true;
            }
        }





        // controller 2

        //linkage/horizontal slides
        if(controller2.AOnce()) {
            if (buttonA) {
                linkageServoLeft.setPosition(0.5);
                linkageServoRight.setPosition(0.5);
                buttonA = false;
            } else {
                linkageServoLeft.setPosition(0.75);
                linkageServoRight.setPosition(0.75);
                buttonA = true;
            }

        }

        //pivot to align specimen
        if (controller2.BOnce()){
            pivotSlide.setPosition(0.5);
            pivotClaw.setPosition(0.5);

        }

        //vertical grabber
        if (controller2.YOnce()){
            if (buttonY){
                grabber.setPosition(0.5);
                buttonY = false;
            } else{
                grabber.setPosition(0.75);
                buttonY = true;
            }
        }


    }
}
