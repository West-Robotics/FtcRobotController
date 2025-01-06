package org.firstinspires.ftc.teamcode.Technofeathers.Gamble.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;


import org.ejml.equation.IntegerSequence;
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
    public boolean buttonB;

    public boolean clawIsOut;
    public boolean servoChanged;
    public boolean canRotateClaw;
    public boolean buttonBforcontroller1;

    public double rightHorizontalGrabberServoValue;
    public double leftHorizontalGrabberServoValue;
    @Override
    public void init(){
        drive = new TechnofeathersDrive();
        controller1 = new Controller(gamepad1);
        controller2 = new Controller(gamepad2);
        drive.setupMotors(hardwareMap);
        //horizontalLift.setupMotors(hardwareMap);
        verticalLift.setupMotors(hardwareMap);
        diffyRotatorLeft = hardwareMap.get(Servo.class, "diffyRotatorLeft");
        diffyRotatorLeft.setDirection(Servo.Direction.REVERSE);
        diffyRotatorRight.setDirection(Servo.Direction.FORWARD);
        diffyRotatorRight = hardwareMap.get(Servo.class, "diffyRotatorRight");
        linkageServoLeft = hardwareMap.get(Servo.class, "linkageServoLeft");
        linkageServoRight = hardwareMap.get(Servo.class, "linkageServoRight");
        pivotSlide = hardwareMap.get(Servo.class, "diffyRotatorLeft");
        pivotClaw = hardwareMap.get(Servo.class, "diffyRotatorLeft");
        grabber = hardwareMap.get(Servo.class, "diffyRotatorLeft");
        horizontalgrabber = hardwareMap.get(Servo.class,"diffyRotaterLeft");
        buttonA = false;
        buttonY = false;
        buttonB = false;
        buttonBforcontroller1 = false;
        rightHorizontalGrabberServoValue = 0.5;
        leftHorizontalGrabberServoValue = 0.5;
        clawIsOut = false;
        servoChanged = false;
        canRotateClaw = false;
    }

    @Override
    public void loop(){
        controller1.update();
        controller2.update();
        drive.drive(controller1.left_stick_x, -controller1.left_stick_y/1.25, controller1.right_stick_x/1.25);
        //horizontalLift.setLiftPower(controller2.left_stick_y); //power for horizontal lift, left stick
        verticalLift.setLiftPower(controller2.left_stick_y/2); //power for vertical lift, left stick controller 2

        servoChanged = false;
        //horizontal grabber
        if (controller1.BOnce()){
            if (buttonBforcontroller1){
                horizontalgrabber.setPosition(0.5);
                buttonBforcontroller1 = false;
            } else{
                horizontalgrabber.setPosition(0.75);
                buttonBforcontroller1 = true;
            }
        }

        //the diffy rotator
        if (controller1.dpadRightOnce()&& canRotateClaw){
            rightHorizontalGrabberServoValue += 0.05;
            leftHorizontalGrabberServoValue +=0.05;
            servoChanged = true;
        }
        if (controller1.dpadLeftOnce()&& canRotateClaw){
            rightHorizontalGrabberServoValue -=0.05;
            leftHorizontalGrabberServoValue -=0.05;
            servoChanged = true;
        }
        if (controller1.dpadUpOnce()){
            rightHorizontalGrabberServoValue =0.5;
            leftHorizontalGrabberServoValue =0.5;
            servoChanged = true;
        }

        if ((leftHorizontalGrabberServoValue !=0.5)||(rightHorizontalGrabberServoValue!=0.5)){
            clawIsOut = true;
        } else{
            clawIsOut = false;
        }

        if (servoChanged){
            diffyRotatorLeft.setPosition(leftHorizontalGrabberServoValue);
            diffyRotatorRight.setPosition(rightHorizontalGrabberServoValue);
        }


        // controller 2

        //linkage/horizontal slides
        if(controller2.dpadUpOnce()) {
            linkageServoLeft.setPosition(1);
            linkageServoRight.setPosition(1);

        }

        if(controller2.dpadLeftOnce()){
            linkageServoLeft.setPosition(0.3);
            linkageServoRight.setPosition(0.3);
        }
        if (controller2.dpadDownOnce()){
           linkageServoRight.setPosition(0);
           linkageServoLeft.setPosition(0);
        }

        //pivots horizontal claw up and down
        if (controller2.YOnce() && !clawIsOut ){
            if (buttonY){
                diffyRotatorLeft.setPosition(0.75);
                diffyRotatorRight.setPosition(0.25);
                canRotateClaw = false;
                buttonY = false;
            } else{
                diffyRotatorRight.setPosition(0.5);
                diffyRotatorLeft.setPosition(0.5);
                canRotateClaw = true;
                buttonY = true;
            }
        }


        //reset vertical slides to lowest positions
        if (controller2.XOnce()){
            //on hold but probably will
        }


        //pivot to align specimen vertical claw
        if (controller2.AOnce()){
            if (buttonA){
                pivotSlide.setPosition(0.5);
                pivotClaw.setPosition(0.5);
                buttonA = false;
            } else{
                pivotSlide.setPosition(0.75);
                pivotSlide.setPosition(0.75);
                buttonA = true;
            }


        }

        //vertical grabber
        if (controller2.BOnce()){
            if (buttonB){
                grabber.setPosition(0.5);
                buttonB = false;
            } else{
                grabber.setPosition(0.75);
                buttonB = true;
            }
        }



        //configure reset buttons

    }
}
