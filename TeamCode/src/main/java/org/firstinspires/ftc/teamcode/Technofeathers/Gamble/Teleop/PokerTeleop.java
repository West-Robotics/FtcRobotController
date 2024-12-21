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
    public VerticalLift verticalLift;

    public Servo diffyRotatorLeft;
    public Servo diffyRotatorRight;
    public Servo linkageServoLeft;
    public Servo linkageServoRight;

    public Servo horizontalLiftServoLeft;
    public Servo horizontalLiftServoRight;
    public Servo pivotSlide;
    public Servo pivotClaw;

    public Servo grabber;

    @Override
    public void init(){
        drive = new TechnofeathersDrive();
        controller1 = new Controller(gamepad1);
        controller2 = new Controller(gamepad2);
        drive.setupMotors(hardwareMap);
        horizontalLift.setupMotors(hardwareMap);
        verticalLift.setupMotors(hardwareMap);
        diffyRotatorLeft = hardwareMap.get(Servo.class, "diffyRotatorLeft");
        diffyRotatorRight = hardwareMap.get(Servo.class, "diffyRotatorRight");
        linkageServoLeft = hardwareMap.get(Servo.class, "linkageServoLeft");
        linkageServoRight = hardwareMap.get(Servo.class, "linkageServoRight");
        horizontalLiftServoLeft = hardwareMap.get(Servo.class, "horizontalLiftServoLeft");
        horizontalLiftServoRight = hardwareMap.get(Servo.class, "horizontalLiftServoRight");
        pivotSlide = hardwareMap.get(Servo.class, "diffyRotatorLeft");
        pivotClaw = hardwareMap.get(Servo.class, "diffyRotatorLeft");
        grabber = hardwareMap.get(Servo.class, "diffyRotatorLeft");
    }

    @Override
    public void loop(){
        controller1.update();
        drive.drive(controller1.left_stick_x, -controller1.left_stick_y/1.25, controller1.right_stick_x/1.25);
        horizontalLift.setLiftPower(controller2.left_stick_y); //power for horizontal lift, left stick
        verticalLift.setLiftPower(controller2.right_stick_y); //power for vertical lift, right stick
    }
}
