package org.firstinspires.ftc.teamcode.Technofeathers;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Controller;

import org.firstinspires.ftc.teamcode.Technofeathers.TechnofeathersDrive;

@TeleOp(name = "Technofeathers TeleOp")
public class TechnofeathersTeleop extends OpMode {

    private TechnofeathersDrive drive;
    private Controller controller;

    private Servo pivot1;
    private Servo pivot2;
    private Servo grabber;
    private DcMotor lift1;
    private DcMotor lift2;
    private DcMotor intake;
    @Override
    public void init() {
        drive = new TechnofeathersDrive(this, hardwareMap);
        controller = new Controller(gamepad1);
        pivot1 = hardwareMap.get(Servo.class,  "pivot1");
        pivot2 = hardwareMap.get(Servo.class,  "pivot2");
        grabber = hardwareMap.get(Servo.class, "grabber");
        lift1 = hardwareMap.get(DcMotor.class,  "lift1");
        lift2 = hardwareMap.get(DcMotor.class, "lift2");
        intake = hardwareMap.get(DcMotor.class, "intake");
        pivot1.setPosition(0.4); //change later
        pivot2.setPosition(0.4); //change later
        private Controller currentGamepad1.A = gamepad1;
        private Controller previousGamepad1.A =  currentGamepad1.A;
    }

    @Override
    public void loop() {
        controller.update();
        drive.drive(controller.left_stick_x, -controller.left_stick_y/1.25, -controller.right_stick_x/1.25);
        controller.update();

        if (currentGamepad1.A && !previousGamepad1.A) {
            intake.setPower(1);
            previousGamepad1.A = currentGamepad1.A;
        }

        if (currentGamepad1.A && !previousGamepad1.A) {
            intake.setPower(0);
        }

        if (controller.B()) {
            pivot1.setPosition(180); //change later
            pivot2.setPosition(180); //change later
        }

        if (controller.X()) {
            //grabbing pixels
            grabber.setPosition(0.6);
        }

        if (controller.Y()) {
            //releasing pixels
            grabber.setPosition(0.7);
        }

        if (controller.leftBumper()){
            lift1.setPower(0.5);
            lift2.setPower(-0.5);
        } else if (controller.rightBumper()){
            lift1.setPower(-0.5);
            lift2.setPower(0.5);
        } else {
            lift1.setPower(0);
            lift2.setPower(0);
        }
    }
}