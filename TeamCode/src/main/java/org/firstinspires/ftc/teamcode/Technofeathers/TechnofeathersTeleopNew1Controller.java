package org.firstinspires.ftc.teamcode.Technofeathers;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Controller;


@TeleOp(name = "TechnofeathersTeleopNew1Controller")
public class TechnofeathersTeleopNew1Controller extends OpMode {
    // TODO: Implement P in TeleOp
    //private TechnofeathersPDTest test = new TechnofeathersPDTest(0.1);
    //smaller kp = slowing down earlier
    //bigger kp = slowing down later
    private TechnofeathersDrive drive;
    private Controller controller;

    private Servo pivot1;
    //private Servo pivot2;
    private Servo grabber;
    private DcMotor lift1;
    private DcMotor lift2;
    private DcMotor intake;

    public int placeholderA = 1;
    public int placeholderB = 1;
    @Override
    public void init() {
        drive = new TechnofeathersDrive(this, hardwareMap);
        controller = new Controller(gamepad1);
        pivot1 = hardwareMap.get(Servo.class,  "pivot1");
        //pivot2 = hardwareMap.get(Servo.class,  "pivot2");
        grabber = hardwareMap.get(Servo.class, "grabber");
        lift1 = hardwareMap.get(DcMotor.class,  "lift1");
        lift2 = hardwareMap.get(DcMotor.class, "lift2");
        lift1.setDirection(DcMotorSimple.Direction.FORWARD);
        lift2.setDirection(DcMotorSimple.Direction.REVERSE);
        intake = hardwareMap.get(DcMotor.class, "intake");
        // TODO: change positions later
    }

    @Override
    public void loop() {
        controller.update();
        drive.drive(-controller.left_stick_x, -controller.left_stick_y/1.25, controller.right_stick_x/1.25);
        /*
        if (controller2.B()) {
            // TODO: change positions later
            pivot1.setPosition(180);
            //pivot2.setPosition(180);
            //test.setDesiredPoint(0.4);
            //test.update(pivot1.getPosition());
        }

         */

        if (controller.X()) {
            // grabbing pixels
            grabber.setPosition(0.6);
        }

        if (controller.Y()) {
            // releasing pixels
            grabber.setPosition(0.7);
        }
        //lift
        if (controller.leftBumper()) {
            lift1.setPower(0.5);
            lift2.setPower(0.5);
        } else if (controller.rightBumper()) {
            lift1.setPower(-0.5);
            lift2.setPower(-0.5);
        } else {
            lift1.setPower(0);
            lift2.setPower(0);
        }

        if (controller.AOnce() && placeholderA == 1) {
            intake.setPower(1);
            placeholderA = 2;
        }
        if (controller.AOnce() && placeholderA == 2){
            intake.setPower(0);
            placeholderA = 1;
        }

        if (controller.BOnce() && placeholderB == 1) {
            intake.setPower(-1);
            placeholderB = 2;
        }

        if (controller.BOnce() && placeholderB == 2){
            intake.setPower(0);
            placeholderB = 1;
        }

        if (controller.dpadRight()){
            pivot1.setPosition(.7);
            //pivot2.setPosition(pivot2.getPosition()+0.05);
        } else if (controller.dpadLeft()){
            pivot1.setPosition(.5);
            //pivot2.setPosition(pivot2.getPosition()-0.05);
        }
    }
}