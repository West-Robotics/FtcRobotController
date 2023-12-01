package org.firstinspires.ftc.teamcode.Technofeathers;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Controller;

import org.firstinspires.ftc.teamcode.Technofeathers.TechnofeathersDrive;


@TeleOp(name = "Technofeathers TeleOp")
public class TechnofeathersTeleop extends OpMode {
    // TODO: Implement P in TeleOp
    private TechnofeathersPDTest test = new TechnofeathersPDTest(0.1);
    //smaller kp = slowing down earlier
    //bigger kp = slowing down later
    private TechnofeathersDrive drive;
    private Controller controller1;
    private Controller controller2;

    private Servo pivot1;
    private Servo pivot2;
    private Servo grabber;
    private DcMotor lift1;
    private DcMotor lift2;
    private DcMotor intake;

    public int placeholderA = 1;
    public int placeholderB = 1;
    @Override
    public void init() {
        drive = new TechnofeathersDrive(this, hardwareMap);
        controller1 = new Controller(gamepad1);
        controller2 = new Controller(gamepad2);
        pivot1 = hardwareMap.get(Servo.class,  "pivot1");
        pivot2 = hardwareMap.get(Servo.class,  "pivot2");
        grabber = hardwareMap.get(Servo.class, "grabber");
        lift1 = hardwareMap.get(DcMotor.class,  "lift1");
        lift2 = hardwareMap.get(DcMotor.class, "lift2");
        lift1.setDirection(DcMotorSimple.Direction.FORWARD);
        lift2.setDirection(DcMotorSimple.Direction.REVERSE);
        intake = hardwareMap.get(DcMotor.class, "intake");
        // TODO: change positions later
        pivot1.setPosition(0.4);
        pivot2.setPosition(0.4);
    }

    @Override
    public void loop() {
        controller1.update();
        controller2.update();
        drive.drive(controller1.left_stick_x, -controller1.left_stick_y/1.25, -controller1.right_stick_x/1.25);

        if (controller2.B()) {
            // TODO: change positions later
            pivot1.setPosition(180);
            pivot2.setPosition(180);
            test.setDesiredPoint(0.4);
            test.update(pivot1.getPosition());
        }

        if (controller2.X()) {
            // grabbing pixels
            grabber.setPosition(0.6);
        }

        if (controller2.Y()) {
            // releasing pixels
            grabber.setPosition(0.7);
        }
        //lift
        if (controller2.leftBumper()) {
            lift1.setPower(0.5);
            lift2.setPower(0.5);
        } else if (controller2.rightBumper()) {
            lift1.setPower(-0.5);
            lift2.setPower(-0.5);
        } else {
            lift1.setPower(0);
            lift2.setPower(0);
        }

        if (controller2.AOnce() && placeholderA == 1) {
            intake.setPower(1);
            placeholderA = 2;
        }
        if (controller2.AOnce() && placeholderA == 2){
                intake.setPower(0);
                placeholderA = 1;
        }

        if (controller2.BOnce() && placeholderB == 1) {
            intake.setPower(1);
            placeholderB = 2;
        }

        if (controller2.BOnce() && placeholderB == 2){
            intake.setPower(0);
            placeholderB = 1;
        }

        if (controller2.left_trigger > 0.9){
            pivot1.setPosition(pivot1.getPosition()+0.05);
            pivot2.setPosition(pivot2.getPosition()+0.05);
        } else if (controller2.right_trigger > 0.9){
            pivot1.setPosition(pivot1.getPosition()-0.05);
            pivot2.setPosition(pivot2.getPosition()-0.05);
        }
    }
}