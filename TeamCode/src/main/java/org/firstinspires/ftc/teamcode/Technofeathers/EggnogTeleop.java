package org.firstinspires.ftc.teamcode.Technofeathers;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Controller;

@TeleOp(name = "Eggnog Teleop")
public class EggnogTeleop extends OpMode {
    private TechnofeathersPDTest test = new TechnofeathersPDTest(0.1);
    //smaller kp = slowing down earlier
    //bigger kp = slowing down later
    private TechnofeathersDrive drive;
    private Controller controller1;
    //private Controller controller2;

    private Servo pivot1;
    private Servo grabber;
    private DcMotor lift1;
    private DcMotor lift2;
    private DcMotor intake;
    private Servo stopper;
    private int i = 0;
    private int j = 0;

    public int intake_state = 0;
    public int intakeOn = 0;
    public int placeholderB = 1;

    public int placeholderC = 1;
    public int placeholderD = 1;

    public int placeholderE = 1;
    public int placeholderF = 1;

    public int placeholderG = 1;
    public int placeholderH = 1;
    public int placeholderI = 1;

    public ElapsedTime timer = new ElapsedTime();

    @Override
    public void init() {
        drive = new TechnofeathersDrive(this, hardwareMap);
        controller1 = new Controller(gamepad1);
        //controller2 = new Controller(gamepad2);
        pivot1 = hardwareMap.get(Servo.class,  "pivot1");
        grabber = hardwareMap.get(Servo.class, "grabber");
        lift1 = hardwareMap.get(DcMotor.class,  "lift1");
        lift2 = hardwareMap.get(DcMotor.class, "lift2");
        lift1.setDirection(DcMotorSimple.Direction.FORWARD);
        lift2.setDirection(DcMotorSimple.Direction.REVERSE);
        intake = hardwareMap.get(DcMotor.class, "intake");
        stopper = hardwareMap.get(Servo.class, "stopper");
    }

    @Override
    public void loop() {
        controller1.update();
        drive.drive(controller1.left_stick_x, controller1.left_stick_y/1.25, controller1.right_stick_x/1.25);
        if (controller1.AOnce() && intakeOn == 0 && i == 0) {
            stopper.setPosition(.9);
            intake.setPower(1);
            intakeOn = 1;
            //import timer later
        } else if (controller1.AOnce() && intakeOn == 1 && j == 0){
            stopper.setPosition(.37);
            intake.setPower(0);
            intakeOn = 0;
        }

        if (controller1.BOnce() && placeholderG == 1) {
            intake.setPower(-1);
            placeholderG = 2;
        } else if (controller1.BOnce() && placeholderG == 2){
            intake.setPower(0);
            placeholderG = 1;
        }



        if (controller1.dpadUpOnce() && placeholderC == 1) {
            // grabbing pixels
            grabber.setPosition(0);
            placeholderC = 2;
        }

        if (controller1.dpadDownOnce() && placeholderC == 2) {
            // releasing pixels
            grabber.setPosition(1);
            placeholderC = 1;
        }

        if (controller1.YOnce() && placeholderD == 1) {
            pivot1.setPosition(0);
            placeholderD = 2;
        } else if (controller1.YOnce() && placeholderD ==2) {
            pivot1.setPosition(0.80);
            placeholderD = 1;
        }

        if (controller1.XOnce() && placeholderI == 1) {
            stopper.setPosition(.9);
            placeholderI = 2;
        } else if (controller1.XOnce() && placeholderI == 2) {
            stopper.setPosition(.37);
            placeholderI = 1;
        }

        //lift
        if (controller1.leftBumper()) {
            lift1.setPower(0.5);
            lift2.setPower(0.5);
        } else if (controller1.rightBumper()) {
            lift1.setPower(-0.5);
            lift2.setPower(-0.5);
        } else {
            lift1.setPower(0);
            lift2.setPower(0);
        }
        i = 0;
        j = 0;
/*
        if (controller1.left_trigger > 0.9){
            pivot1.setPosition(pivot1.getPosition() + 0.05);
            //pivot2.setPosition(pivot2.getPosition()+0.05);
        } else if (controller1.right_trigger > 0.9){
            pivot1.setPosition(pivot1.getPosition()-0.05);
            //pivot2.setPosition(pivot2.getPosition()-0.05);
        }

 */

        /*if (controller1.BOnce() ) {
            intake.setPower(-1);
        }

        if (controller1.BOnce()){
            intake.setPower(0);
        }
        */


        //if (controller2.dpadRight()){
        //pivot1.setPosition(.65);

        //} else if (controller2.dpadRight()){
        //pivot1.setPosition(.75);
        //}
    }
}