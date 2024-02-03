package org.firstinspires.ftc.teamcode.Technofeathers.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Controller;
import org.firstinspires.ftc.teamcode.Technofeathers.TechnofeathersDrive;
import org.firstinspires.ftc.teamcode.Technofeathers.TechnofeathersPDTest;

@TeleOp(name = "EggnogTeleopAutomated")
public class EggnogTeleopAutomated extends OpMode {
    private TechnofeathersPDTest test = new TechnofeathersPDTest(0.1);
    //smaller kp = slowing down earlier
    //bigger kp = slowing down later
    public TechnofeathersDrive drive;
    public Controller controller1;
    public Servo pivot1;
    public Servo grabber;
    public Servo airplaneLauncher;
    public DcMotor lift1;
    public DcMotor lift2;
    public DcMotor intake;
    public Servo stopper;
    public DistanceSensor distSense1;
    //private int i = 0;
    //private int j = 0;

    public int intakeOn = 0;
    public int planeLaunched = 0;
    public int grabbedPixels = 0;
    public int pivotReadyToDrop = 1;

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
        pivot1 = hardwareMap.get(Servo.class,  "pivot1");
        grabber = hardwareMap.get(Servo.class, "grabber");
        lift1 = hardwareMap.get(DcMotor.class,  "lift1");
        lift2 = hardwareMap.get(DcMotor.class, "lift2");
        lift1.setDirection(DcMotorSimple.Direction.FORWARD);
        lift2.setDirection(DcMotorSimple.Direction.REVERSE);
        lift1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake = hardwareMap.get(DcMotor.class, "intake");
        stopper = hardwareMap.get(Servo.class, "stopper");
        airplaneLauncher = hardwareMap.get(Servo.class, "airplaneLauncher");
        distSense1 = hardwareMap.get(DistanceSensor.class, "distSense1");

        lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset the motor encoder
        lift1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // Turn the motor back on when we are done
        lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset the motor encoder
        lift2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // Turn the motor back on when we are done
        pivot1.setPosition(1);
    }

    @Override
    public void loop() {
        controller1.update();
        drive.drive(controller1.left_stick_x, controller1.left_stick_y/1.25, controller1.right_stick_x/1.25);
        if (controller1.left_stick_x == 0 && controller1.left_stick_y == 0 && controller1.right_stick_x == 0) {
            drive.drive(0,0,0);
        }

        //drive.drive(-controller1.left_stick_x, -controller1.left_stick_y/1.25, -controller1.right_stick_x/1.25);


        if(controller1.dpadLeftOnce()) {
            ScoringPosition();
        }
        if(controller1.dpadRightOnce()) {
            PixelDropAndReset();
        }
        if (controller1.AOnce() && intakeOn == 0) {
            IntakeRun();
            intakeOn = 1;
            //import timer later
        } else if (controller1.AOnce() && intakeOn == 1){
            stopper.setPosition(.37);
            intake.setPower(0);
            intakeOn = 0;
        }
        //lift at good position, stopper down, intake

        if (controller1.BOnce() && placeholderG == 1) {
            intake.setPower(-1);
            placeholderG = 2;
        } else if (controller1.BOnce() && placeholderG == 2){
            intake.setPower(0);
            placeholderG = 1;
        }


        if (controller1.dpadUpOnce() && grabbedPixels == 0) {
            // grabbing pixels
            grabber.setPosition(0.67);
            grabbedPixels = 1;
        }

        if (controller1.dpadDownOnce() && grabbedPixels == 1) {
            // releasing pixels
            grabber.setPosition(1);
            grabbedPixels = 0;
        }

        if (controller1.YOnce() && pivotReadyToDrop == 0) {
            pivot1.setPosition(0);
            pivotReadyToDrop = 1;
        } else if (controller1.YOnce() && pivotReadyToDrop == 1) {
            pivot1.setPosition(1);
            pivotReadyToDrop = 0;
        }
/*
        if (controller1.startOnce() && controller1.B()) {

        }

 */
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

        double lift1CurrentRotation = lift1.getCurrentPosition()/537.7;
        double lift2CurrentRotation = lift2.getCurrentPosition()/537.7;

        if (controller1.right_trigger > 0.9 && planeLaunched == 0) {
            airplaneLauncher.setPosition(0.5);
            planeLaunched = 1;
        }

        if(controller1.right_trigger > 0.9 && planeLaunched == 1) {
            airplaneLauncher.setPosition(0.9);
            planeLaunched = 0;
        }
        if(controller1.backOnce()){
            pivot1.setPosition(0.25);
        }

        /*
        i = 0;
        j = 0;
         */
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


        //if (controller1.dpadRight()){
        //pivot1.setPosition(.65);

        //} else if (controller1.dpadRight()){
        //pivot1.setPosition(.75);
        //}
    }
    private void ScoringPosition() {
        ElapsedTime teleopTimer1 = new ElapsedTime();
        teleopTimer1.reset();
        while (teleopTimer1.seconds() < 0.5) {
            grabber.setPosition(0.67);
        }
        while (0.5 < teleopTimer1.seconds() && teleopTimer1.seconds() < 1.5) {
            lift1.setPower(1);
            lift2.setPower(1);
        }
        while (1.5 < teleopTimer1.seconds() && teleopTimer1.seconds() < 2.2) {
            pivot1.setPosition(0);
        }
    }
    public void PixelDropAndReset() {
        ElapsedTime teleopTimer2 = new ElapsedTime();
        teleopTimer2.reset();
        while(teleopTimer2.seconds() < 0.7) {
            grabber.setPosition(1);
        }

        while(0.7 < teleopTimer2.seconds() && teleopTimer2.seconds() < 1.2) {
            lift1.setPower(-1);
            lift2.setPower(-1);
        }
    }
    public void IntakeRun() {
        ElapsedTime teleopTimer3 = new ElapsedTime();
        teleopTimer3.reset();
        while(teleopTimer3.seconds() < 0.5) {
            stopper.setPosition(.9);
        }
        while(1 < teleopTimer3.seconds() && teleopTimer3.seconds() < 2) {
            intake.setPower(1);
        }
    }
}