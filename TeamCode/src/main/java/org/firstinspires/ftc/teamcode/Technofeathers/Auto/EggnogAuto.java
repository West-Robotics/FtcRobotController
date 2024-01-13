package org.firstinspires.ftc.teamcode.Technofeathers.Auto;
//TODO: MAKE SURE THIS IS VERSION THAT IS USED DURING COMPETITION

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Controller;
import org.firstinspires.ftc.teamcode.Technofeathers.TechnofeathersPDTest;

import java.util.concurrent.TimeUnit;

@Autonomous(name="EggnogAuto", group="Technofeathers")
public class EggnogAuto extends LinearOpMode {

    private TechnofeathersPDTest test = new TechnofeathersPDTest(0.1);
    private Servo pivot1;
    private Servo grabber;
    private DcMotor lift1;
    private DcMotor lift2;
    private DcMotor intake;
    private Servo stopper;
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    public int intake_state = 0;
    public int placeholderB = 1;
    public int placeholderX = 1;
    public int placeholderY = 1;

    @Override public void runOpMode() throws InterruptedException {

        //TechnofeathersDrive drive = new TechnofeathersDrive(this, hardwareMap);
        //may be deleted
        /*
        @Override
        public void init() {
            drive = new TechnofeathersDrive(this, hardwareMap);
            pivot1 = hardwareMap.get(Servo.class,  "pivot1");
            grabber = hardwareMap.get(Servo.class, "grabber");
            lift1 = hardwareMap.get(DcMotor.class,  "lift1");
            lift2 = hardwareMap.get(DcMotor.class, "lift2");
            lift1.setDirection(DcMotorSimple.Direction.FORWARD);
            lift2.setDirection(DcMotorSimple.Direction.REVERSE);
            intake = hardwareMap.get(DcMotor.class, "intake");
            stopper = hardwareMap.get(Servo.class, "stopper");
        }
        */

        waitForStart();
        ElapsedTime e = new ElapsedTime();
        e.reset();
        e.startTime();
        /*
        //beginning intake
        while(e.time(TimeUnit.SECONDS)<1) {
            intake.setPower(1);
        }
        */
        frontRight.setTargetPosition(4);
        frontLeft.setTargetPosition(4);
        backRight.setTargetPosition(4);
        backLeft.setTargetPosition(4);

        e.reset();
    }
}
