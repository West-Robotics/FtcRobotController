package org.firstinspires.ftc.teamcode.Technofeathers.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Controller;
import org.firstinspires.ftc.teamcode.Technofeathers.TechnofeathersDrive;
import org.firstinspires.ftc.teamcode.Technofeathers.TechnofeathersPDTest;

import java.util.concurrent.TimeUnit;

@Autonomous(name="TechnofeathersTestAuto", group="Technofeathers")
public class TechnofeathersTestAuto extends LinearOpMode {

    TechnofeathersDrive drive;
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

        drive = new TechnofeathersDrive(this, hardwareMap);
        pivot1 = hardwareMap.get(Servo.class,  "pivot1");
        grabber = hardwareMap.get(Servo.class, "grabber");
        lift1 = hardwareMap.get(DcMotor.class,  "lift1");
        lift2 = hardwareMap.get(DcMotor.class, "lift2");
        lift1.setDirection(DcMotorSimple.Direction.FORWARD);
        lift2.setDirection(DcMotorSimple.Direction.REVERSE);
        intake = hardwareMap.get(DcMotor.class, "intake");
        stopper = hardwareMap.get(Servo.class, "stopper");
        drive = new TechnofeathersDrive(this, hardwareMap);

        waitForStart();
        ElapsedTime e = new ElapsedTime();
        e.reset();
        e.startTime();
        while (e.time(TimeUnit.SECONDS)<30) {
                    /*
            //beginning intake
            while(e.time(TimeUnit.SECONDS)<1) {
                intake.setPower(1);
            }
            */

                //moves forward
            /*
            frontRight.setTargetPosition(4);
            frontLeft.setTargetPosition(4);
            backRight.setTargetPosition(4);
            backLeft.setTargetPosition(4);


            //moves left
            frontLeft.setTargetPosition(-1);
            frontRight.setTargetPosition(1);
            frontLeft.setTargetPosition(1);
            backRight.setTargetPosition(-1);
            */
            //grabs pixel
            grabber.setPosition(0);

            //lift go up
            lift1.setTargetPosition(1);
            lift2.setTargetPosition(1);


            //pivot go up
            pivot1.setPosition(0);

            //releases pixel
            grabber.setPosition(1);

            //pivot go down
            pivot1.setPosition(0.80);

            //lift go down
            lift1.setTargetPosition(-1);
            lift2.setTargetPosition(-1);
        }



        e.reset();
    }
}
