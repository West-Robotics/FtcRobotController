package org.firstinspires.ftc.teamcode.Technofeathers.UntitledRobot.Auto;

import com.fasterxml.jackson.databind.annotation.JsonAppend;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous()
public class AutoTimeBased extends LinearOpMode{

    protected DcMotor frontRight;
    protected DcMotor frontLeft;
    protected DcMotor backRight;
    protected DcMotor backLeft;

    protected DcMotor arm;

    protected Servo pivot;
    protected Servo grabber;

    @Override
    public void runOpMode() throws InterruptedException {

        pivot = hardwareMap.get(Servo.class,"pivot");
        grabber = hardwareMap.get(Servo.class,"grabber");

        arm = hardwareMap.get(DcMotor.class,"arm");
        arm.setDirection(DcMotorSimple.Direction.FORWARD);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setPower(0);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setPower(0);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setPower(0);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        backRight = hardwareMap.get(DcMotor.class, "backRight");
        backRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setPower(0);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        grabber.setPosition(0.165);

        waitForStart();

        telemetry.addData("arm encoder",arm.getCurrentPosition());
        telemetry.update();
        //closed position

        //to start
        pivot.setPosition(0);
        move(-0.3,600);
        sleep(400);

        arm.setTargetPosition(-810);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(0.1);

        telemetry.addData("arm encoder",arm.getCurrentPosition());
        telemetry.update();

        sleep(4500);

        telemetry.addData("arm encoder",arm.getCurrentPosition());
        telemetry.update();
        pivot.setPosition(0.71);

        sleep(4000);
        move(-0.15,600);

        sleep(3000);

        arm.setPower(0);
        sleep(2000);

        move(-0.2,500);
pl


    }

    public void move(double power, long time){
        frontRight.setPower(power);
        frontLeft.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);

        sleep(time);

        frontRight.setPower(0);
        frontLeft.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);


    }

}
