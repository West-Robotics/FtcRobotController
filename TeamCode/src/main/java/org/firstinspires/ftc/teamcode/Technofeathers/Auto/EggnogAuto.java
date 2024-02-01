package org.firstinspires.ftc.teamcode.Technofeathers.Auto;
//TODO: MAKE SURE THIS IS VERSION THAT IS USED DURING COMPETITION

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Controller;
import org.firstinspires.ftc.teamcode.Technofeathers.TechnofeathersDrive;
import org.firstinspires.ftc.teamcode.Technofeathers.TechnofeathersPDTest;

import java.security.cert.TrustAnchor;
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
    private ColorSensor color;

    private TouchSensor touchSensor;

    private DistanceSensor distanceSensor;

    public int intake_state = 0;
    public int placeholderB = 1;
    public int placeholderX = 1;
    public int placeholderY = 1;

    private double angle = 0;
    private double x;
    private double y;


    public double distance() {
        double valueD = distanceSensor.getDistance(DistanceUnit.CM);
        telemetry.addData("distance",valueD);
        return valueD;
    }

    public boolean isLess(){
        double distance = distance();
        if (distance <=5){
            return true;
        }
        else {
            return  false;
        }
    }

    @Override public void runOpMode() throws InterruptedException {



        frontLeft = hardwareMap.get(DcMotor.class,"frontLeft");
        frontRight = hardwareMap.get(DcMotor.class,"frontRight");
        backLeft = hardwareMap.get(DcMotor.class,"backLeft");
        backRight = hardwareMap.get(DcMotor.class,"backRight");

        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        TechnofeathersDrive drive = new TechnofeathersDrive(this, hardwareMap);
        color = hardwareMap.get(ColorSensor.class,"color");
        touchSensor = hardwareMap.get(TouchSensor.class,"touchSensor");
        pivot1 = hardwareMap.get(Servo.class,  "pivot1");
        grabber = hardwareMap.get(Servo.class, "grabber");
        lift1 = hardwareMap.get(DcMotor.class,  "lift1");
        lift2 = hardwareMap.get(DcMotor.class, "lift2");
        lift1.setDirection(DcMotorSimple.Direction.FORWARD);
        lift2.setDirection(DcMotorSimple.Direction.REVERSE);
        intake = hardwareMap.get(DcMotor.class, "intake");
        stopper = hardwareMap.get(Servo.class, "stopper");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");




        waitForStart();
        if (isStopRequested()){
            angle = -90;

        }

        //boolean isit = isLess();
        //drive.moveUntil(1,20,isit );


        while (opModeIsActive()){
            int ticks = frontLeft.getCurrentPosition();
            int ticks2 = frontRight.getCurrentPosition();
            int ticks3 = backLeft.getCurrentPosition();
            int ticks4 = backRight.getCurrentPosition();


            String number = String.valueOf(ticks);
            telemetry.addLine(number);
            String number2 = String.valueOf(ticks2);
            telemetry.addLine(number2);
            String number3 = String.valueOf(ticks3);
            telemetry.addLine(number3);
            String number4 = String.valueOf(ticks4);
            telemetry.addLine(number4);


        }



    }
}
