package org.firstinspires.ftc.teamcode.Technofeathers.Gamble.Auto;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.apache.commons.math3.geometry.euclidean.twod.Line;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Technofeathers.Gamble.Teleop.Subsystems.VerticalLift;
import org.firstinspires.ftc.teamcode.Technofeathers.TechnofeathersDrive;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.TimeUnit;

@Autonomous
public class blackjackauto extends LinearOpMode {



    public VerticalLift verticalLift = new VerticalLift();


    public ElapsedTime timer = new ElapsedTime();
    public Servo diffyRotatorLeft;
    public Servo diffyRotatorRight;
    public Servo linkageServoLeft;
    public Servo linkageServoRight;

    public Servo pivotSlide;
    public Servo pivotClaw;

    public Servo grabber;
    public Servo horizontalgrabber;

    public TechnofeathersDrive drive;
    SparkFunOTOS myOtos;
    YawPitchRollAngles robotOrientation;
    SparkFunOTOS.Pose2D pos;



    double errorForStraight;
    double errorForStrafe;
    double errorForTurns;




    double lasterrorCheck;

    double currentErrorCheck;
    double lastError3;
    double integralSum;
    double lastError;
    double integralSumForStraight;
    double integralSumForStrafe;
    double lastError2;
    @Override
    public void runOpMode() throws InterruptedException{
        drive = new TechnofeathersDrive();
        drive.setupMotors(hardwareMap);
        myOtos = hardwareMap.get(SparkFunOTOS.class, "sensorOtos");
        grabber = hardwareMap.get(Servo.class,"grabber");
        configureOtos();
        telemetry.addData("Current Otos Angle", myOtos.getPosition().h);
        telemetry.update();
        lastError = 0;
        lastError2 = 0;
        lastError3 = 0;
        currentErrorCheck = 5;
        /*
        verticalLift.setupMotors(hardwareMap);
        diffyRotatorLeft = hardwareMap.get(Servo.class, "diffyRotatorLeft");
        diffyRotatorRight = hardwareMap.get(Servo.class, "diffyRotatorRight");
        linkageServoLeft = hardwareMap.get(Servo.class, "linkageServoLeft");
        linkageServoRight = hardwareMap.get(Servo.class, "linkageServoRight");
        pivotSlide = hardwareMap.get(Servo.class, "diffyRotatorLeft");
        pivotClaw = hardwareMap.get(Servo.class, "diffyRotatorLeft");
        grabber = hardwareMap.get(Servo.class, "diffyRotatorLeft");
        horizontalgrabber = hardwareMap.get(Servo.class,"diffyRotaterLeft");



        grabber.setPosition(0.5);
*/
        grabber.setPosition(0.3);


        waitForStart();
        pos = myOtos.getPosition();
        telemetry.addData("X coordinate", pos.x);
        telemetry.addData("Y coordinate", pos.y);
        telemetry.addData("Heading angle", pos.h);

        telemetry.update();
        sleep(2000);
        timer.reset();
        /*
        do{
            pos = myOtos.getPosition();
            double targety = 5;
            double targetx = 0;
            double heading = 0;
            double powery = PIDControlForStraight(targety,pos.y,0.5,0,0.0);
            double powerx = -PIDControlForStrafe(targetx,pos.x,0.5,0,0.0);
            double powerheading = -PIDControl(heading,pos.h,1.2,0,0.02);

            lasterrorCheck = currentErrorCheck;
            currentErrorCheck = heading - pos.h;

            telemetry.addData("heading power", powerheading);
            telemetry.addData("x power", powerx);
            telemetry.addData("y power", powery);

            drive.drive(powerx, powery, powerheading);
            timer.reset();

            telemetry.addData("X coordinate", pos.x);
            telemetry.addData("Y coordinate", pos.y);
            telemetry.addData("Heading angle", pos.h);
            telemetry.update();

        } while (opModeIsActive() ) ;// && (Math.abs(lasterrorCheck)>0.5 || Math.abs(currentErrorCheck)>0.5));
        drive.drive(0,0,0);


         */
    }


    public double PIDControlForStraight(double reference, double state,double P, double I, double D){

        double error = (reference - state)/12;
        integralSumForStraight += error * timer.seconds();
        double derivative = (error - lastError2) / (timer.seconds());
        lastError2 = error;

        return  (error * P) + (derivative * D) + (integralSumForStraight * I);
    }

    public double PIDControlForStrafe(double reference, double state,double P, double I, double D){

        double error = (reference - state)/12;
        integralSumForStrafe += error * timer.seconds();
        double derivative = (error - lastError3) / (timer.seconds());
        lastError3 = error;

        return  (error * P) + (derivative * D) + (integralSumForStrafe * I);
    }

    public double PIDControl(double reference, double state,double p, double i, double d){
        reference = Math.toRadians(reference);
        state = Math.toRadians(state);

        double error = angleReset(reference-state);
        integralSum += error * timer.seconds();
        double derivative = (error - lastError) / (timer.seconds());
        lastError = error;

        return (error * p) + (derivative * d) + (integralSum * i);
    }

    public double angleReset(double radians){
        while (radians > Math.PI){
            radians -= 2 * Math.PI;
        }
        while (radians < -Math.PI){
            radians += 2 * Math.PI;
        }
        return radians;
    }

    private void configureOtos() {
        telemetry.addLine("Configuring OTOS...");
        telemetry.update();

        myOtos.setLinearUnit(DistanceUnit.INCH);
        myOtos.setAngularUnit(AngleUnit.DEGREES);

        //set in inches and degrees
        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0, 0, 0);
        myOtos.setOffset(offset);

        // Here we can set the linear and angular scalars, which can compensate for
        // scaling issues with the sensor measurements. Note that as of firmware
        // version 1.0, these values will be lost after a power cycle, so you will
        // need to set them each time you power up the sensor. They can be any value
        // from 0.872 to 1.127 in increments of 0.001 (0.1%). It is recommended to
        // first set both scalars to 1.0, then calibrate the angular scalar, then
        // the linear scalar. To calibrate the angular scalar, spin the robot by
        // multiple rotations (eg. 10) to get a precise error, then set the scalar
        // to the inverse of the error. Remember that the angle wraps from -180 to
        // 180 degrees, so for example, if after 10 rotations counterclockwise
        // (positive rotation), the sensor reports -15 degrees, the required scalar
        // would be 3600/3585 = 1.004. To calibrate the linear scalar, move the
        // robot a known distance and measure the error; do this multiple times at
        // multiple speeds to get an average, then set the linear scalar to the
        // inverse of the error. For example, if you move the robot 100 inches and
        // the sensor reports 103 inches, set the linear scalar to 100/103 = 0.971
        myOtos.setLinearScalar(1.0);
        myOtos.setAngularScalar(0.992);
        //calibrate before starting, takes up 617 milliseconds
        myOtos.calibrateImu();

        myOtos.resetTracking();

        // After resetting the tracking, the OTOS will report that the robot is at
        // the origin. If your robot does not start at the origin, or you have
        // another source of location information (eg. vision odometry), you can set
        // the OTOS location to match and it will continue to track from there.
        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(0, 0, 0);
        myOtos.setPosition(currentPosition);


        telemetry.update();
    }
}
