package org.firstinspires.ftc.teamcode.Technofeathers.Bibot.Auto;


import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Technofeathers.TechnofeathersOldDrive;
import org.firstinspires.ftc.teamcode.Technofeathers.TechnofeathersPDTest;

@Autonomous(name="EggnogAutoNearRed", group="Technofeathers")
public class EggnogAutoNearRed extends LinearOpMode {

    @Override public void runOpMode() throws InterruptedException {
        TechnofeathersPDTest test = new TechnofeathersPDTest(0.1);
        Servo pivot1;
        Servo grabber;
        DcMotor lift1;
        DcMotor lift2;
        DcMotor intake;
        Servo stopper;
        DistanceSensor distSense1;
        int tooClose = 0;
        int liftTooHigh = 0;

        TechnofeathersOldDrive drive = new TechnofeathersOldDrive(this, hardwareMap);
        pivot1 = hardwareMap.get(Servo.class,  "pivot1");
        grabber = hardwareMap.get(Servo.class, "grabber");
        lift1 = hardwareMap.get(DcMotor.class,  "lift1");
        lift2 = hardwareMap.get(DcMotor.class, "lift2");
        lift1.setDirection(DcMotorSimple.Direction.FORWARD);
        lift2.setDirection(DcMotorSimple.Direction.REVERSE);
        intake = hardwareMap.get(DcMotor.class, "intake");
        stopper = hardwareMap.get(Servo.class, "stopper");
        distSense1 = hardwareMap.get(DistanceSensor.class, "distSense1");

        telemetry.addLine("Variables Instantiated");

        waitForStart();
        ElapsedTime e = new ElapsedTime();
        e.reset();
        lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset the motor encoder
        lift1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); //Turns motor PID off
        //um is this really necessary
        lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset the motor encoder
        lift2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry.addLine("Resetted Encoders");


        while (opModeIsActive() && !isStopRequested()) {
            double lift1CurrentRotation = lift1.getCurrentPosition()/537.7;
            //double lift2CurrentRotation = lift2.getCurrentPosition()/537.7;


            if (lift1CurrentRotation > 2) {
                lift1.setPower(0);
                lift2.setPower(0);
                telemetry.addLine("HorizontalLift Limit Imposed");
                liftTooHigh = 1;
            }
            else {
                liftTooHigh = 0;
            }



            if (distSense1.getDistance(INCH) <= 4) {
                tooClose = 1;
                drive.drive(0,0,0);
            }
            else {
                tooClose = 0;
            }
            /*telemetry.addData("Distance: ", distSense1.getDistance(INCH));
            telemetry.addData("Time ran: ", e);

             */
            /*
            if(e.seconds() < 0.5 && distSense1.getDistance(INCH) < 30) {
                telemetry.addLine("Center");
            }
            else if (0.5 < e.seconds() && e.seconds() < 1.5) {
                drive.drive(0,0,-0.4);
                //if (distSense1.)
                telemetry.addLine("Left");
            }
            */
            if (e.seconds() < 3.0 /*&& tooClose == 0*/) {
                drive.drive(-0.45,-0.5,0.04);
                grabber.setPosition(0.67);
                telemetry.addLine("Driving");
                stopper.setPosition(0.37);
                telemetry.addLine("Driving to backdrop");
            }
            while (3.4 < e.seconds() && e.seconds() < 4.3) {
                drive.drive(-0.2,-0.4,0);
            }

            if (4.5 < e.seconds() && e.seconds() < 5.2 && liftTooHigh == 0) {
                drive.drive(0,0,0);
                telemetry.addLine("Drive ran and now it is stopped.");
                lift1.setPower(0.5);
                lift2.setPower(0.5);
                telemetry.addLine("Drive stopped, lift run.");
            }

            if (e.seconds() < 6.9 && e.seconds() > 6.1) {
                lift1.setPower(0);
                lift2.setPower(0);
                pivot1.setPosition(0);
                drive.drive(0,0.1,0);
            }


            while(6.9 < e.seconds() && e.seconds() < 7.1) {
                drive.drive(0,0.1,0);
            }

            while(7.1 < e.seconds() && e.seconds() < 7.5) {
                drive.drive(0,0,0);
            }
            if (e.seconds() < 8.1 && e.seconds() > 7.5) {
                grabber.setPosition(1);
            }
            while (8.6 < e.seconds() && e.seconds() < 8.9) {
                drive.drive(0,0,0);
            }
            /*
            while (8.9 < e.seconds() && e.seconds() < 9.25) {
                lift1.setPower(-0.5);
                lift2.setPower(-0.5);
            }

             */
            /*
            if (e.seconds() < 6.5 && e.seconds() > 5 && tooClose == 0) {
                drive.drive(1,0,0.1);
                stopper.setPosition(0.9);
            }
            if (e.seconds() < 10 && e.seconds() > 6.5 && tooClose == 0) {
                drive.drive(0,1,0.1);
            }
            if (e.seconds() < 11 && e.seconds() > 10 && tooClose == 0) {
                drive.drive(0,0,0);
                intake.setPower(1);
            }
            if (e.seconds() < 12 && e.seconds() > 11 && tooClose == 0) {
                intake.setPower(0);
                stopper.setPosition(0.37);
            }
            if (e.seconds() < 26 && e.seconds() > 25 && tooClose == 0) {
                stopper.setPosition(0.9);
            }

             */
            telemetry.update();
        }
    }
}
