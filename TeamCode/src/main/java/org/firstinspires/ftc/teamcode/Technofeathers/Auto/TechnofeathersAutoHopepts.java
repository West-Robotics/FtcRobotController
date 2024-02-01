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

@Autonomous(name="TechnofeathersAutoHopepts", group="Technofeathers")
public class TechnofeathersAutoHopepts extends LinearOpMode {

    @Override public void runOpMode() throws InterruptedException {

        TechnofeathersPDTest test = new TechnofeathersPDTest(0.1);
        Servo pivot1;
        Servo grabber;
        DcMotor lift1;
        DcMotor lift2;
        DcMotor intake;
        Servo stopper;

        TechnofeathersDrive drive = new TechnofeathersDrive(this, hardwareMap);
        pivot1 = hardwareMap.get(Servo.class,  "pivot1");
        grabber = hardwareMap.get(Servo.class, "grabber");
        lift1 = hardwareMap.get(DcMotor.class,  "lift1");
        lift2 = hardwareMap.get(DcMotor.class, "lift2");
        lift1.setDirection(DcMotorSimple.Direction.FORWARD);
        lift2.setDirection(DcMotorSimple.Direction.REVERSE);
        intake = hardwareMap.get(DcMotor.class, "intake");
        stopper = hardwareMap.get(Servo.class, "stopper");

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
            double lift2CurrentRotation = lift2.getCurrentPosition()/537.7;
            /*
            if (lift1CurrentRotation <= 0) {
                lift1.setPower(0);
                lift2.setPower(0);
                telemetry.addLine("Lift Limit Imposed");
            }

             */

            if (lift1CurrentRotation > 3) {
                lift1.setPower(0);
                lift2.setPower(0);
                telemetry.addLine("Lift Limit Imposed");
            }

            if (e.time(TimeUnit.SECONDS) < 2) {
                drive.drive(0,0.5,0);
                telemetry.addLine("Driving");

                stopper.setPosition(0.37);
                telemetry.addLine("Driving to backdrop");
            }
            telemetry.addData("Time ran: ", e);


            if (2 < e.time(TimeUnit.SECONDS) && e.time(TimeUnit.SECONDS) < 3) {
                telemetry.addLine("new loop");

                drive.drive(0,0,0);
                telemetry.addLine("Drive ran and now it is stopped.");

                //System.out.println("Drive ran and now is stopped");
                lift1.setPower(0.5);
                lift2.setPower(0.5);
                telemetry.addLine("Drive stopped, lift run.");

            }

            lift1.setPower(0);
            lift2.setPower(0);

            if (e.time(TimeUnit.SECONDS) < 4 && e.time(TimeUnit.SECONDS) > 3) {
                pivot1.setPosition(0);
            }
            if (e.time(TimeUnit.SECONDS) < 5 && e.time(TimeUnit.SECONDS) > 4) {
                grabber.setPosition(1);
            }
            if (e.time(TimeUnit.SECONDS) < 6.5 && e.time(TimeUnit.SECONDS) > 5) {
                drive.drive(1,0,0.1);
                stopper.setPosition(0.9);
            }
            if (e.time(TimeUnit.SECONDS) < 10 && e.time(TimeUnit.SECONDS) > 6.5) {
                drive.drive(0,1,0.1);
            }
            if (e.time(TimeUnit.SECONDS) < 11 && e.time(TimeUnit.SECONDS) > 10) {
                drive.drive(0,0,0);
                intake.setPower(1);
            }
            if (e.time(TimeUnit.SECONDS) < 12 && e.time(TimeUnit.SECONDS) > 11) {
                intake.setPower(0);
                stopper.setPosition(0.37);
            }
            if (e.time(TimeUnit.SECONDS) < 26 && e.time(TimeUnit.SECONDS) > 25) {
                stopper.setPosition(0.9);
            }
            telemetry.update();
        }
    }
}