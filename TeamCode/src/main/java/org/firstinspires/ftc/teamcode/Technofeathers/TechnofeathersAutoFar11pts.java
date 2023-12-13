package org.firstinspires.ftc.teamcode.Technofeathers;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.TimeUnit;

@Autonomous(name="TechnofeathersAutoFar11pts", group="Technofeathers")
public class TechnofeathersAutoFar11pts extends LinearOpMode {
    private Servo pivot1;
    private Servo pivot2;
    private Servo grabber;
    private DcMotor lift1;
    private DcMotor lift2;
    private DcMotor intake;

    @Override public void runOpMode() throws InterruptedException {

        TechnofeathersDrive drive = new TechnofeathersDrive(this, hardwareMap);


        waitForStart();
        ElapsedTime e = new ElapsedTime();
        e.reset();
        e.startTime();
        //drive.move(0.5, 10, 180);
        /*while(e.time(TimeUnit.SECONDS)<3.6) {
            drive.drive(0,0.3,0);
        }

         */
        while(e.time(TimeUnit.SECONDS) < 10) {
            drive.drive(0,1,-0.25);
        }

        /*
        while(e.time(TimeUnit.SECONDS)<1) {
            drive.drive(0,0.3,0);
        }

         */

        e.reset();
        //drive.move(0.4, 60, 0);
    }
}