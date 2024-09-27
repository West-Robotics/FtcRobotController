package org.firstinspires.ftc.teamcode.IronNest;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.concurrent.TimeUnit;

@Autonomous(name="IronAutoBackDrop")

public class auto2 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor backleft;
        DcMotor backright;
        DcMotor frontleft;
        DcMotor frontright;
        DistanceSensor dissensor;
        ElapsedTime timer;

        frontleft = hardwareMap.get(DcMotor.class, "frontleft");
        frontright = hardwareMap.get(DcMotor.class, "frontright");
        backleft = hardwareMap.get(DcMotor.class, "backleft");
        backright = hardwareMap.get(DcMotor.class, "backright");
        dissensor = hardwareMap.get(DistanceSensor.class, "dissensor");


        waitForStart();
        if (isStopRequested()) return;


        while (opModeIsActive()) {
            telemetry.addData("FrontLeft", frontleft.getCurrentPosition());
            telemetry.addData("FrontRight", frontright.getCurrentPosition());
            telemetry.addData("Backleft", backleft.getCurrentPosition());
            telemetry.addData("BackRight", backright.getCurrentPosition());
            telemetry.addData("Senssor Data", dissensor.getDistance(DistanceUnit.INCH));
            telemetry.update();
            timer = new ElapsedTime();
            while(timer.seconds()<15);

            if (dissensor.getDistance(DistanceUnit.INCH) > 50) {
                frontleft.setPower(-0.5);
                frontright.setPower(0.5);
                backleft.setPower(-0.5);
                backright.setPower(0.5);
            }

            else {
                timer.reset();
                while(timer.nanoseconds()<500) {

                    frontleft.setPower(1);
                    frontright.setPower(-1);
                    backright.setPower(1);
                    backleft.setPower(1);
                }
                break;
            }

            continue;
        }






        /* Forward
        frontleft.setPower(-1);
        frontright.setPower(1);
        backleft.setPower(-1);
        backright.setPower(1);

        Stop
        frontleft.setPower(0);
        frontright.setPower(0);
        backright.setPower(0);
        backleft.setPower(0);


        Back
        frontleft.setPower(1);
        frontright.setPower(-1);
        backleft.setPower(1);
        backright.setPower(-1);

        timer = new ElapsedTime();
        while (timer.seconds() < .1) {
            continue;
        } */
    }

}
