package org.firstinspires.ftc.teamcode.Technofeathers.Auto;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Technofeathers.TechnofeathersDrive;
import org.firstinspires.ftc.teamcode.Technofeathers.TechnofeathersPDTest;

import java.util.concurrent.TimeUnit;

@Autonomous(name="LiftTest", group="Technofeathers")
public class LiftTest extends LinearOpMode {

    @Override public void runOpMode() throws InterruptedException {

        TechnofeathersDrive drive = new TechnofeathersDrive(this, hardwareMap);
        DcMotor lift1 = hardwareMap.get(DcMotor.class, "lift1");
        lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset the motor encoder
        lift1.setDirection(DcMotorSimple.Direction.FORWARD);
        lift1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift1.setZeroPowerBehavior(BRAKE);

        DcMotor lift2 = hardwareMap.get(DcMotor.class, "lift2");
        lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset the motor encoder
        lift2.setDirection(DcMotorSimple.Direction.REVERSE);
        lift2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift2.setZeroPowerBehavior(BRAKE);

        waitForStart();
        ElapsedTime e = new ElapsedTime();
        e.reset();
        e.startTime();
        while(e.time(TimeUnit.SECONDS) <= 1) {
            lift1.setPower(0.25);
            lift2.setPower(0.25);
            telemetry.addData("Lift Current Rotation: ", -lift1.getCurrentPosition()/537.7);
            telemetry.update();
        }
        while (e.time(TimeUnit.SECONDS) > 1) {
            lift1.setPower(0);
            lift2.setPower(0);
            telemetry.addData("Lift Final Rotation", -lift1.getCurrentPosition()/537.7);
            telemetry.update();
        }
        /*while(e.time(TimeUnit.SECONDS)<3.6) {
            drive.drive(0,0.3,0);
        }

         */



        /*
        while(e.time(TimeUnit.SECONDS)<1) {
            drive.drive(0,0.3,0);
        }

         */

        e.reset();
        //drive.move(0.4, 60, 0);
    }
}
