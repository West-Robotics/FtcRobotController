package org.firstinspires.ftc.teamcode.IronNest;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="IronAutoHumanPlayerRed")

public class auto2red extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor armangle;
        DcMotor armextend;
        DcMotor backleft;
        DcMotor backright;
        Servo claw1;
        Servo claw2;
        Servo clawup;
        DcMotor frontleft;
        DcMotor frontright;
        ElapsedTime timer;

        frontleft = hardwareMap.get(DcMotor.class, "frontleft");
        frontright = hardwareMap.get(DcMotor.class, "frontright");
        backleft = hardwareMap.get(DcMotor.class, "backleft");
        backright = hardwareMap.get(DcMotor.class, "backright");

        waitForStart();
        if (isStopRequested()) return;

        frontleft.setPower(1);
        frontright.setPower(1);
        backright.setPower(-1);
        backleft.setPower(-1);

        timer = new ElapsedTime();
        while (timer.seconds() < 2.2) {
            continue;
        }

        frontleft.setPower(0);
        frontright.setPower(0);
        backright.setPower(0);
        backleft.setPower(0);

        frontleft.setPower(-1);
        frontright.setPower(1);
        backleft.setPower(-1);
        backright.setPower(1);

        timer = new ElapsedTime();
        while (timer.seconds() < 2.9) {
            continue;
        }

        frontleft.setPower(0);
        frontright.setPower(0);
        backright.setPower(0);
        backleft.setPower(0);
    }

}
