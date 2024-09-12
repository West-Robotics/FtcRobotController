package org.firstinspires.ftc.teamcode.Technofeathers.UntitledRobot.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


import org.apache.commons.math3.geometry.euclidean.twod.Line;


@Autonomous(name = "autotester")
public class autotester extends LinearOpMode{

    protected DcMotor frontright;
    protected DcMotor frontleft;
    protected DcMotor backright;
    protected DcMotor backleft;

    public void runOpMode(){
        frontright = hardwareMap.get(DcMotor.class,"frontRight");
        frontleft = hardwareMap.get(DcMotor.class, "frontLeft");
        backleft = hardwareMap.get(DcMotor.class, "backLeft");
        backright = hardwareMap.get(DcMotor.class, "backRight");

        frontright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        frontright.setPower(0.5);
        frontleft.setPower(-0.5);
        backleft.setPower(-0.5);
        backright.setPower(0.5);

        sleep(5000);


    }

}