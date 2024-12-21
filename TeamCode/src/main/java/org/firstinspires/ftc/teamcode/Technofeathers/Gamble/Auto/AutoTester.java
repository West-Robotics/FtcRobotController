package org.firstinspires.ftc.teamcode.Technofeathers.Gamble.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;


@Autonomous(name = "AutoTester")
public class AutoTester extends LinearOpMode{

    protected DcMotor frontRight;
    protected DcMotor frontLeft;
    protected DcMotor backRight;
    protected DcMotor backLeft;

    public void runOpMode() throws InterruptedException{
        frontRight = hardwareMap.get(DcMotor.class,"frontRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        waitForStart();


        frontRight.setPower(-0.5);
        frontLeft.setPower(0.5);
        backLeft.setPower(0.5);
        backRight.setPower(-0.5);

        sleep(2000);

        frontRight.setPower(0);
        frontLeft.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        sleep(500);

        frontRight.setPower(0.5);
        frontLeft.setPower(-0.5);
        backLeft.setPower(-0.5);
        backRight.setPower(0.5);

        sleep(6000);

        frontRight.setPower(0);
        frontLeft.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

    }
}