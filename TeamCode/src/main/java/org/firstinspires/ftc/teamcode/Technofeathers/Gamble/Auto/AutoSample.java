package org.firstinspires.ftc.teamcode.Technofeathers.Gamble.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.ejml.data.DGrowArray;
import org.firstinspires.ftc.teamcode.freehug.Freehugdrive;

@Autonomous
public class AutoSample extends LinearOpMode {
    protected DcMotor frontRight;
    protected DcMotor frontLeft;
    protected DcMotor backRight;
    protected DcMotor backLeft;
    public DcMotor verticalLeftLift;

    public DcMotor verticalRightLift;

    public Servo pivotSlide;
    public Servo pivotClaw;
    public int FRval;
    public int FLval;
    public int BRval;
    public int BLval;
    public Servo grabber;
    public void runOpMode(){

        grabber = hardwareMap.get(Servo.class, "grabber");

        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setPower(0);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setPower(0);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setPower(0);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        backRight = hardwareMap.get(DcMotor.class, "backRight");
        backRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setPower(0);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pivotClaw = hardwareMap.get(Servo.class,"pivotClaw");
        pivotSlide = hardwareMap.get(Servo.class,"pivotSlide");


        verticalLeftLift = hardwareMap.get(DcMotor.class,  "verticalLeftLift");
        verticalRightLift = hardwareMap.get(DcMotor.class, "verticalRightLift");

        verticalLeftLift.setDirection(DcMotorSimple.Direction.FORWARD);
        verticalRightLift.setDirection(DcMotorSimple.Direction.REVERSE);

        verticalLeftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        verticalRightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        verticalLeftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset the motor encoder
        verticalRightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset the motor encoder

        verticalLeftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // Turn the motor back on when we are done
        verticalRightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        FRval = 0;
        FLval = 0;
        BRval = 0;
        BLval = 0;
        grabber.setPosition(0.3);

        waitForStart();

        verticalLeftLift.setTargetPosition(-2860);
        verticalRightLift.setTargetPosition(-2860);
        verticalRightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        verticalLeftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        verticalLeftLift.setPower(0.9);
        verticalRightLift.setPower(0.9);
        while(opModeIsActive()&&verticalRightLift.isBusy()||verticalLeftLift.isBusy()){
            idle();
        }
        verticalLeftLift.setPower(0);
        verticalRightLift.setPower(0);
        verticalRightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        verticalLeftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pivotSlide.setPosition(0.6);
        pivotClaw.setPosition(0.35);


        moveForward(-560,0.2);
        double forwarding= 60;
        FRval +=forwarding;
        FLval -=forwarding;
        BRval +=forwarding;
        BLval -=forwarding;
        frontRight.setTargetPosition(FRval);
        frontLeft.setTargetPosition(FLval);
        backRight.setTargetPosition(BRval);
        backLeft.setTargetPosition(BLval);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        settingPower(0.1);
        while (opModeIsActive()&&(frontLeft.isBusy() || frontRight.isBusy() || backLeft.isBusy() || backRight.isBusy())){
            idle();
        }
        settingPower(0);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        grabber.setPosition(0.5);
        sleep(500);
        moveForward(200,0.2);
        verticalLeftLift.setTargetPosition(260);
        verticalRightLift.setTargetPosition(260);
        verticalRightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        verticalLeftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        verticalLeftLift.setPower(0.95);
        verticalRightLift.setPower(0.95);
        while(opModeIsActive()&&verticalRightLift.isBusy()||verticalLeftLift.isBusy()){
            idle();
        }
        verticalLeftLift.setPower(0);
        verticalRightLift.setPower(0);
        verticalRightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        verticalLeftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


    }

    public void moveForward(int forward,double power){
        FRval +=forward;
        FLval +=forward;
        BRval +=forward;
        BLval +=forward;
        frontRight.setTargetPosition(FRval);
        frontLeft.setTargetPosition(FLval);
        backRight.setTargetPosition(BRval);
        backLeft.setTargetPosition(BLval);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        settingPower(power);
        while (opModeIsActive()&&(frontLeft.isBusy() || frontRight.isBusy() || backLeft.isBusy() || backRight.isBusy())){
            idle();
        }
        settingPower(0);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
    public void settingPower(double poder){
        frontRight.setPower(poder);
        frontLeft.setPower(poder);
        backRight.setPower(poder);
        backLeft.setPower(poder);
    }
}
