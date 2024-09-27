// Make sure to update IronnestOneDriver whenever you update this.
package org.firstinspires.ftc.teamcode.IronNest;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import org.firstinspires.ftc.teamcode.Controller;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp(name = "Ironnest")
public class IronnestTeleop extends OpMode {
    private Controller controller;
    private Controller controller2;
    private double wristpos;

    private DcMotor frontleft;
    private DcMotor frontright;
    private DcMotor backleft;
    private DcMotor backright;
    private DcMotor armangle;
    private DcMotor armextend;

    private Servo plane;
    private Servo clawright;
    private Servo clawleft;
    private Servo clawup;
    public double f= (float) 0.1;
    private TouchSensor armreset;
    public int armpos;
    public double drivespeed=0.5;
    public double F;
    private final double ticksindegrees = 1800/1120;


    @Override
    public void init() {
        controller = new Controller(gamepad1);
        controller2 = new Controller(gamepad2);
        frontleft = hardwareMap.get(DcMotor.class, "frontleft");
        frontright = hardwareMap.get(DcMotor.class, "frontright");
        backleft = hardwareMap.get(DcMotor.class, "backleft");
        backright = hardwareMap.get(DcMotor.class, "backright");
        armextend = hardwareMap.get(DcMotor.class, "armextend");
        armangle = hardwareMap.get(DcMotor.class, "armangle");
        plane = hardwareMap.get(Servo.class, "plane");
        clawup = hardwareMap.get(Servo.class, "clawup");
        clawright = hardwareMap.get(Servo.class, "claw1");
        clawleft = hardwareMap.get(Servo.class, "claw2");
        armreset = hardwareMap.get(TouchSensor .class, "armreset");
        if(armreset.isPressed()){
            armangle.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            armangle.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }





    }

    @Override
    public void loop() {
        if (gamepad1.right_trigger>0.5){
            drivespeed = 0.5;
        }
        if (gamepad1.right_bumper){
            drivespeed = 1;
        }
        //forward
        frontleft.setPower(0 - gamepad1.left_stick_x*drivespeed + gamepad1.left_stick_y*drivespeed);
        frontright.setPower(0 - gamepad1.left_stick_x*drivespeed - gamepad1.left_stick_y*drivespeed);
        backright.setPower(0 + gamepad1.left_stick_x*drivespeed - gamepad1.left_stick_y*drivespeed);
        backleft.setPower(0 + gamepad1.left_stick_x*drivespeed + gamepad1.left_stick_y*drivespeed);

        if (gamepad1.x) {
            frontleft.setPower(drivespeed);
            frontright.setPower(drivespeed);
            backleft.setPower(drivespeed);
            backright.setPower(drivespeed);
        }
        if (gamepad1.b) {
            frontleft.setPower(-drivespeed);
            frontright.setPower(-drivespeed);
            backleft.setPower(-drivespeed);
            backright.setPower(-drivespeed);
        }


        armpos = armangle.getCurrentPosition()-(1120/180);
        F = Math.cos(Math.toRadians(armpos/ticksindegrees))*f;
        if(armreset.isPressed() && armpos<-4 && armpos>-8){
            armangle.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            armangle.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        if (gamepad2.b){
            armextend.setPower(1);
        }else if (gamepad2.x){
            armextend.setPower(-1);
        }else {
            armextend.setPower(0);
        }
        if (gamepad2.y) {
            armangle.setPower(0.5);
        } else if (gamepad2.a) {
            armangle.setPower(-0.5);
        } else {
            armangle.setPower(F);
        }

        if (gamepad2.right_trigger>0.5) {
            clawright.setPosition(0.17);

        } else if (gamepad2.right_bumper) {
            clawright.setPosition(0.085);

        }
        if (gamepad2.left_trigger>0.5) {
            clawleft.setPosition(0.265);
        } else if (gamepad2.left_bumper) {
            clawleft.setPosition(0.37);
        }
        if (gamepad2.dpad_up) {
            wristpos += 0.75;
            clawup.setPosition(wristpos/100);
        }else if(gamepad2.dpad_down) {
            wristpos -=0.75;
            clawup.setPosition(wristpos/100);
        }
        if (gamepad2.left_stick_button){
            wristpos = (armpos*ticksindegrees)-(100/9);
            clawup.setPosition(wristpos);
        }

        frontleft.setPower(0);
        frontright.setPower(0);
        backright.setPower(0);
        backleft.setPower(0);
        if (wristpos >100){
            wristpos =100;
        }
        if (wristpos <21){
            wristpos = 21;
        }
        if (gamepad1.left_bumper) {
            plane.setPosition(0);

        }
        if (gamepad1.y) {
            plane.setPosition(0.5);

        }



    }
}

