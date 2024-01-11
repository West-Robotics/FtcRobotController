package org.firstinspires.ftc.teamcode.ironnest;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import org.firstinspires.ftc.teamcode.Controller;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Ironnest")
public class IronnestTeleop extends OpMode {
    private Controller controller;
    private float wristpos;

    private DcMotor frontleft;
    private DcMotor frontright;
    private DcMotor backleft;
    private DcMotor backright;
    private DcMotor armangle;
    private DcMotor armextend;

    //private Servo plane;
    private Servo claw1;
    private Servo claw2;
    private Servo clawup;

    public void init() {
        controller = new Controller(gamepad1);
        frontleft = hardwareMap.get(DcMotor.class, "frontleft");
        frontright = hardwareMap.get(DcMotor.class, "frontright");
        backleft = hardwareMap.get(DcMotor.class, "backleft");
        backright = hardwareMap.get(DcMotor.class, "backright");
        armextend = hardwareMap.get(DcMotor.class, "armextend");
        armangle = hardwareMap.get(DcMotor.class, "armangle");
        //   plane = hardwareMap.get(Servo.class, "plane");
        clawup = hardwareMap.get(Servo.class, "clawup");
        claw1 = hardwareMap.get(Servo.class, "claw1");
        claw2 = hardwareMap.get(Servo.class, "claw2");

    }

    public void loop() {

        //forward
        frontleft.setPower(0 - gamepad1.left_stick_x*2 + gamepad1.left_stick_y*2);
        frontright.setPower(0 - gamepad1.left_stick_x*2 - gamepad1.left_stick_y*2);
        backright.setPower(0 + gamepad1.left_stick_x*2 - gamepad1.left_stick_y*2);
        backleft.setPower(0 + gamepad1.left_stick_x*2 + gamepad1.left_stick_y*2);

        if (gamepad1.dpad_left) {
            frontleft.setPower(2);
            frontright.setPower(2);
            backleft.setPower(2);
            backright.setPower(2);
        }
        if (gamepad1.dpad_right) {
            frontleft.setPower(-2);
            frontright.setPower(-2);
            backleft.setPower(-2);
            backright.setPower(-2);
        }
        if (gamepad1.b){
            armextend.setPower(1);
        }else if (gamepad1.x){
            armextend.setPower(-1);
        }else {
            armextend.setPower(0);
        }
        if (gamepad1.y) {
            armangle.setPower(0.5);
        } else if (gamepad1.a) {
            armangle.setPower(-0.5);
        } else {
            armangle.setPower(0);
        }
        if (gamepad1.right_trigger>0.5) {
            claw1.setPosition(0.17);
            claw2.setPosition(0.17);
        } else if (gamepad1.right_bumper) {
            claw1.setPosition(0.30);
            claw2.setPosition(0);
        }
        if (gamepad1.left_bumper) {
            wristpos += 0.5;
            clawup.setPosition(wristpos/100);
        }else if(gamepad1.left_trigger > 0.5) {
            wristpos -=0.5;
            clawup.setPosition(wristpos/100);
        }
        frontleft.setPower(0);
        frontright.setPower(0);
        backright.setPower(0);
        backleft.setPower(0);
        if (wristpos >100){
            wristpos =100;
        }
        if (wristpos <24){
            wristpos = 24;
        }



    }
}
