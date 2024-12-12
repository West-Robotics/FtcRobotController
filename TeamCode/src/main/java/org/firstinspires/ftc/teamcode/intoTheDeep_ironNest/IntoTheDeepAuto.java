package org.firstinspires.ftc.teamcode.intoTheDeep_ironNest;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImpl;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Controller;

@Autonomous(name = "Into-the-deep Auto")
public class IntoTheDeepAuto extends LinearOpMode {
    PIDController PID;
    DcMotor wheel_1;
    DcMotor wheel2;
    DcMotor wheel4;
    DcMotor wheel3;
    ElapsedTime timer;
    DcMotor sliders;
    Servo secondaryArm;
    Servo claw;
    int maxPosition = -1000;
    public double p=0.00, i=0, d=0;
    public int slider_position;
    public double F;
    public double f= (float) 0.0000005;
    @Override
    public void runOpMode() throws InterruptedException {

        sliders = hardwareMap.get(DcMotor.class, "primary_arm");
        secondaryArm = hardwareMap.get(Servo.class, "secondaryArm");
        claw = hardwareMap.get(Servo.class, "claw");
        wheel_1 = hardwareMap.get(DcMotorImpl.class, "left_front_drive");
        wheel2 = hardwareMap.get(DcMotor.class, "right_back_drive");
        wheel4 = hardwareMap.get(DcMotor.class, "right_front_drive");
        wheel3 = hardwareMap.get(DcMotor.class, "left_back_drive");
        wheel_1.setDirection(DcMotorSimple.Direction.REVERSE);
        sliders.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        timer = new ElapsedTime();
        PID =new PIDController(p, i, d);


        sliders.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sliders.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        waitForStart();
        sliders.setPower(0.5);

        while (opModeIsActive()) {
            if (timer.seconds() < 1) {
                wheel_1.setPower(0.75);
                wheel2.setPower(0.75);
                wheel3.setPower(0.75);
                wheel4.setPower(0.75);
                telemetry.update();
            } else {
                wheel_1.setPower(0);
                wheel2.setPower(0);
                wheel3.setPower(0);
                wheel4.setPower(0);
            }


            PID.setPID(p, i, d);
            slider_position  = sliders.getCurrentPosition();
            F = slider_position*f;
            
            if (slider_position  >= maxPosition) {
                sliders.setPower(-0.5+F);
                telemetry.addData("F",F);
                telemetry.update();
            }else {
               sliders.setPower(F);
            }
        }
    }
}