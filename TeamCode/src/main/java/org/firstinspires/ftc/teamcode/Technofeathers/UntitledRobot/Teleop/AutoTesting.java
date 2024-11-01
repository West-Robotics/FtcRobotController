package org.firstinspires.ftc.teamcode.Technofeathers.UntitledRobot.Teleop;



import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Controller;
import org.firstinspires.ftc.teamcode.Technofeathers.TechnofeathersDrive;



@TeleOp(name = "AutoTesting")
public class AutoTesting extends LinearOpMode {

    //DcMotor frontRight;
    //DcMotor frontLeft;
    //DcMotor backLeft;
    //
    ElapsedTime timer = new ElapsedTime();
    public DcMotor arm;
    TechnofeathersDrive drive = new TechnofeathersDrive();

    public Controller controller1;

    public Servo grabber;
    public Servo pivot;
    public Servo newgrabber;
    public boolean buttonA;
    public boolean buttonY;
    public double encodervalue;

    double lastError;
    double integralSum;
    double powering;

    public double P;
    public double I;
    public double D;

    public void runOpMode() throws InterruptedException {
        arm = hardwareMap.get(DcMotor.class, "arm");
        controller1 = new Controller(gamepad1);
        drive.setupMotors(hardwareMap);
        grabber = hardwareMap.get(Servo.class, "grabber");
        pivot = hardwareMap.get(Servo.class, "pivot");
        arm.setDirection(DcMotorSimple.Direction.FORWARD);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        buttonA = true;
        buttonY = true;

        P = 0.78;
        I=0;
        D=0.3;
        double voltage = 12.90/13.5;
        P = P*voltage;
        I = I*voltage;
        D = D*voltage;

        waitForStart();
        while (opModeIsActive()){
            controller1.update();

            drive.drive(-controller1.left_stick_x, controller1.left_stick_y, controller1.right_stick_x);
            telemetry.addData("pivot", pivot.getPosition());
            telemetry.addData("grabber", grabber.getPosition());

            if (controller1.leftBumperOnce()){
                if (buttonA){
                    grabber.setPosition(0.45);
                    buttonA = false;
                } else{
                    grabber.setPosition(0.3);
                    buttonA = true;
                }
            }

            if (controller1.dpadUpOnce()){
                pivot.setPosition(0.95);
            }
            if (controller1.dpadDownOnce()){
                //to start
                pivot.setPosition(0);
            }
            if (controller1.dpadLeftOnce()){
                pivot.setPosition(0.45);//for specimen drop off
            }
            if(controller1.dpadRightOnce()){
                pivot.setPosition(0.71);//to score
            }
            if(controller1.BOnce()){
                arm.setPower(0);
            }
            encodervalue= arm.getCurrentPosition();
            telemetry.addData("encoder",encodervalue);
            if(controller1.A()){
                powering = PIDForArm(52, encodervalue, P, D, I);
                arm.setPower(powering);
            }
            if(controller1.Y()){
                powering = PIDForArm(550, encodervalue, P, D, I);
                arm.setPower(powering);
            }
            if(controller1.X()){
                powering = PIDForArm(740, encodervalue, P, D, I);
                arm.setPower(powering);
            }

            telemetry.addData("power",powering);

            telemetry.update();
        }
    }
    public double PIDForArm(double reference, double state,double p, double d, double i){
        double error = (reference/500)-(state/500);
        telemetry.addData("Error in rotations",error);
        telemetry.addData("error per tick", reference-state);

        integralSum += error * timer.seconds();
        double derivative = (error - lastError) / (timer.seconds());
        lastError = error;
        timer.reset();
        double returning = (error * p) + (derivative * d) + (integralSum * i);
        return returning;
    }

}