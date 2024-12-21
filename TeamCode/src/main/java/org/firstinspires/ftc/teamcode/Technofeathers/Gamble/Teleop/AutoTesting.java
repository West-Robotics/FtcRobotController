package org.firstinspires.ftc.teamcode.Technofeathers.Gamble.Teleop;



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
    public Controller controller2;

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
        controller2 = new Controller(gamepad2);
        drive.setupMotors(hardwareMap);
        grabber = hardwareMap.get(Servo.class, "grabber");
        pivot = hardwareMap.get(Servo.class, "pivot");

        arm.setDirection(DcMotorSimple.Direction.FORWARD);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        buttonA = true;
        buttonY = true;

        P = 0.75;
        I = 0;
        D = 0.3;
        double voltage = 13.5/12.82;
        P = P * voltage;
        I = I * voltage;
        D = D * voltage;

        waitForStart();
        while (opModeIsActive()){
            controller1.update();
            controller2.update();
            drive.drive(-controller1.left_stick_x/1.2, controller1.left_stick_y/1.2, controller1.right_stick_x/1.2);

            telemetry.addData("pivot", pivot.getPosition());
            telemetry.addData("grabber", grabber.getPosition());
            telemetry.addData("armencoder",arm.getCurrentPosition());

            if (controller2.B()) {
                arm.setPower(0.003);
            } else if (controller2.A()) {
                arm.setPower(-0.003);
            } else {
                arm.setPower(controller2.left_stick_y/4);
            }

            if (controller1.AOnce()){
                if (buttonA){
                    grabber.setPosition(0.17); //closed
                    buttonA = false;
                } else{
                    grabber.setPosition(0.29); // opened
                    buttonA = true;
                }
            }

            if (controller1.dpadRightOnce()){
                pivot.setPosition(1);//to pick up
            }
            if (controller1.dpadDownOnce()){
                //to start
                pivot.setPosition(0);

            }
            if(controller1.dpadUpOnce()){
                pivot.setPosition(0.71);//to score
            }
            if (controller1.dpadLeftOnce()){
                pivot.setPosition(0.35);
            }
            /*
            encodervalue= arm.getCurrentPosition();
            if (controller1.leftBumper()){
                powering = PIDForArm(40, encodervalue, P, D, I);
                arm.setPower(powering);
            }

            if(controller1.BOnce()){
                arm.setPower(0);
            }
            telemetry.addData("encoder",encodervalue);
            if(controller1.Y()){
                powering = PIDForArm(235, encodervalue, P, D, I);
                arm.setPower(powering);
            }
            if(controller1.X()){
                powering = PIDForArm(535, encodervalue, P, D, I);
                arm.setPower(powering);
            }

            telemetry.addData("power",powering);


             */
            telemetry.update();
        }
    }
    public double PIDForArm(double reference, double state,double p, double d, double i){
        double error = (reference/290)-(state/290);
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