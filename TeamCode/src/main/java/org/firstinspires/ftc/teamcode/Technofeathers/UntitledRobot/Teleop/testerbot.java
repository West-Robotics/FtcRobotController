package org.firstinspires.ftc.teamcode.Technofeathers.UntitledRobot.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Controller;
@TeleOp(name="Automeanguy")
public class testerbot extends LinearOpMode {

    public DcMotor motor;
    public Controller controller1;
    public float encodervalue;
    ElapsedTime timer = new ElapsedTime();
    double lastError;
    double integralSum;
    double powering;

    public double P;
    public double I;
    public double D;
    public void runOpMode() throws InterruptedException{
        motor = hardwareMap.get(DcMotor.class,"arm");
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        controller1 = new Controller(gamepad1);
        powering=0;

        P = 0.78;
        I=0;
        D=0.3;
        double voltage = 13.1/13.5;
        P = P*voltage;
        I = I*voltage;
        D = D*voltage;
        waitForStart();
        while (opModeIsActive()){
            controller1.update();


            motor.setPower(controller1.right_stick_y);
            encodervalue= motor.getCurrentPosition();
            telemetry.addData("encoder",encodervalue);
            telemetry.update();
            /*
            if(controller1.BOnce()){
                motor.setPower(0);
            }
            encodervalue= motor.getCurrentPosition();
            telemetry.addData("encoder",encodervalue);
            if(controller1.A()){
                powering = PIDForArm(42, encodervalue, P, D, I);
                motor.setPower(powering);
            }
            if(controller1.Y()){
                powering = PIDForArm(550, encodervalue, P, D, I);
                motor.setPower(powering);
            }
            if(controller1.X()){
                powering = PIDForArm(700, encodervalue, P, D, I);
                motor.setPower(powering);
            }

            telemetry.addData("power",powering);
            telemetry.update();

             */
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
