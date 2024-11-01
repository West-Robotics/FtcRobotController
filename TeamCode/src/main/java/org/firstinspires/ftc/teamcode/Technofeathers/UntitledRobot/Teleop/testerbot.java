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
    boolean ying;
    public void runOpMode() throws InterruptedException{
        motor = hardwareMap.get(DcMotor.class,"arm");
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        controller1 = new Controller(gamepad1);
        powering=0;
        ying = false;
        waitForStart();
        while (opModeIsActive()){
            controller1.update();
            if(controller1.AOnce()){
                motor.setPower(0.5);
            }
            if(controller1.BOnce()){
                motor.setPower(0);
            }
            encodervalue= motor.getCurrentPosition();
            telemetry.addData("encoder",encodervalue);

            if(controller1.Y()){

                powering = PIDForArm(550, encodervalue, 0.8, 0.14, 0);
                motor.setPower(powering);

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
