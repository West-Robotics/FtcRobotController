package org.firstinspires.ftc.teamcode.Technofeathers.UntitledRobot.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Controller;
@TeleOp
public class testerbot extends LinearOpMode {

    public DcMotor motor;
    public Controller controller1;
    public float encodervalue;


    public void runOpMode() throws InterruptedException{
        motor = hardwareMap.get(DcMotor.class,"arm");
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        controller1 = new Controller(gamepad1);

        waitForStart();
        while (opModeIsActive()){
            controller1.update();
            if(controller1.AOnce()){
                motor.setTargetPosition(-700);
                motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motor.setPower(0.5);
                while(opModeIsActive() && motor.isBusy()){
                    idle();
                }
            }
            if(controller1.BOnce()){
                motor.setPower(0);
            }
            encodervalue= motor.getCurrentPosition();
            telemetry.addData("encoder",encodervalue);
            telemetry.update();
        }

    }
}
