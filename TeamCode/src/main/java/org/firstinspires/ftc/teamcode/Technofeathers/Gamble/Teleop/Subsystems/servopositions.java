package org.firstinspires.ftc.teamcode.Technofeathers.Gamble.Teleop.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Controller;

@TeleOp(name = "AutomationForServo")
public class servopositions extends LinearOpMode{
    public DcMotor frontLeft;

    public DcMotor verticalLeftLift;

    public DcMotor verticalRightLift;

    public Controller controller1;
    public Servo pivotclaw;
    public Servo pivotslide;
    public void runOpMode(){
        controller1 = new Controller(gamepad1);

        frontLeft = hardwareMap.get(DcMotor.class,"frontLeft");
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        pivotclaw = hardwareMap.get(Servo.class,"pivotClaw");
        pivotslide = hardwareMap.get(Servo.class,"pivotSlide");

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

        waitForStart();
        while (opModeIsActive()){
            controller1.update();
            verticalLeftLift.setPower(controller1.left_stick_y/1.5);
            verticalRightLift.setPower(controller1.left_stick_y/1.5);


            if (controller1.AOnce()){
                pivotslide.setPosition(0.965);
                pivotclaw.setPosition(0.5);
            }
            if (controller1.BOnce()){
                pivotslide.setPosition(0.2);
                pivotclaw.setPosition(0.2);
            }


            telemetry.addData("fronleft motor",frontLeft.getCurrentPosition());
            telemetry.addData("right motor", verticalRightLift.getCurrentPosition());
            telemetry.addData("left motor", verticalLeftLift.getCurrentPosition());
            telemetry.update();
        }
    }

}
