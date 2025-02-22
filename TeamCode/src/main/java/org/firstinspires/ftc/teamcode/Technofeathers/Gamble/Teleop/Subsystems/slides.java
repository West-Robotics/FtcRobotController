package org.firstinspires.ftc.teamcode.Technofeathers.Gamble.Teleop.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.apache.commons.math3.geometry.euclidean.twod.Line;
import org.firstinspires.ftc.teamcode.Controller;
@TeleOp(name = "SlideConfig")
public class slides extends LinearOpMode {
    public DcMotor verticalLeftLift;

    public DcMotor verticalRightLift;

    public Servo pivotSlide;
    public Servo pivotClaw;

    Controller controller1;
    public void runOpMode(){

        controller1 = new Controller(gamepad1);
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
        waitForStart();
        while (opModeIsActive()){
            controller1.update();
            double power = controller1.left_stick_y/1.2;
            verticalLeftLift.setPower(power);
            verticalRightLift.setPower(power);
            if (controller1.dpadRightOnce()){
                pivotClaw.setPosition(0.75);
            }
            if (controller1.dpadLeftOnce()){
                pivotClaw.setPosition(0.5);
            }
            if (controller1.dpadUpOnce()){
                pivotClaw.setPosition(1);
            }
            if (controller1.dpadDownOnce()){
                pivotSlide.setPosition(0.1);
                pivotClaw.setPosition(0);
            }
            telemetry.addData("pivot",pivotClaw.getPosition());
            telemetry.addData("rightlift",verticalRightLift.getCurrentPosition());
            telemetry.addData("leflift",verticalLeftLift.getCurrentPosition());
            telemetry.update();

        }
    }

}
