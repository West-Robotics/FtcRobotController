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
    public double armencoder;


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
        waitForStart();
        while (opModeIsActive()){
            controller1.update();

            drive.drive(controller1.left_stick_x, -controller1.left_stick_y, controller1.right_stick_x);
            telemetry.addData("Side: ", controller1.left_stick_x);
            telemetry.addData("Forward: ", controller1.left_stick_y);
            telemetry.addData("Turn: ", controller1.right_stick_x);
            telemetry.addData("pivot", pivot.getPosition());
            telemetry.addData("grabber", grabber.getPosition());
            telemetry.addData("arm",arm.getCurrentPosition());
            telemetry.update();


            if (controller1.leftBumperOnce()) {
                if (buttonA) {
                    grabber.setPosition(0.15);
                    buttonA = false;
                } else {
                    grabber.setPosition(0.24);
                    buttonA = true;
                }
            }

            if (controller1.YOnce()) {

                arm.setTargetPosition(550);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armencoder = arm.getCurrentPosition();
                while(opModeIsActive()&&(armencoder<300)){
                    armencoder = arm.getCurrentPosition();
                    arm.setPower(0.2);
                }
                pivot.setPosition(0.75) ;
                arm.setPower(0.1);
            }
            if (controller1.XOnce()) {

                arm.setTargetPosition(830);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armencoder = arm.getCurrentPosition();
                while (opModeIsActive() &&(armencoder<400)){
                    armencoder = arm.getCurrentPosition();
                    arm.setPower(0.22);
                }
                pivot.setPosition(0.9);
                arm.setPower(0.1);

            }
            if (controller1.AOnce()){


                arm.setTargetPosition(930);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armencoder = arm.getCurrentPosition();
                while (opModeIsActive()&&(armencoder<400)){
                    armencoder = arm.getCurrentPosition();
                    arm.setPower(0.22);
                }
                pivot.setPosition(0.73);
                arm.setPower(0.1);

            }

            if (controller1.BOnce()) {
                pivot.setPosition(0.18);
                arm.setTargetPosition(80);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armencoder = arm.getCurrentPosition();
                while (opModeIsActive()&&armencoder>750){
                    armencoder = arm.getCurrentPosition();
                    arm.setPower(0.2);
                }

                arm.setPower(0.08);
                while (opModeIsActive()&&(armencoder>80)){
                    armencoder = arm.getCurrentPosition();
                }
            }

            if (controller1.dpadUpOnce()) {
                arm.setPower(0);
                arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            if (controller1.dpadDownOnce()){
                pivot.setPosition(0.45);
            }
            if(controller1.dpadRightOnce()){
                pivot.setPosition(0.25);
            }
        }
    }


}