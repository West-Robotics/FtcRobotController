package org.firstinspires.ftc.teamcode.Technofeathers.Gamble.Teleop.Subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Controller;
import org.firstinspires.ftc.teamcode.Technofeathers.TechnofeathersDrive;

@TeleOp(name = "AutomationForServoing")
public class otherpositions extends LinearOpMode{
    public DcMotor frontLeft;

    public DcMotor verticalLeftLift;

    public DcMotor verticalRightLift;

    public Controller controller1;
    public Servo pivotClaw;
    public Servo pivotSlide;

    public Servo linkageServoLeft;
    public Servo linkageServoRight;

    public Servo diffyRotatorLeft;
    public Servo diffyRotatorRight;

    public Servo grabber;
    public Servo horizontalgrabber;

    public double rightValue;
    public double leftValue;


    public Controller controller2;
    public boolean buttonA;
    public boolean buttonX;
    public boolean buttonY;
    public boolean buttonBforcontroller1;

    public TechnofeathersDrive drive;

    public double rightHorizontalGrabberServoValue;
    public double leftHorizontalGrabberServoValue;
    public boolean servoChanged;
    public boolean canRotateClawVertically;
    public boolean canRotateClawHorizontally;
    public boolean canlinkage;
    public double linkagerightval;
    public double linkageleftval;


    public void runOpMode(){
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();
        controller1 = new Controller(gamepad1);
        controller2 = new Controller(gamepad2);

        drive = new TechnofeathersDrive();

        linkageServoLeft = hardwareMap.get(Servo.class, "linkageServoLeft");
        linkageServoRight = hardwareMap.get(Servo.class, "linkageServoRight");

        frontLeft = hardwareMap.get(DcMotor.class,"frontLeft");
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        pivotClaw = hardwareMap.get(Servo.class,"pivotClaw");
        pivotSlide = hardwareMap.get(Servo.class,"pivotSlide");

        diffyRotatorLeft = hardwareMap.get(Servo.class, "diffyRotatorLeft");
        diffyRotatorRight = hardwareMap.get(Servo.class, "diffyRotatorRight");
        diffyRotatorLeft.setDirection(Servo.Direction.REVERSE);
        diffyRotatorRight.setDirection(Servo.Direction.FORWARD);

        grabber = hardwareMap.get(Servo.class, "grabber");
        horizontalgrabber = hardwareMap.get(Servo.class,"horizontalGrabber");

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

        drive.setupMotors(hardwareMap);

        buttonA = false;
        buttonX = false;
        buttonY = false;
        buttonBforcontroller1 = false;

        servoChanged = false;
        canRotateClawVertically = true;
        canRotateClawHorizontally = false;

        canlinkage = true;

        rightHorizontalGrabberServoValue = 0.05;
        leftHorizontalGrabberServoValue = 0.05;

        rightValue = 1;
        leftValue = 0;
        waitForStart();
        diffyRotatorLeft.setPosition(0.05);
        diffyRotatorRight.setPosition(0.05);


        while (opModeIsActive()){
            controller1.update();
            controller2.update();

            drive.drive(controller1.left_stick_x, -controller1.left_stick_y/1.25, controller1.right_stick_x/1.25);

            //vertical slides
            double power = controller2.left_stick_y/1.2;
            double rightpos = verticalRightLift.getCurrentPosition();
            double leftpos = verticalLeftLift.getCurrentPosition();
            if ( ((rightpos>-2900) && (power<=0)) || ((rightpos<80) && (power>=0)) ){
                verticalLeftLift.setPower(power);
                verticalRightLift.setPower(power);
            } else {
                verticalLeftLift.setPower(0);
                verticalRightLift.setPower(0);
            }

            //scoring and intake pivot positions
            /*
            if (controller2.XOnce()){
                if (buttonX){
                    pivotSlide.setPosition(0.25);
                    pivotClaw.setPosition(0.5);
                    buttonX = false;
                } else{
                    pivotSlide.setPosition(1);
                    pivotClaw.setPosition(0.4);
                    buttonX = true;
                }
            }

             */
            if (controller2.XOnce()){
                pivotSlide.setPosition(0.25);
                pivotClaw.setPosition(0.5);
            }
            if (controller2.BOnce()){
                pivotSlide.setPosition(1);
                pivotClaw.setPosition(0.4);
            }
            if (controller2.YOnce()){
                pivotSlide.setPosition(0.65);
                pivotClaw.setPosition(0.15);
                verticalRightLift.setTargetPosition(-100);
                verticalLeftLift.setTargetPosition(-100);
                verticalLeftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                verticalRightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                verticalLeftLift.setPower(0.4);
                verticalRightLift.setPower(0.4);
                while(opModeIsActive() && verticalRightLift.isBusy() || verticalLeftLift.isBusy()){
                    idle();
                }
                verticalLeftLift.setPower(0);
                verticalRightLift.setPower(0);

                verticalRightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                verticalLeftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                grabber.setPosition(0.5);
                sleep(500);
                linkagerightval = 0.9;
                linkageleftval = 0.1;
                linkageServoRight.setPosition(linkagerightval);
                linkageServoLeft.setPosition(linkageleftval);


            }

            //vertical grabber
            if(controller2.AOnce()){
                if (buttonA){
                    grabber.setPosition(0.5);
                    buttonA = false;
                } else{
                    grabber.setPosition(0.3);
                    buttonA = true;
                }
            }

            if (controller2.dpadUpOnce()){
                pivotSlide.setPosition(0);
                pivotClaw.setPosition(0.4);
            }

            //horizontal pivot for grabber
            if (controller1.dpadRightOnce()&& canRotateClawHorizontally){
                rightHorizontalGrabberServoValue -= 0.05;
                leftHorizontalGrabberServoValue +=0.05;
                servoChanged = true;

            }

            if (controller1.dpadLeftOnce()&& canRotateClawHorizontally){
                rightHorizontalGrabberServoValue +=0.05;
                leftHorizontalGrabberServoValue -=0.05;
                servoChanged = true;
            }

            if (controller1.dpadUpOnce()){
                rightHorizontalGrabberServoValue =0.5;
                leftHorizontalGrabberServoValue =0.5;
                servoChanged = true;
            }


            if (controller1.dpadDownOnce()){
                leftHorizontalGrabberServoValue = 0.05;
                rightHorizontalGrabberServoValue =0.05;
                servoChanged = true;
            }



            //linkage
            if (controller1.XOnce() ){
                leftHorizontalGrabberServoValue = 0.05;
                rightHorizontalGrabberServoValue =0.05;
                diffyRotatorLeft.setPosition(leftHorizontalGrabberServoValue);
                diffyRotatorRight.setPosition(rightHorizontalGrabberServoValue);
                canRotateClawHorizontally = false;
                sleep(750);
                linkageServoLeft.setPosition(0.55); //back
                linkageServoRight.setPosition(0.45);
                linkagerightval = 0.45;
                linkageleftval = 0.55;
            }
            if (controller1.YOnce() ){
                linkageServoLeft.setPosition(0);  //front
                linkageServoRight.setPosition(1);
                linkagerightval = 1;
                linkageleftval = 0;
                sleep(750);
                leftHorizontalGrabberServoValue = 0.5;
                rightHorizontalGrabberServoValue =0.5;
                diffyRotatorLeft.setPosition(leftHorizontalGrabberServoValue);
                diffyRotatorRight.setPosition(rightHorizontalGrabberServoValue);
                canRotateClawHorizontally = true;

            }
            if (controller1.BOnce()) {
                if (buttonBforcontroller1) {
                    horizontalgrabber.setPosition(0.13);
                    buttonBforcontroller1 = false;
                } else {
                    horizontalgrabber.setPosition(0.23);
                    buttonBforcontroller1 = true;
                }
            }
            if (controller1.rightBumper()&&linkageleftval>0){
                linkageleftval-=0.001;
                linkagerightval +=0.001;
                linkageServoRight.setPosition(linkagerightval);
                linkageServoLeft.setPosition(linkageleftval);
            }
            if (controller1.leftBumper()){
                linkageleftval+=0.001;
                linkagerightval -=0.001;
                linkageServoRight.setPosition(linkagerightval);
                linkageServoLeft.setPosition(linkageleftval);
            }

            if (servoChanged){
                diffyRotatorLeft.setPosition(leftHorizontalGrabberServoValue);
                diffyRotatorRight.setPosition(rightHorizontalGrabberServoValue);
                servoChanged = false;

            }
            telemetry.addData("power",controller1.left_stick_y/1.5);
            telemetry.addData("right linkage servo",linkageServoRight.getPosition());
            telemetry.addData("left linkage Servo",linkageServoLeft.getPosition());
            telemetry.addData("grabber",grabber.getPosition());
            telemetry.addData("horizontal grabber",horizontalgrabber.getPosition());
            telemetry.addData("right diffy",diffyRotatorRight.getPosition());
            telemetry.addData("left diffy",diffyRotatorLeft.getPosition());
            telemetry.addData("pivotslide",pivotSlide.getPosition());
            telemetry.addData("pivotClaw",pivotClaw.getPosition());
            telemetry.addData("right motor", rightpos);
            telemetry.addData("left motor", leftpos);

            telemetry.addData("rightbumper",controller1.rightBumper());
            telemetry.addData("right trigger",controller1.right_trigger);
            telemetry.update();

        }
    }

}
