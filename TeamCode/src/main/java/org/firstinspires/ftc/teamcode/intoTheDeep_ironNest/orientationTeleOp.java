package org.firstinspires.ftc.teamcode.intoTheDeep_ironNest;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Controller;

@TeleOp
public class orientationTeleOp extends LinearOpMode {
    public DcMotor leftFront;
    public DcMotor leftBack;
    public DcMotor rightFront;
    public DcMotor rightBack;

    public Controller controller1;


    public void runOpMode() {

        controller1 = new Controller(gamepad1);

        leftFront = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBack = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFront = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBack = hardwareMap.get(DcMotor.class, "right_back_drive");
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while(opModeIsActive()){
            controller1.update();


            // UP DOWN RIGHT LEFT movement

            if (controller1.dpadUp()){
                power(1);
            } else if (controller1.dpadDown()) {
                power(-1);
            } else if (controller1.dpadRight()) {

            } else if (controller1.dpadLeft()){

            } else {
                power(0);
            }



            //TANK DRIVE
            /*
            rightBack.setPower(controller1.right_trigger);
            rightFront.setPower(controller1.right_trigger);
            leftBack.setPower(controller1.left_trigger);
            leftFront.setPower(controller1.left_trigger);

             */


            




        }
    }

    public void power(double x){
        leftFront.setPower(x);
        leftBack.setPower(x);
        rightFront.setPower(x);
        rightBack.setPower(x);
    }
}
