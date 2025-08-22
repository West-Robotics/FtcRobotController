package org.firstinspires.ftc.teamcode.orientation;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.Controller;

@TeleOp
public class orientTele extends LinearOpMode {

    public Controller controller1;

    public DcMotor frontLeft;
    public DcMotor frontRight;
    public DcMotor backLeft;
    public DcMotor backRight;
    public void runOpMode() throws InterruptedException {
        controller1 = new Controller(gamepad1);
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setPower(0);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setPower(0);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setPower(0);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        backRight = hardwareMap.get(DcMotor.class, "backRight");
        backRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setPower(0);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        waitForStart();

        while (opModeIsActive()) {
            controller1.update();
            frontLeft.setPower(-controller1.left_stick_y);
            frontRight.setPower(controller1.left_stick_y);
            backLeft.setPower(-controller1.left_stick_y);
            backRight.setPower(controller1.left_stick_y);
            telemetry.addData("left joy stick y", controller1.left_stick_y);
            telemetry.addData("left joy stick x", controller1.left_stick_x);
            telemetry.addData("right joy stick y", controller1.right_stick_y);
            telemetry.addData("right joy stick x", controller1.right_stick_x);
            telemetry.addData("right trigger", controller1.right_trigger);
            telemetry.addData("left trigger", controller1.left_trigger);
            telemetry.addData("left bumper", controller1.leftBumper());
            telemetry.addData("right bumper", controller1.rightBumper());
            telemetry.update();
        }
    }
}
