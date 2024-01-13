package org.firstinspires.ftc.teamcode.Wingman;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.Controller;
import org.firstinspires.ftc.teamcode.Wingman.subsystems.Lift;
import org.firstinspires.ftc.teamcode.Wingman.subsystems.MFDrive;

@Disabled
@TeleOp(name = "MF TeleOp")
public class MFTeleop extends OpMode {

    private MFDrive drive;
    private Controller controller;
    private Controller controller2;
    private TouchSensor liftLimit;
    private Servo clawServo;
    private DcMotor liftMotor;
    private DcMotor liftMotor2;
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    public MFTeleop() {
    }

    @Override
    public void init() {
        drive = new MFDrive(this, hardwareMap);
        controller = new Controller(gamepad1);
        Lift lift = new Lift();
        controller2 = new Controller(gamepad2);
        liftLimit = hardwareMap.get(TouchSensor.class, "liftLimit");
        liftMotor = hardwareMap.get(DcMotor.class, "liftMotor");
        liftMotor2 = hardwareMap.get(DcMotor.class, "liftMotor2");
        clawServo = hardwareMap.get(Servo.class, "clawServo");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop() {
        controller.update();
        controller2.update();

        int limit;
        if (liftLimit.isPressed()) {
            limit = 1;
        } else {
            limit = 0;
        }
        telemetry.addData("LiftLimit", limit);
        telemetry.addData("LiftEncdr", liftMotor.getCurrentPosition());
        telemetry.addData("DriveEncdrFL", frontLeft.getCurrentPosition());
        telemetry.addData("DriveEncdrFR", frontRight.getCurrentPosition());
        telemetry.addData("DriveEncdrBL", backLeft.getCurrentPosition());
        telemetry.addData("DriveEncdrBR", backRight.getCurrentPosition());
        telemetry.update();
        if (controller.left_trigger == 1) {
            drive.drive(-controller.left_stick_x * 1 / 4, -controller.left_stick_y * 1 / 4, -controller.right_stick_x * 1 / 3);
        } else {
            drive.drive(-controller.left_stick_x * 2 / 3, -controller.left_stick_y * 2 / 3, -controller.right_stick_x * 2 / 3);
        }

//        telemetry.addData("gyro", gyrog)

        if (controller2.dpadRight() || controller.dpadRight()) {
            if (liftMotor.getCurrentPosition() < -3000) {
                liftMotor.setPower(0);
                liftMotor2.setPower(0);
            } else {
                clawServo.setPosition(0.45);
                liftMotor.setPower(-1);
                liftMotor2.setPower(1);
            }
        } else if (controller2.dpadLeft() || controller.dpadLeft()) {
            if (liftMotor.getCurrentPosition() < -1500) {
                liftMotor.setPower(0);
                liftMotor2.setPower(0);
            } else {
                clawServo.setPosition(0.45);
                liftMotor.setPower(-1);
                liftMotor2.setPower(1);
            }
        } else if (controller2.dpadUp() || controller.dpadUp()) {
            if (liftMotor.getCurrentPosition() < -2250) {
                liftMotor.setPower(0);
                liftMotor2.setPower(0);
            } else {
                clawServo.setPosition(0.45);
                liftMotor.setPower(-1);
                liftMotor2.setPower(1);
            }
        } else if (controller2.X() || controller.X()) {
            if (liftLimit.isPressed()) {
                liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                clawServo.setPosition(0);
                liftMotor.setPower(0);
                liftMotor2.setPower(0);
            } else {
                if (clawServo.getPosition() != 0.45) {
                    clawServo.setPosition(0.45);
                }
                liftMotor.setPower(1);
                liftMotor2.setPower(-1);
            }
        } else if (controller2.left_trigger == 1) {
            if (liftLimit.isPressed()) {
                liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                clawServo.setPosition(0);
                liftMotor.setPower(0);
                liftMotor2.setPower(0);
            } else {
                liftMotor.setPower(0.2);
                liftMotor2.setPower(-0.2);
            }
        } else if (controller2.right_trigger == 1) {
            liftMotor.setPower(-0.4);
            liftMotor2.setPower(0.4);
        } else if (controller2.A()) {
            if (liftMotor.getCurrentPosition() < -600) {
                liftMotor.setPower(0);
                liftMotor2.setPower(0);
            } else {
                clawServo.setPosition(0.45);
                liftMotor.setPower(-1);
                liftMotor2.setPower(1);
            }
        } else {
            liftMotor.setPower(0);
            liftMotor2.setPower(0);

            if (controller.AOnce()) {
                clawServo.setPosition(0.45);
            } else if (controller.BOnce() || controller2.BOnce()) {
                clawServo.setPosition(0.3);
            }
        }
    }
}

