// TODO: Add PID tuning with controller (methods for changing variables are done already)
// TODO: Update preset values, add presets for stack
// TODO: Add slow descent + encoder reset on limit switch
// TODO: Set up all FSMs
// TODO: Tune servo positions for FSMs
// TODO: Tune drive PID
package org.firstinspires.ftc.teamcode.WizardBot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;

import org.firstinspires.ftc.teamcode.Controller;

@Disabled
@TeleOp(name = "Manual Wizard Teleop")
public class ManualWizardTeleop extends OpMode {

    private WizardBotDrive drive;
    private Controller controller;
    private Controller controller2;
    private ServoEx clawServo;
    private ServoEx wristServo;
    private ServoEx pivotServo1;
    private ServoEx pivotServo2;
    private DcMotor lift1;
    private DcMotor lift2;
    private DcMotor intake1;
    private DcMotor intake2;
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    int slowMode = 0;
    int pivotServo1Pos = 180;
    int pivotServo2Pos = 180;
    int clawServoPos = 180;

    public ManualWizardTeleop() {
    }

    @Override
    public void init() {
        drive = new WizardBotDrive(this, hardwareMap);
        controller = new Controller(gamepad1);
        controller2 = new Controller(gamepad2);

        clawServo = new SimpleServo(hardwareMap, "clawServo", 0, 360);
        wristServo = new SimpleServo(hardwareMap, "wristServo", 0, 360);
        pivotServo1 = new SimpleServo(hardwareMap, "pivotServo1", 0, 360);
        pivotServo2 = new SimpleServo(hardwareMap, "pivotServo2", 0, 360);
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        lift1 = hardwareMap.get(DcMotor.class, "lift1");
        lift2 = hardwareMap.get(DcMotor.class, "lift2");
        intake1 = hardwareMap.get(DcMotor.class, "intake1");
        intake2 = hardwareMap.get(DcMotor.class, "intake2");
    }

    @Override
    public void loop() {
        controller.update();
        controller2.update();

        telemetry.addData("LiftEncdr", lift1.getCurrentPosition());
        telemetry.addData("LiftEncdr2", lift2.getCurrentPosition());
        telemetry.addData("SlowMode?", slowMode);
        telemetry.addData("pivot1Pos", pivotServo1Pos);
        telemetry.addData("pivot2Pos", pivotServo2Pos);
        telemetry.addData("clawServoPos", clawServoPos);
        telemetry.update();

        // Controls
        // Driver 1: Left Trigger hold to intake, release to score, Right Trigger score
        // Driver 2: Buttons for HorizontalLift, Triggers for fine control

        //Drive Controls
        if (controller.leftStickButtonOnce()) {
            drive.FieldCentricToggle();
        } else if (controller.rightStickButtonOnce()) {
            if (slowMode == 0) {
                slowMode = 1;
            } else {
                slowMode = 0;
            }
        }
        if (slowMode == 0) {
            drive.drive(-controller.left_stick_x, -controller.left_stick_y, -controller.right_stick_x);
        } else {
            drive.drive(-controller.left_stick_x/2, -controller.left_stick_y/2, -controller.right_stick_x /2);
        }

        //HorizontalLift Controls
        if (controller.left_trigger == 1) {
            lift1.setPower(-.1);
            lift2.setPower(.1);
        } else if (controller.right_trigger == 1) {
            lift1.setPower(1);
            lift2.setPower(-1);
        } else {
            lift1.setPower(0);
            lift2.setPower(0);
        }

        //Claw Controls
        if (controller.AOnce()) {
            clawServoPos = 180;
        } else if (controller.BOnce()) {
            clawServoPos = 155;
        }
        clawServo.turnToAngle(clawServoPos);

        //Intake Controls
        if (controller.dpadUpOnce()) {
            intake1.setPower(.5);
            intake2.setPower(-.5);
        } else if (controller.dpadDownOnce()) {
            intake1.setPower(0);
            intake2.setPower(0);
        }

        //Wrist Pivot Controls
        if (controller.leftBumperOnce()) {
            pivotServo1Pos = pivotServo1Pos+5;
            pivotServo2Pos = pivotServo2Pos-5;
        } else if (controller.rightBumperOnce()) {
            pivotServo1Pos = 120;
            pivotServo2Pos = 240;
        } else if (controller.dpadLeftOnce()) {
            pivotServo1Pos = 140;
            pivotServo2Pos = 220;
        } else if (controller.dpadRightOnce()) {
            pivotServo1Pos = 310;
            pivotServo2Pos = 50;
        }
        pivotServo1.turnToAngle(pivotServo1Pos);
        pivotServo2.turnToAngle(pivotServo2Pos);
        //140 1 220 2 Intake
        //310 1 50 2 Outtake
        //Driving 260 1 100 2

        //Wrist Twist Controls
        if (controller.XOnce()) {
            wristServo.turnToAngle(16);
        } else if (controller.YOnce()) {
            wristServo.turnToAngle(256);
        }
    }
}

