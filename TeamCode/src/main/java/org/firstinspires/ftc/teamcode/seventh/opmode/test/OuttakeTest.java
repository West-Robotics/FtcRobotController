package org.firstinspires.ftc.teamcode.seventh.opmode.test;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;
import com.sun.source.doctree.StartElementTree;

import org.firstinspires.ftc.teamcode.seventh.robot.hardware.Hardware;
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.OuttakeSubsystem;

@TeleOp(name = "OuttakeTest")
public class OuttakeTest extends LinearOpMode {

    Hardware hardware = Hardware.getInstance();
    OuttakeSubsystem out;

    GamepadEx gamepad;

    @Override
    public void runOpMode() throws InterruptedException {
        hardware.init(hardwareMap);
        gamepad = new GamepadEx(gamepad1);
        out = new OuttakeSubsystem(hardware);

//        hardware.pivot.setDirection(Servo.Direction.FORWARD);
//        hardware.pivot.setPwmRange(new PwmControl.PwmRange(500, 2500));
//        hardware.fingerLeft.setDirection(Servo.Direction.FORWARD);
//        hardware.fingerLeft.setPwmRange(new PwmControl.PwmRange(500, 2500));
//        hardware.fingerRight.setDirection(Servo.Direction.REVERSE);
//        hardware.fingerRight.setPwmRange(new PwmControl.PwmRange(500, 2500));
//        double position = 0.5;

        waitForStart();
        hardware.intake.setDirection(DcMotorSimple.Direction.FORWARD);
        hardware.intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        hardware.intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (opModeIsActive() && !isStopRequested()) {
//            if (gamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) < 0.5) {
//                position = 0.5;
//            } else {
//                position = gamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);
//            }
//            hardware.pivot.setPosition(gamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER));
//            hardware.fingerLeft.setPosition(gamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)+0.4);
//            hardware.fingerRight.setPosition(gamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)+0.34);
            if (gamepad.getButton(GamepadKeys.Button.A)) {
                out.update(OuttakeSubsystem.OuttakeState.TRANSFER);
            } else if (gamepad.getButton(GamepadKeys.Button.B)) {
                out.update(OuttakeSubsystem.OuttakeState.LOCK);
            } else if (gamepad.getButton(GamepadKeys.Button.X)) {
                out.update(OuttakeSubsystem.OuttakeState.INTERMEDIARY);
            } else if (gamepad.getButton(GamepadKeys.Button.Y)) {
                out.update(OuttakeSubsystem.OuttakeState.OUTTAKE_READY);
            } else if (gamepad.getButton(GamepadKeys.Button.DPAD_DOWN)) {
                out.update(OuttakeSubsystem.OuttakeState.OUTTAKE_DROP);
            }
            if (gamepad.getButton(GamepadKeys.Button.LEFT_BUMPER)) {
                hardware.intake.setPower(-1);
            } else if (gamepad.getButton(GamepadKeys.Button.RIGHT_BUMPER)) {
                hardware.intake.setPower(0);
            } else if (gamepad.getButton(GamepadKeys.Button.DPAD_LEFT)) {
                hardware.intake.setPower(0.2);
            }
            telemetry.addData("pivot pos", hardware.pivot.getPosition());
            telemetry.addData("left pos", hardware.fingerLeft.getPosition());
            telemetry.addData("right pos", hardware.fingerRight.getPosition());
            telemetry.update();
        }
    }
}
