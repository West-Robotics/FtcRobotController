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

import org.firstinspires.ftc.teamcode.seventh.robot.command.ScoreCommand;
import org.firstinspires.ftc.teamcode.seventh.robot.hardware.Hardware;
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.OuttakeSubsystem;

@TeleOp(name = "CycleTest")
public class CycleTest extends LinearOpMode {

    Hardware hardware = Hardware.getInstance();
    ScoreCommand score;

    GamepadEx gamepad;
    ScoreCommand.ScoreState state = ScoreCommand.ScoreState.TRANSFER;
    StateMachine machine = new StateMachineBuilder()
            .state(ScoreCommand.ScoreState.TRANSFER)
            .onEnter(() -> state = ScoreCommand.ScoreState.TRANSFER)
            .transition(() -> gamepad.getButton(GamepadKeys.Button.B), ScoreCommand.ScoreState.LOCK)
            .state(ScoreCommand.ScoreState.LOCK)
            .onEnter(() -> state = ScoreCommand.ScoreState.LOCK)
            .transition(() -> gamepad.getButton(GamepadKeys.Button.X), ScoreCommand.ScoreState.OUTTAKE_READY)
            .state(ScoreCommand.ScoreState.OUTTAKE_READY)
            .onEnter(() -> state = ScoreCommand.ScoreState.OUTTAKE_READY)
            .transition(() -> gamepad.getButton(GamepadKeys.Button.DPAD_DOWN), ScoreCommand.ScoreState.OUTTAKE_DROP)
            .transition(() -> gamepad.getButton(GamepadKeys.Button.DPAD_LEFT), ScoreCommand.ScoreState.OUTTAKE_DROP_L)
            .transition(() -> gamepad.getButton(GamepadKeys.Button.DPAD_RIGHT), ScoreCommand.ScoreState.OUTTAKE_DROP_R)
            .state(ScoreCommand.ScoreState.OUTTAKE_DROP)
            .onEnter(() -> state = ScoreCommand.ScoreState.OUTTAKE_DROP)
            .transition(() -> gamepad.getButton(GamepadKeys.Button.A), ScoreCommand.ScoreState.TRANSFER)
            .state(ScoreCommand.ScoreState.OUTTAKE_DROP_L)
            .onEnter(() -> state = ScoreCommand.ScoreState.OUTTAKE_DROP_L)
            .transition(() -> gamepad.getButton(GamepadKeys.Button.A), ScoreCommand.ScoreState.TRANSFER)
            .state(ScoreCommand.ScoreState.OUTTAKE_DROP_R)
            .onEnter(() -> state = ScoreCommand.ScoreState.OUTTAKE_DROP_R)
            .transition(() -> gamepad.getButton(GamepadKeys.Button.A), ScoreCommand.ScoreState.TRANSFER)
            .build();

    @Override
    public void runOpMode() throws InterruptedException {
        hardware.init(hardwareMap);
        score = new ScoreCommand(hardware);
        gamepad = new GamepadEx(gamepad1);

        waitForStart();
        hardware.intake.setDirection(DcMotorSimple.Direction.FORWARD);
        hardware.intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        hardware.intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        machine.start();
        while (opModeIsActive() && !isStopRequested()) {
            machine.update();
            score.update(state, gamepad.getLeftY());
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
