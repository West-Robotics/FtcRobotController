package org.firstinspires.ftc.teamcode.seventh.opmode.test;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;
import com.sun.source.doctree.StartElementTree;

import org.firstinspires.ftc.teamcode.seventh.robot.command.ScoreCommand;
import org.firstinspires.ftc.teamcode.seventh.robot.hardware.Hardware;
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.LiftSubsystem;
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.OuttakeSubsystem;

@TeleOp(name = "ScoreTest")
public class ScoreTest extends LinearOpMode {

    Hardware hardware = Hardware.getInstance();
    ScoreCommand score;
    GamepadEx gamepad;
    ScoreCommand.ScoreState state = ScoreCommand.ScoreState.TRANSFER;
    StateMachine machine = new StateMachineBuilder()
            .state(ScoreCommand.ScoreState.TRANSFER)
            .onEnter(() -> state = ScoreCommand.ScoreState.TRANSFER)
            .transition(() -> gamepad.getButton(GamepadKeys.Button.LEFT_BUMPER), ScoreCommand.ScoreState.LOCK)
            .state(ScoreCommand.ScoreState.LOCK)
            .onEnter(() -> state = ScoreCommand.ScoreState.LOCK)
            .transition(() -> gamepad.getButton(GamepadKeys.Button.RIGHT_BUMPER), ScoreCommand.ScoreState.OUTTAKE_READY)
            .state(ScoreCommand.ScoreState.OUTTAKE_READY)
            .onEnter(() -> state = ScoreCommand.ScoreState.OUTTAKE_READY)
            .transition(() -> gamepad.getButton(GamepadKeys.Button.DPAD_DOWN), ScoreCommand.ScoreState.OUTTAKE_DROP)
            .transition(() -> gamepad.getButton(GamepadKeys.Button.DPAD_LEFT), ScoreCommand.ScoreState.OUTTAKE_DROP_L)
            .transition(() -> gamepad.getButton(GamepadKeys.Button.DPAD_RIGHT), ScoreCommand.ScoreState.OUTTAKE_DROP_R)
            .state(ScoreCommand.ScoreState.OUTTAKE_DROP)
            .onEnter(() -> state = ScoreCommand.ScoreState.OUTTAKE_DROP)
            .transition(() -> gamepad.getButton(GamepadKeys.Button.DPAD_UP), ScoreCommand.ScoreState.TRANSFER)
            .state(ScoreCommand.ScoreState.OUTTAKE_DROP_L)
            .onEnter(() -> state = ScoreCommand.ScoreState.OUTTAKE_DROP_L)
            .transition(() -> gamepad.getButton(GamepadKeys.Button.DPAD_UP), ScoreCommand.ScoreState.TRANSFER)
            .state(ScoreCommand.ScoreState.OUTTAKE_DROP_R)
            .onEnter(() -> state = ScoreCommand.ScoreState.OUTTAKE_DROP_R)
            .transition(() -> gamepad.getButton(GamepadKeys.Button.DPAD_UP), ScoreCommand.ScoreState.TRANSFER)
            .build();

    @Override
    public void runOpMode() throws InterruptedException {
        hardware.init(hardwareMap);
        score = new ScoreCommand(hardware);
        gamepad = new GamepadEx(gamepad1);

        waitForStart();

        machine.start();
        while (opModeIsActive() && !isStopRequested()) {
            machine.update();
            score.update(state, gamepad.getLeftY());
            telemetry.addData("power", gamepad.getLeftY());
//            telemetry.addData("dist", score.getDist());
            telemetry.addData("state", state.toString());
            telemetry.addData("position of pivot", hardware.pivot.getPosition());
            telemetry.update();
        }
    }
}
