package org.firstinspires.ftc.teamcode.seventh.opmode.test;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;
import com.sun.source.doctree.StartElementTree;

import org.firstinspires.ftc.teamcode.seventh.robot.hardware.Hardware;
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.IntakeSubsystem;

//@TeleOp(name =
//    "🟦🟦🟦🟦🟦🟦🟦🟦🟦⬛⬛⬛⬛⬛🟦🟦🟦🟦🟦🟦🟦\n" +
//    "🟦🟦🟦🟦🟦🟦🟦🟦⬛⬛🟥🟥🟥⬛⬛⬛🟦🟦🟦🟦🟦\n" +
//    "🟦🟦🟦🟦🟦🟦🟦⬛⬛🟥🟥🟥🟥🟥🟥🟥⬛🟦🟦🟦🟦\n" +
//    "🟦🟦🟦🟦🟦🟦🟦⬛🟥🟥🟥🟥🟥🟥🟥🟥⬛🟦🟦🟦🟦\n" +
//    "🟦🟦🟦🟦🟦🟦🟦⬛🟥🟥🟥🟥🟥🟥🟥🟥🟥⬛🟦🟦🟦\n" +
//    "🟦🟦🟦🟦🟦🟦🟦⬛🟥🟥🟥⬜⬜⬜⬜⬜⬜⬛⬛🟦🟦\n" +
//    "🟦🟦🟦🟦⬛⬛⬛🟥🟥🟥⬜⬜⬜⬜⬜⬜⬜⬜⬛🟦🟦\n" +
//    "🟦🟦🟦⬛🟥⬛⬛🟥🟥🟥⬜⬜⬜⬜⬜⬜⬜⬜⬛🟦🟦\n" +
//    "🟦🟦🟦⬛🟥⬛⬛🟥🟥🟥⬛⬜⬜⬜⬜⬜⬜⬜⬛🟦🟦\n" +
//    "🟦🟦⬛⬛🟥⬛⬛🟥🟥🟥🟥⬛⬛⬛⬛⬛⬛⬛🟦🟦🟦\n" +
//    "🟦🟦⬛⬛🟥⬛⬛🟥🟥🟥🟥🟥🟥🟥🟥🟥🟥⬛🟦🟦🟦\n" +
//    "🟦🟦⬛⬛🟥⬛⬛🟥🟥🟥🟥🟥🟥🟥🟥🟥🟥⬛🟦🟦🟦\n" +
//    "🟦🟦⬛⬛🟥⬛⬛🟥🟥🟥🟥🟥🟥🟥🟥🟥🟥⬛🟦🟦🟦\n" +
//    "🟦🟦⬛⬛🟥⬛⬛🟥🟥🟥🟥🟥🟥🟥🟥🟥🟥⬛🟦🟦🟦\n" +
//    "🟦🟦⬛⬛🟥⬛⬛🟥🟥🟥🟥🟥🟥🟥🟥🟥🟥⬛🟦🟦🟦\n" +
//    "🟦🟦⬛⬛🟥⬛⬛🟥🟥🟥🟥🟥🟥🟥🟥🟥🟥⬛🟦🟦🟦\n" +
//    "🟦🟦⬛⬛🟥⬛⬛🟥🟥🟥🟥🟥🟥🟥🟥🟥🟥⬛🟦🟦🟦\n" +
//    "🟦🟦🟦⬛⬛⬛⬛🟥🟥🟥🟥⬛⬛🟥🟥🟥🟥⬛🟦🟦🟦\n" +
//    "🟦🟦🟦🟦🟦⬛⬛🟥🟥🟥🟥⬛⬛🟥🟥🟥🟥⬛🟦🟦🟦\n" +
//    "🟦🟦🟦🟦🟦⬛⬛⬛🟥🟥⬛⬛⬛🟥🟥🟥⬛🟦🟦🟦🟦\n" +
//    "🟦🟦🟦🟦🟦🟦🟦⬛⬛⬛⬛🟦⬛⬛⬛⬛⬛🟦🟦🟦🟦")
@TeleOp(name = ":(")
public class IntakeTest extends LinearOpMode {

    Hardware hardware = Hardware.getInstance();
    IntakeSubsystem intake;
    IntakeSubsystem.IntakeState intakeState = IntakeSubsystem.IntakeState.STOP;
    IntakeSubsystem.OuterState outerState = IntakeSubsystem.OuterState.STACK_1;
    enum LinearStates {
        INTAKE,
        COLLECTED,
        SPIT
    }
    enum OuterStates {
        RAISE,
        LOWER
    }
    StateMachine machine = new StateMachineBuilder()
            .state(LinearStates.COLLECTED)
            .onEnter(() -> intakeState = IntakeSubsystem.IntakeState.STOP)
            .transition(() -> gamepad1.a, LinearStates.INTAKE)
            .transition(() -> gamepad1.y, LinearStates.SPIT)
            .state(LinearStates.INTAKE)
            .onEnter(() -> intakeState = IntakeSubsystem.IntakeState.INTAKE)
            .transition(() -> gamepad1.b, LinearStates.COLLECTED)
            .transition(() -> gamepad1.y, LinearStates.SPIT)
            .state(LinearStates.SPIT)
            .onEnter(() -> intakeState = IntakeSubsystem.IntakeState.SPIT)
            .transition(() -> gamepad1.a, LinearStates.INTAKE)
            .transition(() -> gamepad1.b, LinearStates.COLLECTED)
            .build();

    GamepadEx gamepad;

    @Override
    public void runOpMode() throws InterruptedException {
        hardware.init(hardwareMap);
        intake = new IntakeSubsystem(hardware);
        gamepad = new GamepadEx(gamepad1);

        waitForStart();

        machine.start();
        while (opModeIsActive() && !isStopRequested()) {
            machine.update();
            intake.update(intakeState, outerState);
            gamepad.readButtons();
            if (gamepad.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
                outerState = intake.lower(outerState);
            } else if (gamepad.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                outerState = intake.higher(outerState);
            }
            telemetry.addData("state", machine.getState().name());
            telemetry.addData("outer state", outerState.name());
            telemetry.addData("left pos", hardware.outerPivotLeft.getPosition());
            telemetry.addData("right pos", hardware.outerPivotRight.getPosition());
            telemetry.update();
        }
    }
}
