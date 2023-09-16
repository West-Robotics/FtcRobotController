package org.firstinspires.ftc.teamcode.seventh.opmode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.teamcode.seventh.robot.hardware.Hardware;
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.IntakeSubsystem;

public class IntakeTest extends LinearOpMode {

    Hardware hardware = Hardware.getInstance();
    IntakeSubsystem intake;
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
            .onEnter(() -> intake.update(IntakeSubsystem.IntakeState.STOP, outerState))
            .transition(() -> gamepad1.a, LinearStates.INTAKE)
            .transition(() -> gamepad1.y, LinearStates.SPIT)
            .state(LinearStates.INTAKE)
            .onEnter(() -> intake.update(IntakeSubsystem.IntakeState.INTAKE, outerState))
            .transition(() -> gamepad1.b, LinearStates.COLLECTED)
            .transition(() -> gamepad1.y, LinearStates.SPIT)
            .state(LinearStates.SPIT)
            .onEnter(() -> intake.update(IntakeSubsystem.IntakeState.SPIT, outerState))
            .transition(() -> gamepad1.a, LinearStates.INTAKE)
            .transition(() -> gamepad1.b, LinearStates.COLLECTED)
            .build();
    StateMachine outerMachine = new StateMachineBuilder()
            .state(OuterStates.RAISE)
            .onEnter(() -> outerState = intake.higher(outerState))
            .transition(() -> gamepad1.dpad_down, OuterStates.LOWER)
            .transition(() -> gamepad1.dpad_up, OuterStates.RAISE)
            .state(OuterStates.LOWER)
            .onEnter(() -> outerState = intake.lower(outerState))
            .transition(() -> gamepad1.dpad_down, OuterStates.LOWER)
            .transition(() -> gamepad1.dpad_up, OuterStates.RAISE)
            .build();


    @Override
    public void runOpMode() throws InterruptedException {
        hardware.init(hardwareMap);
        intake = new IntakeSubsystem(hardware);

        waitForStart();

        machine.start();
        outerMachine.start();
        while (opModeIsActive() && !isStopRequested()) {
            machine.update();
            outerMachine.start();
        }
    }
}
