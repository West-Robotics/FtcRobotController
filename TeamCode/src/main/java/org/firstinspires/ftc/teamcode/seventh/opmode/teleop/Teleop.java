package org.firstinspires.ftc.teamcode.seventh.opmode.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.seventh.robot.command.CycleCommand;
import org.firstinspires.ftc.teamcode.seventh.robot.hardware.Hardware;
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.LiftSubsystem;
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.OutputSubsystem;

@TeleOp(name = "SussyOp")
public class Teleop extends LinearOpMode {

    // IDEAS:
    // automatically flip driving direction
    // lock robot to face exactly backdrop
    // mecanum feedforward

    Hardware hardware = Hardware.getInstance();
    public SampleMecanumDrive drive;
    CycleCommand cycle;
    IntakeSubsystem intake;
    LiftSubsystem lift;
    OutputSubsystem out;

    GamepadEx primary;
    GamepadEx secondary;
    CycleCommand.CycleState cycleState = CycleCommand.CycleState.INTAKE;
    StateMachine cycleMachine = new StateMachineBuilder()
            .state(CycleCommand.CycleState.LOCK)
            .transition(() -> secondary.wasJustPressed(GamepadKeys.Button.A), CycleCommand.CycleState.INTAKE)
            .transition(() -> secondary.wasJustPressed(GamepadKeys.Button.DPAD_UP), CycleCommand.CycleState.READY)
            .transition(() -> secondary.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER), CycleCommand.CycleState.SPIT)
            .state(CycleCommand.CycleState.INTAKE)
            .transition(() -> secondary.wasJustPressed(GamepadKeys.Button.B), CycleCommand.CycleState.LOCK)
            .transition(() -> secondary.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER), CycleCommand.CycleState.SPIT)
            .state(CycleCommand.CycleState.READY)
            .transition(() -> secondary.wasJustPressed(GamepadKeys.Button.DPAD_DOWN), CycleCommand.CycleState.LOCK)
            .state(CycleCommand.CycleState.SPIT)
            .transition(() -> secondary.wasJustPressed(GamepadKeys.Button.A), CycleCommand.CycleState.INTAKE)
            .transition(() -> secondary.wasJustPressed(GamepadKeys.Button.B), CycleCommand.CycleState.LOCK)
            .build();
    StateMachine outMachine = new StateMachineBuilder()
            .state(OutputSubsystem.OutputState.LOCK)
            .transition(() -> secondary.wasJustPressed(GamepadKeys.Button.A), OutputSubsystem.OutputState.INTAKE)
            .transition(() -> secondary.wasJustPressed(GamepadKeys.Button.DPAD_UP), OutputSubsystem.OutputState.READY)
            // this is actually just spit
            .transition(() -> secondary.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER), CycleCommand.CycleState.INTAKE)
            .state(OutputSubsystem.OutputState.INTAKE)
            .transition(() -> secondary.wasJustPressed(GamepadKeys.Button.B), OutputSubsystem.OutputState.LOCK)
            .state(OutputSubsystem.OutputState.READY)
            .transition(() -> secondary.wasJustPressed(GamepadKeys.Button.DPAD_DOWN), OutputSubsystem.OutputState.LOCK)
            .transition(() -> primary.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER), OutputSubsystem.OutputState.DROP)
            .transition(() -> primary.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.9, OutputSubsystem.OutputState.DROP_L)
            .transition(() -> primary.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.9, OutputSubsystem.OutputState.DROP_R)
            .state(OutputSubsystem.OutputState.DROP)
            .transition(() -> secondary.wasJustPressed(GamepadKeys.Button.DPAD_DOWN), OutputSubsystem.OutputState.LOCK)
            .transition(() -> primary.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER), OutputSubsystem.OutputState.READY)
            .transition(() -> primary.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.9, OutputSubsystem.OutputState.DROP_L)
            .transition(() -> primary.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.9, OutputSubsystem.OutputState.DROP_R)
            .state(OutputSubsystem.OutputState.DROP_L)
            .transition(() -> secondary.wasJustPressed(GamepadKeys.Button.DPAD_DOWN), OutputSubsystem.OutputState.LOCK)
            .transition(() -> primary.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER), OutputSubsystem.OutputState.DROP)
            .transition(() -> primary.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.9, OutputSubsystem.OutputState.DROP_R)
            .state(OutputSubsystem.OutputState.DROP_R)
            .transition(() -> secondary.wasJustPressed(GamepadKeys.Button.DPAD_DOWN), OutputSubsystem.OutputState.LOCK)
            .transition(() -> primary.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER), OutputSubsystem.OutputState.DROP)
            .transition(() -> primary.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.9, OutputSubsystem.OutputState.DROP_L)
            .build();

    double loopTime = 0.0;
    double x = 0.0;
    double y = 0.0;
    double turn = 0.0;
    final static double SLEW_RATE = 4;

    @Override
    public void runOpMode() throws InterruptedException {
        hardware.init(hardwareMap);
        drive = new SampleMecanumDrive(hardware, hardwareMap);
        intake = new IntakeSubsystem(hardware);
        lift = new LiftSubsystem(hardware);
        out = new OutputSubsystem(hardware);
        cycle = new CycleCommand(intake, lift, out);
        primary = new GamepadEx(gamepad1);
        secondary = new GamepadEx(gamepad2);

        cycleMachine.start();
        outMachine.start();
        hardware.read(intake, lift, out);
        cycle.update((CycleCommand.CycleState) cycleMachine.getState(), (OutputSubsystem.OutputState) outMachine.getState());
        hardware.write(intake, lift, out);
        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            double loop = System.nanoTime();
            // remember to convert to seconds (multiply by 10^-9) for slew rate lmao
            double dt = loop - loopTime;
            loopTime = loop;

            primary.readButtons();
            secondary.readButtons();

            // update state machines
            cycleMachine.update();
            outMachine.update();

            // update all subsystems
            hardware.read(intake, lift, out);
            cycle.update((CycleCommand.CycleState) cycleMachine.getState(), (OutputSubsystem.OutputState) outMachine.getState());
            hardware.write(intake, lift, out);

            // only change dt powers by at max the slew rate
            if (Math.abs(primary.getLeftY() - x) < SLEW_RATE*dt/1000000000.0) {
                x += primary.getLeftY() - x;
                telemetry.addData("dx", primary.getLeftY() - x);
                telemetry.addLine("less than slew");
            } else {
                x += Math.signum(primary.getLeftY() - x) * SLEW_RATE*dt/1000000000.0;
                telemetry.addData("dx", Math.signum(primary.getLeftY() - x) * SLEW_RATE*dt/1000000000.0);
                telemetry.addLine("greater than slew");
            }
            if (Math.abs(-primary.getLeftX() - y) < SLEW_RATE*dt/1000000000.0) {
                y += -primary.getLeftX() - y;
            } else {
                y += Math.signum(-primary.getLeftX() - y) * SLEW_RATE*dt/1000000000.0;
            }
            if (Math.abs(-primary.getRightX() - turn) < SLEW_RATE*dt/1000000000.0) {
                turn += -primary.getRightX() - turn;
            } else {
                turn += Math.signum(-primary.getRightX() - turn) * SLEW_RATE*dt/1000000000.0;
            }

            if (cycleState == CycleCommand.CycleState.READY
                    || intake.getState() == IntakeSubsystem.IntakeState.INTAKE
                    || primary.isDown(GamepadKeys.Button.LEFT_BUMPER)) {
                drive.setWeightedDrivePower(new Pose2d(Math.pow(x/2, 3), Math.pow(y/2, 3), Math.pow(turn/2, 3)));
            } else {
                drive.setWeightedDrivePower(new Pose2d(Math.pow(x, 3), Math.pow(y, 3), Math.pow(turn/1.5, 3)));
            }
            telemetry.addData("pivot pos", hardware.pivot.getPosition());
            telemetry.addData("left pos", hardware.fingerLeft.getPosition());
            telemetry.addData("right pos", hardware.fingerRight.getPosition());
            telemetry.addData("lift dist", lift.getDistance());
            telemetry.addData("hz ", 1000000000 / dt);
            telemetry.update();
        }
    }
}
