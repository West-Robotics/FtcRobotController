package org.firstinspires.ftc.teamcode.seventh.opmode.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.teamcode.PIDController;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.seventh.robot.command.ScoreCommand;
import org.firstinspires.ftc.teamcode.seventh.robot.hardware.Globals;
import org.firstinspires.ftc.teamcode.seventh.robot.hardware.Hardware;
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.OutputSubsystem;

@TeleOp(name = "SussyOp")
public class Teleop extends LinearOpMode {

    // IDEAS:
    // automatically flip driving direction
    // lock robot to face exactly backdrop

    Hardware hardware = Hardware.getInstance();
    public SampleMecanumDrive drive;
    PIDController headingPID = new PIDController(Globals.HEADING_P, 0, Globals.HEADING_D);
    ScoreCommand score;
    OutputSubsystem.OutputState outState = OutputSubsystem.OutputState.LOCK;
    IntakeSubsystem intake;
    PIDController liftPid = new PIDController(Globals.LIFT_P, Globals.LIFT_I, Globals.LIFT_D);

    GamepadEx primary;
    GamepadEx secondary;
    ScoreCommand.ScoreState state = ScoreCommand.ScoreState.INTAKE;
    StateMachine machine = new StateMachineBuilder()
            .state(ScoreCommand.ScoreState.LOCK)
            .onEnter(() -> {})
            .transition(() -> secondary.getButton(GamepadKeys.Button.B), ScoreCommand.ScoreState.INTAKE)
            .transition(() -> secondary.getButton(GamepadKeys.Button.DPAD_UP), ScoreCommand.ScoreState.READY)
            .state(ScoreCommand.ScoreState.INTAKE)
            .onEnter(() -> {})
            .transition(() -> secondary.getButton(GamepadKeys.Button.A), ScoreCommand.ScoreState.LOCK)
            .state(ScoreCommand.ScoreState.READY)
            .onEnter(() -> {})
            .transition(() -> secondary.getButton(GamepadKeys.Button.DPAD_DOWN), ScoreCommand.ScoreState.LOCK)
            .build();
    StateMachine outMachine = new StateMachineBuilder()
            .state(OutputSubsystem.OutputState.LOCK)
            .transition(() -> secondary.getButton(GamepadKeys.Button.A), OutputSubsystem.OutputState.INTAKE)
            .transition(() -> secondary.getButton(GamepadKeys.Button.DPAD_UP), OutputSubsystem.OutputState.READY)
            .state(OutputSubsystem.OutputState.INTAKE)
            .transition(() -> secondary.getButton(GamepadKeys.Button.B), OutputSubsystem.OutputState.LOCK)
            .state(OutputSubsystem.OutputState.READY)
            .transition(() -> secondary.getButton(GamepadKeys.Button.DPAD_DOWN), OutputSubsystem.OutputState.LOCK)
            .transition(() -> primary.getButton(GamepadKeys.Button.RIGHT_BUMPER), OutputSubsystem.OutputState.DROP)
            .transition(() -> primary.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.9, OutputSubsystem.OutputState.DROP_L)
            .transition(() -> primary.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.9, OutputSubsystem.OutputState.DROP_R)
            .state(OutputSubsystem.OutputState.DROP)
            .transition(() -> secondary.getButton(GamepadKeys.Button.DPAD_DOWN), OutputSubsystem.OutputState.LOCK)
            .transition(() -> primary.getButton(GamepadKeys.Button.RIGHT_BUMPER), OutputSubsystem.OutputState.READY)
            .transition(() -> primary.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.9, OutputSubsystem.OutputState.DROP_L)
            .transition(() -> primary.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.9, OutputSubsystem.OutputState.DROP_R)
            .state(OutputSubsystem.OutputState.DROP_L)
            .transition(() -> secondary.getButton(GamepadKeys.Button.DPAD_DOWN), OutputSubsystem.OutputState.LOCK)
            .transition(() -> primary.getButton(GamepadKeys.Button.RIGHT_BUMPER), OutputSubsystem.OutputState.DROP)
            .transition(() -> primary.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.9, OutputSubsystem.OutputState.DROP_R)
            .state(OutputSubsystem.OutputState.DROP_R)
            .transition(() -> secondary.getButton(GamepadKeys.Button.DPAD_DOWN), OutputSubsystem.OutputState.LOCK)
            .transition(() -> primary.getButton(GamepadKeys.Button.RIGHT_BUMPER), OutputSubsystem.OutputState.DROP)
            .transition(() -> primary.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.9, OutputSubsystem.OutputState.DROP_L)
            .build();

    double loopTime = 0.0;
    double x = 0.0;
    double y = 0.0;
    double turn = 0.0;
    final static double SLEW_RATE = 0.00001;

    @Override
    public void runOpMode() throws InterruptedException {
        hardware.init(hardwareMap);
        drive = new SampleMecanumDrive(hardware, hardwareMap);
        score = new ScoreCommand(hardware);
        intake = new IntakeSubsystem(hardware);
        primary = new GamepadEx(gamepad1);
        secondary = new GamepadEx(gamepad2);
        double power = 0.0;
        double position = Globals.LIFT_MIN;
        liftPid.setSetpoint(Globals.LIFT_MIN);
        liftPid.setOutputRange(0, 0.4);
        liftPid.reset();
        liftPid.enable();
        waitForStart();

        machine.start();
        outMachine.start();
        while (opModeIsActive() && !isStopRequested()) {
            double loop = System.nanoTime();
            double dt = loop - loopTime;
            machine.update();
            outMachine.update();
            if (secondary.getButton(GamepadKeys.Button.DPAD_UP)) {
                liftPid.setSetpoint(Globals.LIFT_MAX);
            } else if (secondary.getButton(GamepadKeys.Button.DPAD_DOWN)) {
                liftPid.setSetpoint(Globals.LIFT_MIN);
            }
//            if (liftPid.getSetpoint() == Globals.LIFT_MIN) {
//
//            }
            power = liftPid.performPID(score.getLeftDist());
            if (power < 0) {
                power = power/3;
            }
            score.update((ScoreCommand.ScoreState) machine.getState(), (OutputSubsystem.OutputState) outMachine.getState(), power);
            telemetry.addData("power", liftPid.performPID(score.getLeftDist()));
//            score.update(state, secondary.getLeftY());
            x += Math.max(-SLEW_RATE*dt, Math.min(primary.getLeftY() - x, SLEW_RATE*dt));
            y += Math.max(-SLEW_RATE*dt, Math.min(-primary.getLeftX() - y, SLEW_RATE*dt));
            turn += Math.max(-SLEW_RATE*dt, Math.min(-primary.getRightX() - turn, SLEW_RATE*dt));

            if (state == ScoreCommand.ScoreState.READY || hardware.intake.getPower() > 0.5 || primary.getButton(GamepadKeys.Button.LEFT_BUMPER)) {
                drive.setWeightedDrivePower(new Pose2d(x/2, y/2, turn/1.5));
            } else {
                drive.setWeightedDrivePower(new Pose2d(x, y, turn/1.5));
            }
            if (secondary.getButton(GamepadKeys.Button.LEFT_BUMPER)) {
                intake.update(IntakeSubsystem.IntakeState.INTAKE, IntakeSubsystem.OuterState.STACK_1);
            } else if (secondary.getButton(GamepadKeys.Button.RIGHT_BUMPER)) {
                intake.update(IntakeSubsystem.IntakeState.STOP, IntakeSubsystem.OuterState.STACK_1);
            } else if (secondary.getButton(GamepadKeys.Button.DPAD_LEFT)) {
                intake.update(IntakeSubsystem.IntakeState.SPIT, IntakeSubsystem.OuterState.STACK_1);
            }
            telemetry.addData("pivot pos", hardware.pivot.getPosition());
            telemetry.addData("left pos", hardware.fingerLeft.getPosition());
            telemetry.addData("right pos", hardware.fingerRight.getPosition());
            telemetry.addData("lift left dist", score.getLeftDist());
            telemetry.addData("lift right dist", score.getRightDist());
            telemetry.addData("hz ", 1000000000 / dt);
            loopTime = loop;
            telemetry.update();
        }
    }
}
