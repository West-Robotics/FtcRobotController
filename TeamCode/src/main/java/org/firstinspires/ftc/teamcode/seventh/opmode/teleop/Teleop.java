package org.firstinspires.ftc.teamcode.seventh.opmode.teleop;

import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
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

import org.firstinspires.ftc.teamcode.PIDController;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.seventh.robot.command.ScoreCommand;
import org.firstinspires.ftc.teamcode.seventh.robot.hardware.Globals;
import org.firstinspires.ftc.teamcode.seventh.robot.hardware.Hardware;
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.OuttakeSubsystem;

@TeleOp(name = "SussyOp")
public class Teleop extends LinearOpMode {

    Hardware hardware = Hardware.getInstance();
    SampleMecanumDrive dt;
    ScoreCommand score;
    IntakeSubsystem intake;
    PIDController liftPid = new PIDController(Globals.LIFT_P, Globals.LIFT_I, Globals.LIFT_D);

    GamepadEx primary;
    GamepadEx secondary;
    ScoreCommand.ScoreState state = ScoreCommand.ScoreState.TRANSFER;
    StateMachine machine = new StateMachineBuilder()
            .state(ScoreCommand.ScoreState.TRANSFER)
            .onEnter(() -> state = ScoreCommand.ScoreState.TRANSFER)
            .transition(() -> secondary.getButton(GamepadKeys.Button.A), ScoreCommand.ScoreState.LOCK)
            .state(ScoreCommand.ScoreState.LOCK)
            .onEnter(() -> state = ScoreCommand.ScoreState.LOCK)
            .transition(() -> secondary.getButton(GamepadKeys.Button.DPAD_UP), ScoreCommand.ScoreState.OUTTAKE_READY)
            .state(ScoreCommand.ScoreState.OUTTAKE_READY)
            .onEnter(() -> state = ScoreCommand.ScoreState.OUTTAKE_READY)
            .transition(() -> primary.getButton(GamepadKeys.Button.RIGHT_BUMPER), ScoreCommand.ScoreState.OUTTAKE_DROP)
            .transition(() -> primary.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.95, ScoreCommand.ScoreState.OUTTAKE_DROP_L)
            .transition(() -> primary.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.95, ScoreCommand.ScoreState.OUTTAKE_DROP_R)
            .state(ScoreCommand.ScoreState.OUTTAKE_DROP)
            .onEnter(() -> state = ScoreCommand.ScoreState.OUTTAKE_DROP)
            .transition(() -> secondary.getButton(GamepadKeys.Button.DPAD_DOWN), ScoreCommand.ScoreState.TRANSFER)
            .state(ScoreCommand.ScoreState.OUTTAKE_DROP_L)
            .onEnter(() -> state = ScoreCommand.ScoreState.OUTTAKE_DROP_L)
            .transition(() -> secondary.getButton(GamepadKeys.Button.DPAD_DOWN), ScoreCommand.ScoreState.TRANSFER)
            .state(ScoreCommand.ScoreState.OUTTAKE_DROP_R)
            .onEnter(() -> state = ScoreCommand.ScoreState.OUTTAKE_DROP_R)
            .transition(() -> secondary.getButton(GamepadKeys.Button.DPAD_DOWN), ScoreCommand.ScoreState.TRANSFER)
            .build();

    @Override
    public void runOpMode() throws InterruptedException {
        hardware.init(hardwareMap);
        dt = new SampleMecanumDrive(hardware, hardwareMap);
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
        while (opModeIsActive() && !isStopRequested()) {
            machine.update();
            if (secondary.getButton(GamepadKeys.Button.DPAD_UP)) {
                liftPid.setSetpoint(Globals.LIFT_MAX);
            } else if (secondary.getButton(GamepadKeys.Button.DPAD_DOWN)) {
                liftPid.setSetpoint(Globals.LIFT_MIN);
            }
            power = liftPid.performPID(score.getLeftDist());
            if (power < 0) {
                power = power/4;
            }
            score.update(state, power);
            telemetry.addData("power", liftPid.performPID(score.getLeftDist()));
//            score.update(state, secondary.getLeftY());
            if (state == ScoreCommand.ScoreState.OUTTAKE_READY || hardware.intake.getPower() > 0.5 || primary.getButton(GamepadKeys.Button.LEFT_BUMPER)) {
                dt.setDrivePower(new Pose2d(primary.getLeftY()/2, -primary.getLeftX()/2, -primary.getRightX()/2));
            } else {
                dt.setDrivePower(new Pose2d(primary.getLeftY(), -primary.getLeftX(), -primary.getRightX()));
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
            telemetry.update();
        }
    }
}
