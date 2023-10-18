package org.firstinspires.ftc.teamcode.seventh.opmode.teleop

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.sfdev.assembly.state.StateMachineBuilder

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.seventh.robot.command.CycleCommand
import org.firstinspires.ftc.teamcode.seventh.robot.command.CycleCommand.CycleState
import org.firstinspires.ftc.teamcode.seventh.robot.hardware.Hardware
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.IntakeSubsystem
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.LiftSubsystem
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.OutputSubsystem
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.OutputSubsystem.OutputState
import kotlin.math.abs
import kotlin.math.sign
import kotlin.time.DurationUnit
import kotlin.time.TimeSource

@TeleOp(name = "SussyOp")
public class Teleop : LinearOpMode() {

    // run blocking?
    override fun runOpMode() {
        // IDEAS:
        // automatically flip driving direction
        // lock robot to face exactly backdrop
        // mecanum feedforward

        val hardware: Hardware = Hardware.getInstance()
        hardware.init(hardwareMap)
        val drive: SampleMecanumDrive = SampleMecanumDrive(hardware, hardwareMap)
        val intake: IntakeSubsystem = IntakeSubsystem(hardware)
        val lift: LiftSubsystem = LiftSubsystem(hardware)
        val out: OutputSubsystem = OutputSubsystem(hardware)
        val cycle: CycleCommand = CycleCommand(intake, lift, out)
        val primary: GamepadEx = GamepadEx(gamepad1)
        val secondary: GamepadEx = GamepadEx(gamepad2)
        val cycleState = CycleCommand.CycleState.INTAKE
        val cycleMachine = StateMachineBuilder()
            .state(CycleCommand.CycleState.LOCK)
            .transition({ secondary.wasJustPressed(GamepadKeys.Button.A) }, CycleCommand.CycleState.INTAKE)
            .transition({ secondary.wasJustPressed(GamepadKeys.Button.DPAD_UP) }, CycleCommand.CycleState.READY)
            .transition({ secondary.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER) }, CycleCommand.CycleState.SPIT)
            .state(CycleCommand.CycleState.INTAKE)
            .transition({ secondary.wasJustPressed(GamepadKeys.Button.B) }, CycleCommand.CycleState.LOCK)
            .transition({ secondary.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER) }, CycleCommand.CycleState.SPIT)
            .state(CycleCommand.CycleState.READY)
            .transition({ secondary.wasJustPressed(GamepadKeys.Button.DPAD_DOWN) }, CycleCommand.CycleState.LOCK)
            .state(CycleCommand.CycleState.SPIT)
            .transition({ secondary.wasJustPressed(GamepadKeys.Button.A) }, CycleCommand.CycleState.INTAKE)
            .transition({ secondary.wasJustPressed(GamepadKeys.Button.B) }, CycleCommand.CycleState.LOCK)
            .build()
        val outMachine = StateMachineBuilder()
            .state(OutputSubsystem.OutputState.LOCK)
            .transition({ secondary.wasJustPressed(GamepadKeys.Button.A) }, OutputSubsystem.OutputState.INTAKE)
            .transition({ secondary.wasJustPressed(GamepadKeys.Button.DPAD_UP) }, OutputSubsystem.OutputState.READY)
            // this is actually just spit
            .transition({ secondary.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER) }, CycleCommand.CycleState.INTAKE)
            .state(OutputSubsystem.OutputState.INTAKE)
            .transition({ secondary.wasJustPressed(GamepadKeys.Button.B) }, OutputSubsystem.OutputState.LOCK)
            .state(OutputSubsystem.OutputState.READY)
            .transition({ secondary.wasJustPressed(GamepadKeys.Button.DPAD_DOWN) }, OutputSubsystem.OutputState.LOCK)
            .transition({ primary.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER) }, OutputSubsystem.OutputState.DROP)
            .transition({ primary.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.9 }, OutputSubsystem.OutputState.DROP_L)
            .transition({ primary.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.9 }, OutputSubsystem.OutputState.DROP_R)
            .state(OutputSubsystem.OutputState.DROP)
            .transition({ secondary.wasJustPressed(GamepadKeys.Button.DPAD_DOWN) }, OutputSubsystem.OutputState.LOCK)
            .transition({ primary.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER) }, OutputSubsystem.OutputState.READY)
            .transition({ primary.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.9 }, OutputSubsystem.OutputState.DROP_L)
            .transition({ primary.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.9 }, OutputSubsystem.OutputState.DROP_R)
            .state(OutputSubsystem.OutputState.DROP_L)
            .transition({ secondary.wasJustPressed(GamepadKeys.Button.DPAD_DOWN) }, OutputSubsystem.OutputState.LOCK)
            .transition({ primary.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER) }, OutputSubsystem.OutputState.DROP)
            .transition({ primary.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.9 }, OutputSubsystem.OutputState.DROP_R)
            .state(OutputSubsystem.OutputState.DROP_R)
            .transition({ secondary.wasJustPressed(GamepadKeys.Button.DPAD_DOWN) }, OutputSubsystem.OutputState.LOCK)
            .transition({ primary.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER) }, OutputSubsystem.OutputState.DROP)
            .transition({ primary.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.9 }, OutputSubsystem.OutputState.DROP_L)
            .build()

        var x = 0.0
        var y = 0.0
        var turn = 0.0
        // change per millisecond
        val SLEW_RATE = 4.0*1e-3
        val timeSource = TimeSource.Monotonic
        var loopTime: TimeSource.Monotonic.ValueTimeMark = timeSource.markNow()

        cycleMachine.start()
        outMachine.start()
        hardware.read(intake, lift, out)
        cycle.update(cycleMachine.state as CycleState, outMachine.state as OutputState)
        hardware.write(intake, lift, out)
        waitForStart()

        while (opModeIsActive() && !isStopRequested) {
            val loop = timeSource.markNow()
            val dt = (loop - loopTime).toDouble(DurationUnit.MILLISECONDS)
            loopTime = loop
            // remember to convert to seconds (multiply by 10^-3) for slew rate lmao

            primary.readButtons();
            secondary.readButtons();

            // update state machines
            cycleMachine.update();
            outMachine.update();

            // update all subsystems
            hardware.read(intake, lift, out);
            cycle.update(cycleMachine.state as CycleState, outMachine.state as OutputState)
            hardware.write(intake, lift, out);

            // only change dt powers by at max the slew rate
            x += (primary.leftY - x).let {if (abs(it) < SLEW_RATE*dt) it else sign(it)*SLEW_RATE*dt }
            y += (-primary.leftY - y).let {if (abs(it) < SLEW_RATE*dt) it else sign(it)*SLEW_RATE*dt }
            turn += (-primary.rightX - turn).let {if (abs(it) < SLEW_RATE*dt) it else sign(it)*SLEW_RATE*dt }

            val multiplier = when {
                cycleMachine.state == CycleState.READY              -> 0.5
                intake.state == IntakeSubsystem.IntakeState.INTAKE  -> 0.5
                primary.isDown(GamepadKeys.Button.LEFT_BUMPER)      -> 0.5
                else                                                -> 1.0
            }
            drive.setWeightedDrivePower(Pose2d(x*multiplier, y*multiplier, turn/1.5*multiplier))
            telemetry.addData("pivot pos", hardware.pivot.position);
            telemetry.addData("left pos", hardware.fingerLeft.position);
            telemetry.addData("right pos", hardware.fingerRight.position)
            telemetry.addData("lift dist", lift.distance)
            telemetry.addData("hz ", 1000 / dt)
            telemetry.update()
        }
    }
}
