package org.firstinspires.ftc.teamcode.seventh.opmode.teleop

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.util.ElapsedTime
import com.scrapmetal.quackerama.control.Pose2d
import com.scrapmetal.quackerama.control.Rotation2d
import com.scrapmetal.quackerama.control.Vector2d
import com.sfdev.assembly.state.StateMachineBuilder

import org.firstinspires.ftc.teamcode.seventh.robot.command.CycleCommand
import org.firstinspires.ftc.teamcode.seventh.robot.hardware.Globals
import org.firstinspires.ftc.teamcode.seventh.robot.hardware.Robot
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.DriveSubsystem
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.HangSubsystem
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.IntakeSubsystem
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.LiftSubsystem
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.OutputSubsystem
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.RobotState
import kotlin.time.DurationUnit
import kotlin.time.TimeSource

@TeleOp(name =
    "🟦🟦🟦🟦🟦🟦🟦🟦🟦⬛⬛⬛⬛⬛🟦🟦🟦🟦🟦🟦🟦\n" +
    "🟦🟦🟦🟦🟦🟦🟦🟦⬛⬛🟥🟥🟥⬛⬛⬛🟦🟦🟦🟦🟦\n" +
    "🟦🟦🟦🟦🟦🟦🟦⬛⬛🟥🟥🟥🟥🟥🟥🟥⬛🟦🟦🟦🟦\n" +
    "🟦🟦🟦🟦🟦🟦🟦⬛🟥🟥🟥🟥🟥🟥🟥🟥⬛🟦🟦🟦🟦\n" +
    "🟦🟦🟦🟦🟦🟦🟦⬛🟥🟥🟥🟥🟥🟥🟥🟥🟥⬛🟦🟦🟦\n" +
    "🟦🟦🟦🟦🟦🟦🟦⬛🟥🟥🟥⬜⬜⬜⬜⬜⬜⬛⬛🟦🟦\n" +
    "🟦🟦🟦🟦⬛⬛⬛🟥🟥🟥⬜⬜⬜⬜⬜⬜⬜⬜⬛🟦🟦\n" +
    "🟦🟦🟦⬛🟥⬛⬛🟥🟥🟥⬜⬜⬜⬜⬜⬜⬜⬜⬛🟦🟦\n" +
    "🟦🟦🟦⬛🟥⬛⬛🟥🟥🟥⬛⬜⬜⬜⬜⬜⬜⬜⬛🟦🟦\n" +
    "🟦🟦⬛⬛🟥⬛⬛🟥🟥🟥🟥⬛⬛⬛⬛⬛⬛⬛🟦🟦🟦\n" +
    "🟦🟦⬛⬛🟥⬛⬛🟥🟥🟥🟥🟥🟥🟥🟥🟥🟥⬛🟦🟦🟦\n" +
    "🟦🟦⬛⬛🟥⬛⬛🟥🟥🟥🟥🟥🟥🟥🟥🟥🟥⬛🟦🟦🟦\n" +
    "🟦🟦⬛⬛🟥⬛⬛🟥🟥🟥🟥🟥🟥🟥🟥🟥🟥⬛🟦🟦🟦\n" +
    "🟦🟦⬛⬛🟥⬛⬛🟥🟥🟥🟥🟥🟥🟥🟥🟥🟥⬛🟦🟦🟦\n" +
    "🟦🟦⬛⬛🟥⬛⬛🟥🟥🟥🟥🟥🟥🟥🟥🟥🟥⬛🟦🟦🟦\n" +
    "🟦🟦⬛⬛🟥⬛⬛🟥🟥🟥🟥🟥🟥🟥🟥🟥🟥⬛🟦🟦🟦\n" +
    "🟦🟦⬛⬛🟥⬛⬛🟥🟥🟥🟥🟥🟥🟥🟥🟥🟥⬛🟦🟦🟦\n" +
    "🟦🟦🟦⬛⬛⬛⬛🟥🟥🟥🟥⬛⬛🟥🟥🟥🟥⬛🟦🟦🟦\n" +
    "🟦🟦🟦🟦🟦⬛⬛🟥🟥🟥🟥⬛⬛🟥🟥🟥🟥⬛🟦🟦🟦\n" +
    "🟦🟦🟦🟦🟦⬛⬛⬛🟥🟥⬛⬛⬛🟥🟥🟥⬛🟦🟦🟦🟦\n" +
    "🟦🟦🟦🟦🟦🟦🟦⬛⬛⬛⬛🟦⬛⬛⬛⬛⬛🟦🟦🟦🟦")
// TODO: add heading pdf
class SussyTele : LinearOpMode() {
    // run blocking?
    override fun runOpMode() {
        // IDEAS:
        // automatically flip driving direction
        // lock robot to face exactly backdrop
        // mecanum feedforward
        // delete clutter opmodes
        Globals.AUTO = false
        Robot.hardwareMap = hardwareMap
        val allHubs = hardwareMap.getAll(LynxModule::class.java)
        for (hub in allHubs) {
            hub.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL
        }
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
        val drive = DriveSubsystem(hardwareMap)
        val intake = IntakeSubsystem(hardwareMap)
        val lift = LiftSubsystem(hardwareMap)
        val output = OutputSubsystem(hardwareMap)
        val hang = HangSubsystem(hardwareMap)

        val primary = GamepadEx(gamepad1)
        val secondary = GamepadEx(gamepad2)

        val timeSource = TimeSource.Monotonic
        var loopTime: TimeSource.Monotonic.ValueTimeMark = timeSource.markNow()

        // is this bad? maybe switch to a singleton
        var height = 0
        var fillEdge = false
        var fillTimer = ElapsedTime()
        val cycle = CycleCommand(intake, lift, output)
        var intendedOutputState = RobotState.SCORE
        val cycleMachine = StateMachineBuilder()
                .state(RobotState.LOCK)
                .transition({ secondary.wasJustPressed(GamepadKeys.Button.A) }, RobotState.INTAKE)
                .transition({ secondary.wasJustPressed(GamepadKeys.Button.DPAD_UP) }, RobotState.BACKDROP)
                .transition({ secondary.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER) }, RobotState.SPIT)
                .state(RobotState.INTAKE)
                .transition({ secondary.wasJustPressed(GamepadKeys.Button.B) }, RobotState.LOCK)
                .transition({ fillEdge && fillTimer.seconds() > 0.25 }, RobotState.LOCK)
                .transition({ secondary.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER) }, RobotState.SPIT)
                .state(RobotState.SPIT)
                .transition({ secondary.wasJustPressed(GamepadKeys.Button.B) }, RobotState.LOCK)
                .transition({ secondary.wasJustPressed(GamepadKeys.Button.A) }, RobotState.INTAKE)
                .state(RobotState.BACKDROP)
                .transition({ secondary.wasJustPressed(GamepadKeys.Button.DPAD_DOWN) }, RobotState.LOCK)
                .transition({ primary.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER) }, RobotState.EXTEND)
                .transition({ primary.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.9 }, RobotState.EXTEND)
                .transition({ primary.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.9 }, RobotState.EXTEND)
                .onExit { intendedOutputState = when {
                    primary.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER) -> RobotState.SCORE
                    primary.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.9 -> RobotState.SCORE_L
                    primary.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.9 -> RobotState.SCORE_R
                    else -> RobotState.LOCK
                }}
                .state(RobotState.EXTEND)
                // TODO: make this transition based off reached arm
                .transition({ primary.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER) }, intendedOutputState)
                .transition({ secondary.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER) }, RobotState.BACKDROP)
                .state(RobotState.SCORE_L)
                .transition({ secondary.wasJustPressed(GamepadKeys.Button.DPAD_UP) }, RobotState.EXTEND)
                .transition({ primary.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER) }, RobotState.SCORE)
                .transition({ primary.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.9 }, RobotState.SCORE_R)
                .state(RobotState.SCORE)
                .transition({ secondary.wasJustPressed(GamepadKeys.Button.DPAD_UP) }, RobotState.EXTEND)
                .transition({ primary.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.9 }, RobotState.SCORE_L)
                .transition({ primary.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.9 }, RobotState.SCORE_R)
                .state(RobotState.SCORE_R)
                .transition({ secondary.wasJustPressed(GamepadKeys.Button.DPAD_UP) }, RobotState.EXTEND)
                .transition({ primary.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.9 }, RobotState.SCORE_L)
                .transition({ primary.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER) }, RobotState.SCORE)
                .build()

        cycleMachine.start()
        Robot.read(intake, lift, output, hang)
        cycle.update(cycleMachine.state as RobotState, height, 0.0)
        Robot.write(intake, lift, output, hang)
        waitForStart()

        val elapsedTime = ElapsedTime()
        while (opModeIsActive() && !isStopRequested) {
            for (hub in allHubs) {
                hub.clearBulkCache()
            }
            val loop = timeSource.markNow()
            val dt = (loop - loopTime).toDouble(DurationUnit.MILLISECONDS)
            loopTime = loop
            Robot.read(drive, intake, lift, output, hang)

            primary.readButtons()
            secondary.readButtons()

            if (secondary.wasJustPressed(GamepadKeys.Button.DPAD_UP) && height < 5) {
                height++
            } else if (secondary.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
                height = 0
            } else if (secondary.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
                height = 6
            }

            // if (secondary.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.9) {
            //     hang.update(HangSubsystem.HangState.RAISE)
            // } else if (secondary.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.9) {
            //     hang.update(HangSubsystem.HangState.LOWER)
            // } else {
            //     hang.update(HangSubsystem.HangState.STOP)
            // }

            if (secondary.wasJustPressed(GamepadKeys.Button.Y)) {
                intake.raise()
            } else if (secondary.wasJustPressed(GamepadKeys.Button.X)) {
                intake.lower()
            }

            // if (secondary.wasJustPressed(GamepadKeys.Button.START)) {
            //     Globals.PIVOT_OUTTAKE -= 0.01
            // } else if (secondary.wasJustPressed(GamepadKeys.Button.BACK)) {
            //     Globals.PIVOT_OUTTAKE += 0.01
            // }

            // update all subsystems
            cycle.update(
                    cycleMachine.state as RobotState,
                    height,
                    secondary.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)*10.5
            )
            cycleMachine.update()
            drive.update(
                    Pose2d(Vector2d(primary.leftY, -primary.leftX), Rotation2d(-primary.rightX)),
                    correcting = false,
                    fieldOriented = false,
                    dt = Robot.dt,
                    pid = false
            )
            if (output.leftFilled && output.rightFilled && !fillEdge) {
                fillEdge = true
                fillTimer = ElapsedTime()
                primary.gamepad.rumbleBlips(1)
                secondary.gamepad.rumbleBlips(1)
            } else if (!output.leftFilled && !output.rightFilled && fillEdge) {
                fillEdge = false
            }

            telemetry.addData("cycle state", cycleMachine.state as RobotState)
            telemetry.addData("hz ", 1000 / dt)
            telemetry.addData("time", elapsedTime.seconds())
            telemetry.addData("lift commanded", lift.state.commandedExtension)
            telemetry.addData("lift dist", lift.state.extension)
            telemetry.addData("lift power", lift.state.power)
            telemetry.addData("lift current", lift.state.current)
            telemetry.addData("grounded", lift.state.grounded)
            telemetry.addData("arm angle", output.outState.arm)
            telemetry.update()
            Robot.write(intake, lift, output, hang)
        }
    }
}