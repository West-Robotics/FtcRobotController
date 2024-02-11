package org.firstinspires.ftc.teamcode.seventh.opmode.auto

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.configuration.LynxConstants
import com.qualcomm.robotcore.util.ElapsedTime
import com.scrapmetal.quackerama.control.gvf.GG
import com.sfdev.assembly.state.StateMachineBuilder
import org.firstinspires.ftc.teamcode.seventh.robot.command.CycleCommand
import org.firstinspires.ftc.teamcode.seventh.robot.hardware.Globals
import org.firstinspires.ftc.teamcode.seventh.robot.hardware.Robot
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.DriveSubsystem
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.IntakeSubsystem
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.LiftSubsystem
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.OutputSubsystem
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.RobotState
import org.firstinspires.ftc.teamcode.seventh.robot.vision.GetPropPositionPipeline
import org.firstinspires.ftc.teamcode.seventh.robot.vision.Vision
import kotlin.time.DurationUnit
import kotlin.time.TimeSource

@Autonomous(name =
"""
⬛🟥🟥🟥⬛🟩🟩🟩⬛🟫🟫🟫⬛🟧🟧🟧⬛🟨🟨🟨⬛
🟥🟥🟦🟦🟩🟩🟦🟦🟫🟫🟦🟦🟧🟧🟦🟦🟨🟨🟦🟦🟦
🟥🟥🟥🟥🟩🟩🟩🟩🟫🟫🟫🟫🟧🟧🟧🟧🟨🟨🟨🟨🟦
⬛🟥⬛🟥⬛🟩⬛🟩⬛🟫⬛🟫⬛🟧⬛🟧⬛🟨⬛🟨⬛
⬛🟨🟨🟨⬛🟥🟥🟥⬛🟩🟩🟩⬛🟫🟫🟫⬛🟧🟧🟧⬛
🟨🟨🟦🟦🟥🟥🟦🟦🟩🟩🟦🟦🟫🟫🟦🟦🟧🟧🟦🟦🟦
🟨🟨🟨🟨🟥🟥🟥🟥🟩🟩🟩🟩🟫🟫🟫🟫🟧🟧🟧🟧🟦
⬛🟨⬛🟨⬛🟥⬛🟥⬛🟩⬛🟩⬛🟫⬛🟫⬛🟧⬛🟧⬛
⬛🟧🟧🟧⬛🟨🟨🟨⬛🟥🟥🟥⬛🟩🟩🟩⬛🟫🟫🟫⬛
🟧🟧🟦🟦🟨🟨🟦🟦🟥🟥🟦🟦🟩🟩🟦🟦🟫🟫🟦🟦🟦
🟧🟧🟧🟧🟨🟨🟨🟨🟥🟥🟥🟥🟩🟩🟩🟩🟫🟫🟫🟫🟦
⬛🟧⬛🟧⬛🟨⬛🟨⬛🟥⬛🟥⬛🟩⬛🟩⬛🟫⬛🟫⬛
⬛🟫🟫🟫⬛🟧🟧🟧⬛🟨🟨🟨⬛🟥🟥🟥⬛🟩🟩🟩⬛
🟫🟫🟦🟦🟧🟧🟦🟦🟨🟨🟦🟦🟥🟥🟦🟦🟩🟩🟦🟦🟦
🟫🟫🟫🟫🟧🟧🟧🟧🟨🟨🟨🟨🟥🟥🟥🟥🟩🟩🟩🟩🟦
⬛🟫⬛🟫⬛🟧⬛🟧⬛🟨⬛🟨⬛🟥⬛🟥⬛🟩⬛🟩⬛
⬛⬛⬛⬛⬛⬛⬛⬛⬛⬛⬛⬛⬛⬛⬛⬛⬛⬛⬛⬛⬛
"""
)
class SussyAuto : LinearOpMode() {
    enum class AutoStates {
        TO_PURPLE,
        DROP_PURPLE,
        TO_YELLOW,
        BACKDROP,
        EXTEND,
        SCORE,
        ADIOS,
        RETRACT,
        LOCK,
        TO_STACKS,
        INTAKE_STACKS,
        TO_BACKDROP,
        PARK,
    }
    override fun runOpMode() {
        Globals.AUTO = true
        Globals.side = Globals.Side.RED
        Robot.hardwareMap = hardwareMap
        val allHubs = hardwareMap.getAll(LynxModule::class.java)
        var CONTROL_HUB: LynxModule = allHubs[0]
        for (hub in allHubs) {
            hub.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL
            if (hub.isParent && LynxConstants.isEmbeddedSerialNumber(hub.serialNumber)) {
                CONTROL_HUB = hub
            }
        }
        val drive = DriveSubsystem(hardwareMap)
        val intake = IntakeSubsystem(hardwareMap)
        val lift = LiftSubsystem(hardwareMap)
        val output = OutputSubsystem(hardwareMap)
        val vision = Vision(hardwareMap)
        val gamepad = GamepadEx(gamepad1)
        val cycle = CycleCommand(intake, lift, output)

        var state = RobotState.LOCK
        var height = 0
        var stackCount = 5
        val timer = ElapsedTime()

        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
        lateinit var gg: GG
        val autoMachine = StateMachineBuilder()
                .state(AutoStates.TO_PURPLE)
                    .transition { gg.onTarget(drive.getPoseEstimate().position) }
                    .transitionTimed(3.0)
                .state(AutoStates.DROP_PURPLE)
                    .onEnter { intake.setHeight(5) }
                    .transitionTimed(0.5)
                    .onExit { gg.currentIndex++ }
                .state(AutoStates.TO_YELLOW)
                    .transition( { gg.onTarget(drive.getPoseEstimate().position) }, AutoStates.BACKDROP)
                    .transitionTimed(3.0)
                .state(AutoStates.BACKDROP)
                    .onEnter { height = if (stackCount == 5) 1 else 2; state = RobotState.BACKDROP }
                    .transitionTimed(1.0)
                .state(AutoStates.EXTEND)
                    .onEnter { state = RobotState.EXTEND }
                    .transitionTimed(0.5)
                .state(AutoStates.SCORE)
                    .onEnter { state = RobotState.SCORE }
                    .transitionTimed(0.5)
                .state(AutoStates.RETRACT)
                    .onEnter { state = RobotState.BACKDROP }
                    .transitionTimed(0.5)
                .state(AutoStates.LOCK)
                    .onEnter { height = 0; state = RobotState.LOCK }
                    .transition({ stackCount == 5 }, AutoStates.TO_STACKS)
                    .transition({ stackCount == 3 }, AutoStates.PARK, { stackCount = 1 } )
                .state(AutoStates.TO_STACKS)
                    .onEnter { gg.currentIndex++; intake.setHeight(5) }
                // .onEnter { gg.currentIndex++; intake.setHeight(stackCount); state = RobotState.INTAKE }
                    .transitionTimed(3.0)
                .state(AutoStates.INTAKE_STACKS)
                    .onEnter { timer.reset() }
                    .transitionTimed(2.0)
                    .onExit { gg.currentIndex++ }
                .state(AutoStates.TO_BACKDROP)
                    .transition( { gg.onTarget(drive.getPoseEstimate().position) }, AutoStates.BACKDROP)
                    .transitionTimed(5.0)
                .state(AutoStates.PARK)
                    .onEnter { gg.currentIndex++ }
                .build()
        intake.setHeight(1)
        Robot.read(intake, output)
        cycle.update(state, height, 0.0)
        Robot.write(intake, output)

        vision.enableProp()
        drive.startIMUThread(this)
        while (opModeInInit()) {
            telemetry.addData("prop", vision.getPropPosition())
            telemetry.update()
        }
        val paths = AutoPaths(
                Globals.Side.RED,
                Globals.Start.CLOSE,
                Globals.Lane.LANE_2,
                Globals.YellowSide.LEFT,
                Globals.Stack.CLOSE,
                Globals.Park.INNER,
                GetPropPositionPipeline.PropPosition.MIDDLE,
        )
        vision.disableProp()
        drive.setPoseEstimate(paths.initPose)
        gg = GG(
                kN = 0.4,
                kA = 0.0001, // was 0.0004
                paths.purple,
                paths.yellow,
                paths.intake,
                paths.score,
                paths.parkPath,
        )
        autoMachine.start()

        val timeSource = TimeSource.Monotonic
        while (opModeIsActive()) {
            val t1b = timeSource.markNow()
            CONTROL_HUB.clearBulkCache()
            Robot.dtUpdate()
            autoMachine.update()
            val t1e = timeSource.markNow()

            val t2b = timeSource.markNow()
            // update all subsystems
            Robot.read(intake, output, lift, drive)
            val t2e = timeSource.markNow()
            val input = gg.update(drive.getPoseEstimate().position, drive.getVelocity().position)
            drive.update(
                    input = input,
                    correcting = false,
                    fieldOriented = true,
                    dt = Robot.dt,
                    pid = true,
            )

            when {
                autoMachine.state as AutoStates == AutoStates.TO_STACKS &&
                drive.getPoseEstimate().position.u < -48.0
                    -> { state = RobotState.INTAKE }
                autoMachine.state as AutoStates == AutoStates.INTAKE_STACKS && timer.seconds() > 0.75
                    -> { intake.setHeight(4); stackCount -= 2}
                autoMachine.state as AutoStates == AutoStates.TO_BACKDROP && timer.seconds() > 0.5
                    -> { state = RobotState.PRELOCK; timer.reset() }
                autoMachine.state as AutoStates == AutoStates.TO_BACKDROP && timer.seconds() > 0.5
                    -> { state = RobotState.LOCK }
            }
            cycle.update(state, height, drive.wallDist)
            val t3b = timeSource.markNow()
            Robot.write(drive, lift, output, intake)
            val t3e = timeSource.markNow()

            telemetry.addData("hz", 1000 / Robot.dt)
            telemetry.addData("current index", gg.currentIndex)
            telemetry.addData("error", gg.error(drive.getPoseEstimate().position))
            telemetry.addData("onTarget", gg.onTarget(drive.getPoseEstimate().position))
            telemetry.addData("x", drive.getPoseEstimate().position.u)
            telemetry.addData("y", drive.getPoseEstimate().position.v)
            telemetry.addData("state", autoMachine.state as AutoStates)
            telemetry.addData("read", (t2e-t2b).toDouble(DurationUnit.MILLISECONDS))
            telemetry.addData("write", (t3e-t3b).toDouble(DurationUnit.MILLISECONDS))
            telemetry.addData("wall", drive.wallDist)
            telemetry.update()
        }
    }
}