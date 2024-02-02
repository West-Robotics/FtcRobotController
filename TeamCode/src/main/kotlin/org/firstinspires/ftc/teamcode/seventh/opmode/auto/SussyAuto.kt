package org.firstinspires.ftc.teamcode.seventh.opmode.auto

import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.configuration.LynxConstants
import com.scrapmetal.quackerama.control.Pose2d
import com.scrapmetal.quackerama.control.Rotation2d
import com.scrapmetal.quackerama.control.gvf.GG
import com.scrapmetal.quackerama.control.path.path
import com.sfdev.assembly.state.StateMachineBuilder
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.seventh.robot.command.CycleCommand
import org.firstinspires.ftc.teamcode.seventh.robot.command.RRTrajectories
import org.firstinspires.ftc.teamcode.seventh.robot.hardware.Globals
import org.firstinspires.ftc.teamcode.seventh.robot.hardware.Robot
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.DriveSubsystem
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.HangSubsystem
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.IntakeSubsystem
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.LiftSubsystem
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.OutputSubsystem
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.RobotState
import org.firstinspires.ftc.teamcode.seventh.robot.vision.GetPropPositionPipeline
import org.firstinspires.ftc.teamcode.seventh.robot.vision.Vision
import java.lang.Math.toDegrees
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
⬛🟩🟩🟩⬛🟫🟫🟫⬛🟧🟧🟧⬛🟨🟨🟨⬛🟥🟥🟥⬛
🟩🟩🟦🟦🟫🟫🟦🟦🟧🟧🟦🟦🟨🟨🟦🟦🟥🟥🟦🟦🟦
🟩🟩🟩🟩🟫🟫🟫🟫🟧🟧🟧🟧🟨🟨🟨🟨🟥🟥🟥🟥🟦
⬛🟩⬛🟩⬛🟫⬛🟫⬛🟧⬛🟧⬛🟨⬛🟨⬛🟥⬛🟥⬛
⬛⬛⬛⬛⬛⬛⬛⬛⬛⬛⬛⬛⬛⬛⬛⬛⬛⬛⬛⬛⬛
"""
)
class SussyAuto : LinearOpMode() {
    override fun runOpMode() {
        Globals.AUTO = true
        Globals.side = Globals.Side.BLUE
        Globals.start = Globals.Start.CLOSE
        Globals.lane = Globals.Lane.LANE_1
        Robot.hardwareMap = hardwareMap
        val allHubs = hardwareMap.getAll(LynxModule::class.java)
        var CONTROL_HUB: LynxModule = allHubs[0]
        for (hub in allHubs) {
            hub.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL
            if (hub.isParent() && LynxConstants.isEmbeddedSerialNumber(hub.serialNumber)) {
                CONTROL_HUB = hub
            }
        }
        val drive = DriveSubsystem(hardwareMap)
        val intake = IntakeSubsystem(hardwareMap)
        val lift = LiftSubsystem(hardwareMap)
        val output = OutputSubsystem(hardwareMap)
        val vision = Vision(hardwareMap)
        val gamepad = GamepadEx(gamepad1)

        val timeSource = TimeSource.Monotonic
        var loopTime: TimeSource.Monotonic.ValueTimeMark = timeSource.markNow()

        val cycle = CycleCommand(intake, lift, output)

        var state = RobotState.LOCK
        var height = 0
        intake.setHeight(5)

        Robot.read(intake, output)
        cycle.update(state, height, 0.0)
        Robot.write(intake, output)

        vision.enableProp()
        val paths = AutoPaths(
                Globals.Side.RED,
                Globals.Start.FAR,
                Globals.Lane.LANE_2,
                Globals.YellowSide.LEFT,
                Globals.Stack.CLOSE,
                Globals.Park.INNER,
                GetPropPositionPipeline.PropPosition.MIDDLE
        )
        drive.setPoseEstimate(paths.startPose)
        val gg = GG(
                kN = 0.6,
                kA = 0.0004,
                paths.purple,
                paths.plusOne,
                paths.yellow,
                paths.intake,
                paths.score,
                paths.intake,
                paths.score,
        )
        // waitForStart()

        drive.startIMUThread(this)
        while (opModeInInit()) {
            CONTROL_HUB.clearBulkCache()
            Robot.read(drive)
            telemetry.addData("x", drive.getPoseEstimate().position.u)
            telemetry.addData("y", drive.getPoseEstimate().position.v)
            telemetry.addData("heading", drive.getPoseEstimate().heading.polarAngle)
            telemetry.addData("des x", paths.purple.paths[0].endPose.position.u)
            telemetry.addData("des y", paths.purple.paths[0].endPose.position.v)
            telemetry.addData("des heading", paths.purple.paths[0].endPose.heading.polarAngle)
            // telemetry.addData("purple start x", paths.purple.paths[0].startPose.position.u)
            // telemetry.addData("purple start y", paths.purple.paths[0].startPose.position.v)
            // telemetry.addData("purple start heading", paths.purple.paths[0].startPose.heading.polarAngle)
            // telemetry.addData("purple end x", paths.purple.paths[0].endPose.position.u)
            // telemetry.addData("purple end y", paths.purple.paths[0].endPose.position.v)
            // telemetry.addData("plusOne start x", paths.plusOne.paths[0].startPose.position.u)
            // telemetry.addData("plusOne start y", paths.plusOne.paths[0].startPose.position.v)
            // telemetry.addData("plusOne start heading", paths.plusOne.paths[0].startPose.heading.polarAngle)
            // telemetry.addData("plusOne end x", paths.plusOne.paths[0].endPose.position.u)
            // telemetry.addData("plusOne end y", paths.plusOne.paths[0].endPose.position.v)
            telemetry.update()
        }

        // val autoMachine = StateMachineBuilder()
        //         .state(AutoStates.GRAB)
        //         .onEnter { drive.followTrajectorySequenceAsync(traj.collect)}
        //         .transition { !drive.isBusy}
        //         .onExit { intake.setHeight(1) }
        //         .state(AutoStates.COLLECT)
        //         .transitionTimed(0.5)
        //         .onExit { drive.followTrajectorySequenceAsync(traj.dropOff) }
        //         .state(AutoStates.DROP_OFF)
        //         .transition { !drive.isBusy }
        //         .onExit { intake.setHeight(5); drive.followTrajectorySequenceAsync(traj.score)  }
        //         .state(AutoStates.TO_BACKDROP)
        //         .transition { !drive.isBusy }
        //         .onExit { height = 1; state = RobotState.BACKDROP }
        //         .state(AutoStates.RAISE)
        //         .transitionTimed(1.5)
        //         .onExit { state = RobotState.EXTEND }
        //         .state(AutoStates.EXTEND)
        //         .transitionTimed(1.0)
        //         .onExit { height = -1 }
        //         .state(AutoStates.SAFE)
        //         .transitionTimed(1.0)
        //         .onExit { state = RobotState.SCORE }
        //         .state(AutoStates.SCORE)
        //         .transitionTimed(1.0)
        //         .onExit { height = 1 }
        //         .state(AutoStates.UP_AGAIN)
        //         .transitionTimed(1.0)
        //         .onExit { drive.followTrajectorySequenceAsync(traj.park) }
        //         .state(AutoStates.PARK)
        //         .transition { !drive.isBusy }
        //         .onExit { state = RobotState.BACKDROP }
        //         .state(AutoStates.RETRACT)
        //         .transitionTimed(1.0)
        //         .onExit { height = 0; state = RobotState.LOCK }
        //         .state(AutoStates.LOWER)
        //         .build()
        // autoMachine.start()

        // visionPortal.setProcessorEnabled(propProcessor, false)
        // visionPortal.close()
        while (opModeIsActive()) {
            CONTROL_HUB.clearBulkCache()
            val loop = timeSource.markNow()
            val dt = (loop - loopTime).toDouble(DurationUnit.MILLISECONDS)
            loopTime = loop

            // update all subsystems
            Robot.read(drive, intake, lift, output);
            // autoMachine.update()
            cycle.update(state, height, 0.0)
            drive.update(
                    input = Pose2d(),
                    // input = gg.update(drive.getPoseEstimate().position, drive.getVelocity().position),
                    correcting = false,
                    fieldOriented = true,
                    dt = Robot.dt,
                    pid = true,
            )
            val input = gg.update(drive.getPoseEstimate().position, drive.getVelocity().position)
            Robot.write(drive, intake, lift, output);

            if (gamepad.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                gg.currentIndex++
            } else if (gamepad.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
                gg.currentIndex--
            }
            // telemetry.addData("prop pos", traj.prop)
            // telemetry.addData("lift dist", lift.distance)
            // telemetry.addData("auto state", autoMachine.state as AutoStates)
            telemetry.addData("hz", 1000 / dt)
            telemetry.addData("current index", gg.currentIndex)
            telemetry.addData("x", drive.getPoseEstimate().position.u)
            telemetry.addData("y", drive.getPoseEstimate().position.v)
            // telemetry.addData("heading", drive.getPoseEstimate().heading.polarAngle)
            // telemetry.addData("desired mag", input.position.mag)
            // telemetry.addData("desired direction", toDegrees(input.position.polarAngle))
            // telemetry.addData("desired heading", input.heading.polarAngle)
            telemetry.update()
        }
    }
}