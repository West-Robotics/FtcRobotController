package org.firstinspires.ftc.teamcode.seventh.opmode.auto

import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.sfdev.assembly.state.StateMachineBuilder
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.seventh.robot.command.CycleCommand
import org.firstinspires.ftc.teamcode.seventh.robot.command.RRTrajectories
import org.firstinspires.ftc.teamcode.seventh.robot.hardware.Globals
import org.firstinspires.ftc.teamcode.seventh.robot.hardware.Robot
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.IntakeSubsystem
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.LiftSubsystem
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.OutputSubsystem
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.PropPositionProcessor
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.RobotState
import org.firstinspires.ftc.vision.VisionPortal
import kotlin.time.DurationUnit
import kotlin.time.TimeSource

enum class AutoStates {
    DROP_OFF,
    TO_BACKDROP,
    RAISE,
    EXTEND,
    SCORE,
    RETRACT,
    LOWER,
    PARK,
}
@Autonomous(name =
    "RED ⬛🟥🟥🟥⬛🟩🟩🟩⬛🟫🟫🟫⬛🟧🟧🟧⬛🟨🟨🟨⬛\n" +
    "🟥🟥🟦🟦🟩🟩🟦🟦🟫🟫🟦🟦🟧🟧🟦🟦🟨🟨🟦🟦🟦\n" +
    "🟥🟥🟥🟥🟩🟩🟩🟩🟫🟫🟫🟫🟧🟧🟧🟧🟨🟨🟨🟨🟦\n" +
    "⬛🟥⬛🟥⬛🟩⬛🟩⬛🟫⬛🟫⬛🟧⬛🟧⬛🟨⬛🟨⬛\n" +
    "⬛🟨🟨🟨⬛🟥🟥🟥⬛🟩🟩🟩⬛🟫🟫🟫⬛🟧🟧🟧⬛\n" +
    "🟨🟨🟦🟦🟥🟥🟦🟦🟩🟩🟦🟦🟫🟫🟦🟦🟧🟧🟦🟦🟦\n" +
    "🟨🟨🟨🟨🟥🟥🟥🟥🟩🟩🟩🟩🟫🟫🟫🟫🟧🟧🟧🟧🟦\n" +
    "⬛🟨⬛🟨⬛🟥⬛🟥⬛🟩⬛🟩⬛🟫⬛🟫⬛🟧⬛🟧⬛\n" +
    "⬛🟧🟧🟧⬛🟨🟨🟨⬛🟥🟥🟥⬛🟩🟩🟩⬛🟫🟫🟫⬛\n" +
    "🟧🟧🟦🟦🟨🟨🟦🟦🟥🟥🟦🟦🟩🟩🟦🟦🟫🟫🟦🟦🟦\n" +
    "🟧🟧🟧🟧🟨🟨🟨🟨🟥🟥🟥🟥🟩🟩🟩🟩🟫🟫🟫🟫🟦\n" +
    "⬛🟧⬛🟧⬛🟨⬛🟨⬛🟥⬛🟥⬛🟩⬛🟩⬛🟫⬛🟫⬛\n" +
    "⬛🟫🟫🟫⬛🟧🟧🟧⬛🟨🟨🟨⬛🟥🟥🟥⬛🟩🟩🟩⬛\n" +
    "🟫🟫🟦🟦🟧🟧🟦🟦🟨🟨🟦🟦🟥🟥🟦🟦🟩🟩🟦🟦🟦\n" +
    "🟫🟫🟫🟫🟧🟧🟧🟧🟨🟨🟨🟨🟥🟥🟥🟥🟩🟩🟩🟩🟦\n" +
    "⬛🟫⬛🟫⬛🟧⬛🟧⬛🟨⬛🟨⬛🟥⬛🟥⬛🟩⬛🟩⬛\n" +
    "⬛🟩🟩🟩⬛🟫🟫🟫⬛🟧🟧🟧⬛🟨🟨🟨⬛🟥🟥🟥⬛\n" +
    "🟩🟩🟦🟦🟫🟫🟦🟦🟧🟧🟦🟦🟨🟨🟦🟦🟥🟥🟦🟦🟦\n" +
    "🟩🟩🟩🟩🟫🟫🟫🟫🟧🟧🟧🟧🟨🟨🟨🟨🟥🟥🟥🟥🟦\n" +
    "⬛🟩⬛🟩⬛🟫⬛🟫⬛🟧⬛🟧⬛🟨⬛🟨⬛🟥⬛🟥⬛\n" +
    "⬛⬛⬛⬛⬛⬛⬛⬛⬛⬛⬛⬛⬛⬛⬛⬛⬛⬛⬛⬛⬛")
class SussyAutoRed : LinearOpMode() {
    override fun runOpMode() {
        Globals.AUTO = true
        Globals.side = Globals.Side.RED
        Globals.start = Globals.Start.CLOSE
        Globals.lane = Globals.Lane.LANE_1
        Robot.hardwareMap = hardwareMap
        val allHubs = hardwareMap.getAll(LynxModule::class.java)
        for (hub in allHubs) {
            hub.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL
        }
        val drive = SampleMecanumDrive(hardwareMap)
        val intake = IntakeSubsystem(hardwareMap)
        val lift = LiftSubsystem(hardwareMap)
        val out = OutputSubsystem(hardwareMap)

        val timeSource = TimeSource.Monotonic
        var loopTime: TimeSource.Monotonic.ValueTimeMark = timeSource.markNow()

        // is this bad? maybe switch to a singleton
        val cycle = CycleCommand(intake, lift, out)

        var state = RobotState.LOCK
        var height = 0
        intake.setHeight(1)

        Robot.read(intake, out)
        cycle.update(state, height)
        Robot.write(intake, out)

        val propProcessor = PropPositionProcessor()
        val visionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName::class.java, "propCam"), propProcessor)
        while (opModeInInit()) {
            telemetry.addData("prop position", propProcessor.getPosition())
            telemetry.update()
        }

        val traj = RRTrajectories(drive, Globals.side, Globals.start, Globals.lane, propProcessor.getPosition())
        drive.poseEstimate = traj.startPose

        val autoMachine = StateMachineBuilder()
                    .state(AutoStates.DROP_OFF)
                    .onEnter { drive.followTrajectorySequenceAsync(traj.dropOff) }
                    .transition { !drive.isBusy }
                    .onExit { intake.setHeight(5) }
                    .state(AutoStates.TO_BACKDROP)
                    .onEnter { drive.followTrajectorySequenceAsync(traj.score) }
                    .transition { !drive.isBusy }
                    .state(AutoStates.RAISE)
                    .onEnter { height = 1; state = RobotState.BACKDROP }
                    .transitionTimed(2.0)
                    .state(AutoStates.EXTEND)
                    .onEnter { state = RobotState.EXTEND }
                    .transitionTimed(1.0)
                    .state(AutoStates.SCORE)
                    .onEnter { state = RobotState.SCORE }
                    .transitionTimed(1.0)
                    .state(AutoStates.RETRACT)
                    .onEnter { state = RobotState.BACKDROP }
                    .transitionTimed(1.0)
                    .state(AutoStates.LOWER)
                    .onEnter { height = 0; state = RobotState.LOCK }
                    .transitionTimed(2.0)
                    .state(AutoStates.PARK)
                    .onEnter { drive.followTrajectorySequenceAsync(traj.park) }
                    .build()
        autoMachine.start()

        visionPortal.setProcessorEnabled(propProcessor, false)
        visionPortal.close()
        while (opModeIsActive() && !isStopRequested) {
            for (hub in allHubs) {
                hub.clearBulkCache()
            }
            val loop = timeSource.markNow()
            val dt = (loop - loopTime).toDouble(DurationUnit.MILLISECONDS)
            loopTime = loop

            // update all subsystems
            Robot.read(intake, lift, out);
            autoMachine.update()
            cycle.update(state, height)
            drive.update()
            Robot.write(intake, lift, out);

            telemetry.addData("prop pos", traj.prop)
            telemetry.addData("lift dist", lift.distance)
            telemetry.addData("auto state", autoMachine.state as AutoStates)
            telemetry.addData("hz", 1000 / dt)
            telemetry.update()
        }
    }
}