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
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.GetPropPositionPipeline
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.IntakeSubsystem
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.LiftSubsystem
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.OutputSubsystem
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.RobotState
import org.openftc.easyopencv.OpenCvCamera
import org.openftc.easyopencv.OpenCvCameraFactory
import org.openftc.easyopencv.OpenCvCameraRotation
import kotlin.time.DurationUnit
import kotlin.time.TimeSource

enum class AutoStates {
    GRAB,
    COLLECT,
    DROP_OFF,
    TO_BACKDROP,
    RAISE,
    EXTEND,
    SAFE,
    UP_AGAIN,
    SCORE,
    BACK_OFF,
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
        intake.setHeight(5)

        Robot.read(intake, out)
        cycle.update(state, height, 0.0)
        Robot.write(intake, out)

        // val propProcessor = PropPositionProcessor()
        // val visionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName::class.java, "propCam"), propProcessor)
        // while (opModeInInit()) {
        //     telemetry.addData("prop position", propProcessor.getPosition())
        //     telemetry.update()
        // }
        val cameraMonitorViewId: Int = hardwareMap.appContext.resources.getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.packageName);
        val propCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName::class.java, "propCam"), cameraMonitorViewId);
        val propPosition = GetPropPositionPipeline()
        propCam.openCameraDeviceAsync(object: OpenCvCamera.AsyncCameraOpenListener {
            override fun onOpened() {
                propCam.startStreaming(1280, 720, OpenCvCameraRotation.UPSIDE_DOWN);
            }

            override fun onError(errorCode: Int) { }
        });
        propCam.setPipeline(propPosition);
        waitForStart()

        val traj = RRTrajectories(drive, Globals.side, Globals.start, Globals.lane, propPosition.getPosition())
        drive.poseEstimate = traj.startPose

        val autoMachine = StateMachineBuilder()
                .state(AutoStates.GRAB)
                .onEnter { drive.followTrajectorySequenceAsync(traj.collect)}
                .transition { !drive.isBusy}
                .onExit { intake.setHeight(1) }
                .state(AutoStates.COLLECT)
                .transitionTimed(0.5)
                .onExit { drive.followTrajectorySequenceAsync(traj.dropOff) }
                .state(AutoStates.DROP_OFF)
                .transition { !drive.isBusy }
                .onExit { intake.setHeight(5); drive.followTrajectorySequenceAsync(traj.score)  }
                .state(AutoStates.TO_BACKDROP)
                .transition { !drive.isBusy }
                .onExit { height = 1; state = RobotState.BACKDROP }
                .state(AutoStates.RAISE)
                .transitionTimed(1.5)
                .onExit { state = RobotState.EXTEND }
                .state(AutoStates.EXTEND)
                .transitionTimed(1.0)
                .onExit { height = -1 }
                .state(AutoStates.SAFE)
                .transitionTimed(1.0)
                .onExit { state = RobotState.SCORE }
                .state(AutoStates.SCORE)
                .transitionTimed(1.0)
                .onExit { height = 1 }
                .state(AutoStates.UP_AGAIN)
                .transitionTimed(1.0)
                .onExit { drive.followTrajectorySequenceAsync(traj.park) }
                .state(AutoStates.PARK)
                .transition { !drive.isBusy }
                .onExit { state = RobotState.BACKDROP }
                .state(AutoStates.RETRACT)
                .transitionTimed(1.0)
                .onExit { height = 0; state = RobotState.LOCK }
                .state(AutoStates.LOWER)
                .build()
        autoMachine.start()

        // visionPortal.setProcessorEnabled(propProcessor, false)
        // visionPortal.close()
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
            cycle.update(state, height, 0.0)
            drive.update()
            Robot.write(intake, lift, out);

            telemetry.addData("prop pos", traj.prop)
            // telemetry.addData("lift dist", lift.distance)
            telemetry.addData("auto state", autoMachine.state as AutoStates)
            telemetry.addData("hz", 1000 / dt)
            telemetry.update()
        }
    }
}