package org.firstinspires.ftc.teamcode.seventh.opmode.auto

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.seventh.robot.command.AutoMachines
import org.firstinspires.ftc.teamcode.seventh.robot.command.CycleCommand
import org.firstinspires.ftc.teamcode.seventh.robot.command.RRTrajectories
import org.firstinspires.ftc.teamcode.seventh.robot.hardware.Globals
import org.firstinspires.ftc.teamcode.seventh.robot.hardware.Hardware
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.IntakeSubsystem
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.LiftSubsystem
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.OutputSubsystem
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.PropPositionProcessor
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder
import java.lang.Math.toDegrees
import java.lang.Math.toRadians
import kotlin.time.DurationUnit
import kotlin.time.TimeSource

@Autonomous(name =
    "🟦🟥🟥🟥🟦🟩🟩🟩🟦🟫🟫🟫🟦🟧🟧🟧🟦🟨🟨🟨🟦\n" +
    "🟥🟥🟦🟦🟩🟩🟦🟦🟫🟫🟦🟦🟧🟧🟦🟦🟨🟨🟦🟦🟦\n" +
    "🟥🟥🟥🟥🟩🟩🟩🟩🟫🟫🟫🟫🟧🟧🟧🟧🟨🟨🟨🟨🟦\n" +
    "🟦🟥🟦🟥🟦🟩🟦🟩🟦🟫🟦🟫🟦🟧🟦🟧🟦🟨🟦🟨🟦\n" +
    "🟦🟨🟨🟨🟦🟥🟥🟥🟦🟩🟩🟩🟦🟫🟫🟫🟦🟧🟧🟧🟦\n" +
    "🟨🟨🟦🟦🟥🟥🟦🟦🟩🟩🟦🟦🟫🟫🟦🟦🟧🟧🟦🟦🟦\n" +
    "🟨🟨🟨🟨🟥🟥🟥🟥🟩🟩🟩🟩🟫🟫🟫🟫🟧🟧🟧🟧🟦\n" +
    "🟦🟨🟦🟨🟦🟥🟦🟥🟦🟩🟦🟩🟦🟫🟦🟫🟦🟧🟦🟧🟦\n" +
    "🟦🟧🟧🟧🟦🟨🟨🟨🟦🟥🟥🟥🟦🟩🟩🟩🟦🟫🟫🟫🟦\n" +
    "🟧🟧🟦🟦🟨🟨🟦🟦🟥🟥🟦🟦🟩🟩🟦🟦🟫🟫🟦🟦🟦\n" +
    "🟧🟧🟧🟧🟨🟨🟨🟨🟥🟥🟥🟥🟩🟩🟩🟩🟫🟫🟫🟫🟦\n" +
    "🟦🟧🟦🟧🟦🟨🟦🟨🟦🟥🟦🟥🟦🟩🟦🟩🟦🟫🟦🟫🟦\n" +
    "🟦🟫🟫🟫🟦🟧🟧🟧🟦🟨🟨🟨🟦🟥🟥🟥🟦🟩🟩🟩🟦\n" +
    "🟫🟫🟦🟦🟧🟧🟦🟦🟨🟨🟦🟦🟥🟥🟦🟦🟩🟩🟦🟦🟦\n" +
    "🟫🟫🟫🟫🟧🟧🟧🟧🟨🟨🟨🟨🟥🟥🟥🟥🟩🟩🟩🟩🟦\n" +
    "🟦🟫🟦🟫🟦🟧🟦🟧🟦🟨🟦🟨🟦🟥🟦🟥🟦🟩🟦🟩🟦\n" +
    "🟦🟩🟩🟩🟦🟫🟫🟫🟦🟧🟧🟧🟦🟨🟨🟨🟦🟥🟥🟥🟦\n" +
    "🟩🟩🟦🟦🟫🟫🟦🟦🟧🟧🟦🟦🟨🟨🟦🟦🟥🟥🟦🟦🟦\n" +
    "🟩🟩🟩🟩🟫🟫🟫🟫🟧🟧🟧🟧🟨🟨🟨🟨🟥🟥🟥🟥🟦\n" +
    "🟦🟩🟦🟩🟦🟫🟦🟫🟦🟧🟦🟧🟦🟨🟦🟨🟦🟥🟦🟥🟦\n" +
    "🟦🟦🟦🟦🟦🟦🟦🟦🟦🟦🟦🟦🟦🟦🟦🟦🟦🟦🟦🟦🟦")
class SussyAuto : LinearOpMode() {
    override fun runOpMode() {
        Globals.AUTO = true
        Globals.side = Globals.Side.RED
        Globals.start = Globals.Start.CLOSE
        Globals.lane = Globals.Lane.LANE_1
        val allHubs = hardwareMap.getAll(LynxModule::class.java)
        for (hub in allHubs) {
            hub.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL
        }
        val hardware = Hardware.getInstance(hardwareMap)
        val drive = SampleMecanumDrive(hardware, hardwareMap)
        val intake = IntakeSubsystem(hardware)
        val lift = LiftSubsystem(hardware)
        val out = OutputSubsystem(hardware)

        val primary = GamepadEx(gamepad1)
        val secondary = GamepadEx(gamepad2)
        val timeSource = TimeSource.Monotonic
        var loopTime: TimeSource.Monotonic.ValueTimeMark = timeSource.markNow()

        // is this bad? maybe switch to a singleton
        val cycle = CycleCommand(intake, lift, out)

        hardware.read(intake, out)
        cycle.update(CycleCommand.CycleState.LOCK, OutputSubsystem.OutputState.LOCK)
        hardware.write(intake, out)

        telemetry.addLine("waiting for start")
        telemetry.update()
        val dashboard = FtcDashboard.getInstance()

        waitForStart()
        val traj = RRTrajectories(drive, Globals.side, Globals.start, Globals.lane, PropPositionProcessor.PropPosition.MIDDLE)
        val autoMachine = AutoMachines.getAutoMachine(drive, cycle, traj)
        autoMachine.start()

        val t: TrajectorySequence = drive.trajectorySequenceBuilder(Pose2d( 36.0, -65.6,-90.0))
            .back(10.0)
            .turn(90.0, toRadians(30.0), toRadians(30.0))
            .turn(-90.0, toRadians(30.0), toRadians(30.0))
            .build()
        drive.followTrajectorySequenceAsync(t)
        while (opModeIsActive() && !isStopRequested) {
            for (hub in allHubs) {
                hub.clearBulkCache()
            }
            val loop = timeSource.markNow()
            val dt = (loop - loopTime).toDouble(DurationUnit.MILLISECONDS)
            loopTime = loop

            // update all subsystems
            hardware.read(intake, out);
            // autoMachine.update()
            drive.update()
            hardware.write(intake, out);

            telemetry.addData("lift dist", lift.distance)
            telemetry.addData("list state", lift.state)
            telemetry.addData("auto state", autoMachine.state as AutoMachines.AutoStates)
            telemetry.addData("hz", 1000 / dt)
            telemetry.addData("raw ext heading", toDegrees(drive.rawExternalHeading))
            telemetry.addData("ext heading", toDegrees(drive.externalHeading))
            telemetry.addData("pose estimate heading", toDegrees(drive.poseEstimate.heading))
            telemetry.update()
            val packet = TelemetryPacket()
            packet.put("raw ext heading", toDegrees(drive.rawExternalHeading))
            packet.put("ext heading", toDegrees(drive.externalHeading))
            packet.put("pose estimate heading", toDegrees(drive.poseEstimate.heading))
            dashboard.sendTelemetryPacket(packet)
        }
        // hardware.visionPortal.close()
        // hardware.stop()
    }
}