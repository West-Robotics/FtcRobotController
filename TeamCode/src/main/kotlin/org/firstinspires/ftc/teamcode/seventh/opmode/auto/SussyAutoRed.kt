// package org.firstinspires.ftc.teamcode.seventh.opmode.auto
//
// import com.arcrobotics.ftclib.gamepad.GamepadEx
// import com.qualcomm.hardware.lynx.LynxModule
// import com.qualcomm.robotcore.eventloop.opmode.Autonomous
// import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
// import org.firstinspires.ftc.teamcode.seventh.drive.SampleMecanumDrive
// import org.firstinspires.ftc.teamcode.seventh.robot.command.AutoMachines
// import org.firstinspires.ftc.teamcode.seventh.robot.command.CycleCommand
// import org.firstinspires.ftc.teamcode.seventh.robot.command.RRTrajectories
// import org.firstinspires.ftc.teamcode.seventh.robot.hardware.Globals
// import org.firstinspires.ftc.teamcode.seventh.robot.hardware.Hardware
// import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.IntakeSubsystem
// import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.LiftSubsystem
// import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.OutputSubsystem
// import kotlin.time.DurationUnit
// import kotlin.time.TimeSource
//
// @Autonomous(name =
//     "RED ⬛🟥🟥🟥⬛🟩🟩🟩⬛🟫🟫🟫⬛🟧🟧🟧⬛🟨🟨🟨⬛\n" +
//     "🟥🟥🟦🟦🟩🟩🟦🟦🟫🟫🟦🟦🟧🟧🟦🟦🟨🟨🟦🟦🟦\n" +
//     "🟥🟥🟥🟥🟩🟩🟩🟩🟫🟫🟫🟫🟧🟧🟧🟧🟨🟨🟨🟨🟦\n" +
//     "⬛🟥⬛🟥⬛🟩⬛🟩⬛🟫⬛🟫⬛🟧⬛🟧⬛🟨⬛🟨⬛\n" +
//     "⬛🟨🟨🟨⬛🟥🟥🟥⬛🟩🟩🟩⬛🟫🟫🟫⬛🟧🟧🟧⬛\n" +
//     "🟨🟨🟦🟦🟥🟥🟦🟦🟩🟩🟦🟦🟫🟫🟦🟦🟧🟧🟦🟦🟦\n" +
//     "🟨🟨🟨🟨🟥🟥🟥🟥🟩🟩🟩🟩🟫🟫🟫🟫🟧🟧🟧🟧🟦\n" +
//     "⬛🟨⬛🟨⬛🟥⬛🟥⬛🟩⬛🟩⬛🟫⬛🟫⬛🟧⬛🟧⬛\n" +
//     "⬛🟧🟧🟧⬛🟨🟨🟨⬛🟥🟥🟥⬛🟩🟩🟩⬛🟫🟫🟫⬛\n" +
//     "🟧🟧🟦🟦🟨🟨🟦🟦🟥🟥🟦🟦🟩🟩🟦🟦🟫🟫🟦🟦🟦\n" +
//     "🟧🟧🟧🟧🟨🟨🟨🟨🟥🟥🟥🟥🟩🟩🟩🟩🟫🟫🟫🟫🟦\n" +
//     "⬛🟧⬛🟧⬛🟨⬛🟨⬛🟥⬛🟥⬛🟩⬛🟩⬛🟫⬛🟫⬛\n" +
//     "⬛🟫🟫🟫⬛🟧🟧🟧⬛🟨🟨🟨⬛🟥🟥🟥⬛🟩🟩🟩⬛\n" +
//     "🟫🟫🟦🟦🟧🟧🟦🟦🟨🟨🟦🟦🟥🟥🟦🟦🟩🟩🟦🟦🟦\n" +
//     "🟫🟫🟫🟫🟧🟧🟧🟧🟨🟨🟨🟨🟥🟥🟥🟥🟩🟩🟩🟩🟦\n" +
//     "⬛🟫⬛🟫⬛🟧⬛🟧⬛🟨⬛🟨⬛🟥⬛🟥⬛🟩⬛🟩⬛\n" +
//     "⬛🟩🟩🟩⬛🟫🟫🟫⬛🟧🟧🟧⬛🟨🟨🟨⬛🟥🟥🟥⬛\n" +
//     "🟩🟩🟦🟦🟫🟫🟦🟦🟧🟧🟦🟦🟨🟨🟦🟦🟥🟥🟦🟦🟦\n" +
//     "🟩🟩🟩🟩🟫🟫🟫🟫🟧🟧🟧🟧🟨🟨🟨🟨🟥🟥🟥🟥🟦\n" +
//     "⬛🟩⬛🟩⬛🟫⬛🟫⬛🟧⬛🟧⬛🟨⬛🟨⬛🟥⬛🟥⬛\n" +
//     "⬛⬛⬛⬛⬛⬛⬛⬛⬛⬛⬛⬛⬛⬛⬛⬛⬛⬛⬛⬛⬛")
// class SussyAutoRed : LinearOpMode() {
//     override fun runOpMode() {
//         Globals.AUTO = true
//         Globals.side = Globals.Side.RED
//         Globals.start = Globals.Start.CLOSE
//         Globals.lane = Globals.Lane.LANE_1
//         val hardware = Hardware.getInstance(hardwareMap)
//         val allHubs = hardwareMap.getAll(LynxModule::class.java)
//         for (hub in allHubs) {
//             hub.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL
//         }
//         val drive =
//             SampleMecanumDrive(
//                 hardware,
//                 hardwareMap
//             )
//         val intake = IntakeSubsystem(hardware)
//         val lift = LiftSubsystem(hardware)
//         val out = OutputSubsystem(hardware)
//
//         val primary = GamepadEx(gamepad1)
//         val secondary = GamepadEx(gamepad2)
//         val timeSource = TimeSource.Monotonic
//         var loopTime: TimeSource.Monotonic.ValueTimeMark = timeSource.markNow()
//
//         // is this bad? maybe switch to a singleton
//         val cycle = CycleCommand(intake, lift, out)
//
//         hardware.read(intake, out)
//         cycle.update(CycleCommand.CycleState.LOCK, OutputSubsystem.OutputState.LOCK, -1)
//         hardware.write(intake, out)
//
//         while (opModeInInit()) {
//             telemetry.addData("prop position", hardware.propPosition.getPosition())
//             telemetry.update()
//         }
//
//         val traj = RRTrajectories(drive, Globals.side, Globals.start, Globals.lane, hardware.propPosition.getPosition())
//         val autoMachine = AutoMachines.getAutoMachine(drive, cycle, traj)
//         autoMachine.start()
//         hardware.propCam.closeCameraDevice()
//
//         // drive.poseEstimate = Pose2d(0.0, 0.0, 0.0)
//         // val t: TrajectorySequence = drive.trajectorySequenceBuilder(Pose2d( 0.0, 0.0,-90.0))
//         //     .back(10.0, SampleMecanumDrive.getVelocityConstraint(20.0, toRadians(60.0), DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(10.0))
//         //     .turn(90.0, toRadians(30.0), toRadians(30.0))
//         //     .turn(-90.0, toRadians(30.0), toRadians(30.0))
//         //     .build()
//         // drive.followTrajectorySequenceAsync(t)
//         // val t: TrajectorySequence = drive.trajectorySequenceBuilder(Pose2d(0.0, 0.0, 0.0))
//         //     .forward(10.0, SampleMecanumDrive.getVelocityConstraint(20.0, toRadians(60.0), DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(10.0))
//         //     .build()
//         // drive.followTrajectorySequence(t)
//         while (opModeIsActive() && !isStopRequested) {
//             for (hub in allHubs) {
//                 hub.clearBulkCache()
//             }
//             val loop = timeSource.markNow()
//             val dt = (loop - loopTime).toDouble(DurationUnit.MILLISECONDS)
//             loopTime = loop
//
//             // update all subsystems
//             hardware.read(intake, lift, out);
//             autoMachine.update()
//             cycle.update()
//             drive.update()
//             hardware.write(intake, lift, out);
//
//             telemetry.addData("prop pos", traj.prop)
//             telemetry.addData("lift dist", lift.distance)
//             telemetry.addData("list state", lift.state)
//             telemetry.addData("auto state", autoMachine.state as AutoMachines.AutoStates)
//             telemetry.addData("hz", 1000 / dt)
//             telemetry.update()
//         }
//         // hardware.visionPortal.close()
//         // hardware.stop()
//     }
// }