// package org.firstinspires.ftc.teamcode.seventh.opmode.auto
//
// import com.qualcomm.robotcore.eventloop.opmode.Autonomous
// import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
// import com.scrapmetal.quackerama.control.gvf.GG
// import org.firstinspires.ftc.teamcode.seventh.opmode.teleop.Gigapad
// import org.firstinspires.ftc.teamcode.seventh.robot.hardware.Globals
// import org.firstinspires.ftc.teamcode.seventh.robot.hardware.Robot
// import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.DriveSubsystem
// import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.IntakeSubsystem
// import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.LiftSubsystem
// import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.OutputSubsystem
// import org.firstinspires.ftc.teamcode.seventh.robot.vision.GetPropPositionPipeline
// import org.firstinspires.ftc.teamcode.seventh.robot.vision.Vision
// import java.lang.Math.toDegrees
// import kotlin.time.TimeSource
//
// @Autonomous(name =
// """
// ⬛🟥🟥🟥⬛🟩🟩🟩⬛🟫🟫🟫⬛🟧🟧🟧⬛🟨🟨🟨⬛
// 🟥🟥🟦🟦🟩🟩🟦🟦🟫🟫🟦🟦🟧🟧🟦🟦🟨🟨🟦🟦🟦
// 🟥🟥🟥🟥🟩🟩🟩🟩🟫🟫🟫🟫🟧🟧🟧🟧🟨🟨🟨🟨🟦
// ⬛🟥⬛🟥⬛🟩⬛🟩⬛🟫⬛🟫⬛🟧⬛🟧⬛🟨⬛🟨⬛
// ⬛🟨🟨🟨⬛🟥🟥🟥⬛🟩🟩🟩⬛🟫🟫🟫⬛🟧🟧🟧⬛
// 🟨🟨🟦🟦🟥🟥🟦🟦🟩🟩🟦🟦🟫🟫🟦🟦🟧🟧🟦🟦🟦
// 🟨🟨🟨🟨🟥🟥🟥🟥🟩🟩🟩🟩🟫🟫🟫🟫🟧🟧🟧🟧🟦
// ⬛🟨⬛🟨⬛🟥⬛🟥⬛🟩⬛🟩⬛🟫⬛🟫⬛🟧⬛🟧⬛
// ⬛🟧🟧🟧⬛🟨🟨🟨⬛🟥🟥🟥⬛🟩🟩🟩⬛🟫🟫🟫⬛
// 🟧🟧🟦🟦🟨🟨🟦🟦🟥🟥🟦🟦🟩🟩🟦🟦🟫🟫🟦🟦🟦
// 🟧🟧🟧🟧🟨🟨🟨🟨🟥🟥🟥🟥🟩🟩🟩🟩🟫🟫🟫🟫🟦
// ⬛🟧⬛🟧⬛🟨⬛🟨⬛🟥⬛🟥⬛🟩⬛🟩⬛🟫⬛🟫⬛
// ⬛🟫🟫🟫⬛🟧🟧🟧⬛🟨🟨🟨⬛🟥🟥🟥⬛🟩🟩🟩⬛
// 🟫🟫🟦🟦🟧🟧🟦🟦🟨🟨🟦🟦🟥🟥🟦🟦🟩🟩🟦🟦🟦
// 🟫🟫🟫🟫🟧🟧🟧🟧🟨🟨🟨🟨🟥🟥🟥🟥🟩🟩🟩🟩🟦
// ⬛🟫⬛🟫⬛🟧⬛🟧⬛🟨⬛🟨⬛🟥⬛🟥⬛🟩⬛🟩⬛
// ⬛⬛⬛⬛⬛⬛⬛⬛⬛⬛⬛⬛⬛⬛⬛⬛⬛⬛⬛⬛⬛
// """
// )
// class SussyAuto : LinearOpMode() {
//     override fun runOpMode() {
//         Globals.AUTO = true
//         Globals.alliance = Globals.Alliance.RED
//         Globals.lane = Globals.Lane.LANE_1
//         Globals.start = Globals.Start.AUDIENCE
//         val drive = DriveSubsystem(hardwareMap)
//         val intake = IntakeSubsystem(hardwareMap)
//         val lift = LiftSubsystem(hardwareMap)
//         val output = OutputSubsystem(hardwareMap)
//         val vision = Vision(hardwareMap)
//         val gamepad = Gigapad(gamepad1)
//         Robot.init(hardwareMap, telemetry, gamepad, null)
//
//         var stackCount = 5
//
//         lateinit var gg: GG
//         intake.setHeight(1)
//
//         // vision.enableProp()
//         drive.startIMUThread(this)
//         while (opModeInInit()) {
//             telemetry.addData("prop", vision.getPropPosition())
//             telemetry.update()
//         }
//         val paths = AutoPositions(
//                 Globals.Alliance.BLUE,
//                 Globals.Start.BACKDROP,
//                 Globals.Lane.LANE_2,
//                 Globals.YellowSide.LEFT,
//                 Globals.Stack.CLOSE,
//                 Globals.Park.INNER,
//                 GetPropPositionPipeline.PropPosition.MIDDLE,
//         )
//         vision.disableProp()
//         drive.setPoseEstimate(paths.initPose)
//         gg = GG(
//                 kN = 0.5,
//                 kD = 0.004,
//                 maxVel = 1.0,
//                 paths.purple,
//                 paths.yellow,
//                 paths.intake,
//                 paths.score,
//                 paths.parkPath,
//         )
//         autoMachine.start()
//
//         val timeSource = TimeSource.Monotonic
//         while (opModeIsActive()) {
//             autoMachine.update()
//             // update all subsystems
//             Robot.read(intake, output, lift, drive)
//
//             val input = gg.update(drive.getPoseEstimate().position, drive.getVelocity().position)
//             drive.updateClosed(input, true)
//
//             telemetry.addData("hz", 1 / Robot.getDt())
//             telemetry.addData("error", gg.error(drive.getPoseEstimate().position))
//             telemetry.addData("onTarget", gg.onTarget(drive.getPoseEstimate().position))
//             telemetry.addData("x", drive.getPoseEstimate().position.x)
//             telemetry.addData("y", drive.getPoseEstimate().position.y)
//             telemetry.addData("heading error", toDegrees(input.heading.polarAngle - drive.getPoseEstimate().heading.polarAngle))
//             Robot.write(drive, lift, output, intake)
//         }
//     }
// }