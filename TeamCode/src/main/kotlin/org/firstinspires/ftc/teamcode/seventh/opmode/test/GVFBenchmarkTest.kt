// package org.firstinspires.ftc.teamcode.seventh.opmode.test
//
// import com.arcrobotics.ftclib.gamepad.GamepadEx
// import com.arcrobotics.ftclib.gamepad.GamepadKeys
// import com.qualcomm.hardware.lynx.LynxModule
// import com.qualcomm.robotcore.eventloop.opmode.Autonomous
// import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
// import com.scrapmetal.quackerama.control.gvf.GG
// import com.scrapmetal.quackerama.control.path.path
// import org.firstinspires.ftc.teamcode.seventh.opmode.teleop.Gigapad
// import org.firstinspires.ftc.teamcode.seventh.robot.hardware.Robot
// import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.DriveSubsystem
// import java.lang.Math.toDegrees
// import java.lang.Math.toRadians
//
// @Autonomous(name = "GVF Benchmark")
// class GVFBenchmarkTest : LinearOpMode() {
//     override fun runOpMode() {
//         val gamepad = Gigapad(gamepad1)
//         Robot.init(hardwareMap, telemetry, gamepad, null)
//         val drive = DriveSubsystem(hardwareMap)
//         val line = path { line { label("line")
//                 start(0.0, 0.0)
//                 end(80.0, 0.0)
//                 constraints {
//                     decelDist(12.0)
//                     heading(toRadians(0.0)) }}}
//         val hermite = path {
//             hermite {
//                 label("pixels to the back")
//                 start { pos(0.0, 0.0); ang(toRadians(0.0));  v(12.0) }
//                 end   { pos(40.0, 0.0); ang(toRadians(45.0)); v(50.0) }
//                 constraints {
//                     decelDist(12.0)
//                     heading(toRadians(0.0)) }}}
//         val gg = GG(0.9, 0.007, 1.0, line)
//
//         drive.startIMUThread(this)
//         while (opModeInInit()) {
//             telemetry.addData("dt", Robot.getDt()*1000)
//             telemetry.addData("x", drive.getPoseEstimate().position.x)
//             telemetry.addData("y", drive.getPoseEstimate().position.y)
//             telemetry.addData("current index", gg.currentIndex)
//             telemetry.addData("x", gg.update(drive.getPoseEstimate().position, drive.getVelocity().position).position.x)
//             telemetry.addData("y", gg.update(drive.getPoseEstimate().position, drive.getVelocity().position).position.y)
//             telemetry.update()
//         }
//         while (opModeIsActive()) {
//             Robot.read(drive)
//             // drive.update(
//             //     input = gg.update(drive.getPoseEstimate().position, drive.getVelocity().position),
//             //     correcting = false,
//             //     fieldOriented = true,
//             //     dt = Robot.dt,
//             //     pid = true,
//             // )
//             if (gamepad.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
//                 gg.currentIndex++
//             } else if (gamepad.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
//                 gg.currentIndex--
//             }
//             Robot.write(drive)
//             telemetry.addData("dt", Robot.getDt()*1000)
//             telemetry.addData("x", drive.getPoseEstimate().position.x)
//             telemetry.addData("y", drive.getPoseEstimate().position.y)
//             telemetry.addData("heading", toDegrees(drive.getPoseEstimate().heading.theta))
//             telemetry.addData("current index", gg.currentIndex)
//             telemetry.addData("x power", gg.update(drive.getPoseEstimate().position, drive.getVelocity().position).position.x)
//             telemetry.addData("y power", gg.update(drive.getPoseEstimate().position, drive.getVelocity().position).position.y)
//             telemetry.addData("turn power", toDegrees(gg.update(drive.getPoseEstimate().position, drive.getVelocity().position).heading.polarAngle))
//             telemetry.update()
//         }
//     }
// }
//