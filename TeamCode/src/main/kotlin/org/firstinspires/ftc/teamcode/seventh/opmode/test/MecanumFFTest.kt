// package org.firstinspires.ftc.teamcode.seventh.opmode.test
//
// import com.arcrobotics.ftclib.gamepad.GamepadKeys
// import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
// import com.qualcomm.robotcore.eventloop.opmode.TeleOp
// import com.scrapmetal.quackerama.control.Pose2d
// import com.scrapmetal.quackerama.control.Rotation2d
// import com.scrapmetal.quackerama.control.Vector2d
// import org.firstinspires.ftc.teamcode.seventh.opmode.teleop.Gigapad
// import org.firstinspires.ftc.teamcode.seventh.robot.hardware.Robot
// import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.DriveSubsystem
// import java.lang.Math.toDegrees
// import java.lang.Math.toRadians
//
// @TeleOp(name = "Mecanum FF Measurement")
// class MecanumFFTest : LinearOpMode() {
//     override fun runOpMode() {
//         val gamepad = Gigapad(gamepad1)
//         Robot.init(hardwareMap, telemetry, gamepad, null)
//         val drive = DriveSubsystem(hardwareMap)
//
//         var input = Pose2d(Vector2d(0.01, 0.0), Rotation2d(0.0))
//         waitForStart()
//
//         while (opModeIsActive() && !isStopRequested) {
//             Robot.read(drive)
//
//             input = when {
//                 gamepad.wasJustPressed(GamepadKeys.Button.DPAD_UP) -> Pose2d(input.position + input.position.unit*0.01, input.heading)
//                 gamepad.wasJustPressed(GamepadKeys.Button.DPAD_DOWN) -> Pose2d(input.position - input.position.unit*0.01, input.heading)
//                 gamepad.wasJustPressed(GamepadKeys.Button.DPAD_LEFT) -> Pose2d(Vector2d(input.position.mag, Rotation2d(input.position.polarAngle + toRadians(10.0))), input.heading)
//                 gamepad.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT) -> Pose2d(Vector2d(input.position.mag, Rotation2d(input.position.polarAngle - toRadians(10.0))), input.heading)
//                 else -> input
//             }
//             drive.updateOpen(input, false)
//
//             telemetry.addData("mag", input.position.mag)
//             telemetry.addData("ang", toDegrees(input.position.polarAngle))
//             Robot.write(drive)
//         }
//     }
// }