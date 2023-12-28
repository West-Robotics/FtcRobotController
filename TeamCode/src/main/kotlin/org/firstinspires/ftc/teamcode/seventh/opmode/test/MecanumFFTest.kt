package org.firstinspires.ftc.teamcode.seventh.opmode.test

import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.scrapmetal.quackerama.control.Pose2d
import com.scrapmetal.quackerama.control.Rotation2d
import com.scrapmetal.quackerama.control.Vector2d
import org.firstinspires.ftc.teamcode.seventh.robot.hardware.Robot
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.DriveSubsystem
import java.lang.Math.toDegrees
import java.lang.Math.toRadians

@TeleOp(name = "Mecanum FF Measurement")
class MecanumFFTest : LinearOpMode() {
    override fun runOpMode() {
        Robot.hardwareMap = hardwareMap
        val drive = DriveSubsystem(hardwareMap)
        val gamepad = GamepadEx(gamepad1)
        val allHubs = hardwareMap.getAll(LynxModule::class.java)
        for (hub in allHubs) {
            hub.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL
        }

        var input = Pose2d(Vector2d(0.01, 0.0), Rotation2d(0.0))
        waitForStart()

        while (opModeIsActive() && !isStopRequested) {
            for (hub in allHubs) {
                hub.clearBulkCache()
            }
            Robot.read(drive)
            gamepad.readButtons()

            input = when {
                gamepad.wasJustPressed(GamepadKeys.Button.DPAD_UP) -> Pose2d(input.position + input.position.unit*0.01, input.heading)
                gamepad.wasJustPressed(GamepadKeys.Button.DPAD_DOWN) -> Pose2d(input.position - input.position.unit*0.01, input.heading)
                gamepad.wasJustPressed(GamepadKeys.Button.DPAD_LEFT) -> Pose2d(Vector2d(input.position.mag, Rotation2d(input.position.polarAngle + toRadians(10.0))), input.heading)
                gamepad.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT) -> Pose2d(Vector2d(input.position.mag, Rotation2d(input.position.polarAngle - toRadians(10.0))), input.heading)
                else -> input
            }
            drive.update(input = input,
                         correcting = false,
                         fieldOriented = false,
                         dt = Robot.dt)

            Robot.write(drive)
            telemetry.addData("mag", input.position.mag)
            telemetry.addData("ang", toDegrees(input.position.polarAngle))
            telemetry.update()
        }
    }
}