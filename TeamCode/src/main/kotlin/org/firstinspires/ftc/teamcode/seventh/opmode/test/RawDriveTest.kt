package org.firstinspires.ftc.teamcode.seventh.opmode.test

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive

@TeleOp(name = "Raw Drive Test")
class RawDriveTest : LinearOpMode() {
    override fun runOpMode() {
        val drive = SampleMecanumDrive(hardwareMap)
        val gamepad = GamepadEx(gamepad1)
        var turn = 0.0

        waitForStart()
        while (opModeIsActive() && !isStopRequested) {
            gamepad.readButtons()
            turn += when {
                gamepad.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER) -> 0.01
                gamepad.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER) -> -0.01
                else -> 0.0
            }
            drive.setWeightedDrivePower(Pose2d(gamepad.leftY, -gamepad.leftX, turn))
            drive.update()
            println("Hello World!")
            println("Goodbye World!")
            telemetry.addData("left x", gamepad.leftX)
            telemetry.addData("left y", gamepad.leftY)
            telemetry.addData("turn", turn)
            telemetry.update()
        }
    }
}