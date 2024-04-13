package org.firstinspires.ftc.teamcode.seventh.opmode.test

import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.seventh.opmode.teleop.Gigapad
import org.firstinspires.ftc.teamcode.seventh.robot.hardware.Robot

@TeleOp(name = "Gamepad Test")
class GamepadTest : LinearOpMode() {
    override fun runOpMode() {
        val gamepad = Gigapad(gamepad1)
        Robot.init(hardwareMap, telemetry, gamepad, null)

        waitForStart()
        while (opModeIsActive() && !isStopRequested) {
            Robot.read()
            telemetry.addData("dt", Robot.getDt()*1000)
            telemetry.addData("left x", gamepad.leftX)
            telemetry.addData("left y", gamepad.leftY)
            telemetry.addData("right x", gamepad.rightX)
            telemetry.addData("right y", gamepad.rightY)
            telemetry.addData("left trigger", gamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER))
            telemetry.addData("right trigger", gamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER))
            telemetry.addData("dpad up", gamepad.isDown(GamepadKeys.Button.DPAD_UP))
            telemetry.addData("dpad down", gamepad.isDown(GamepadKeys.Button.DPAD_DOWN))
            telemetry.addData("dpad left", gamepad.isDown(GamepadKeys.Button.DPAD_LEFT))
            telemetry.addData("dpad right", gamepad.isDown(GamepadKeys.Button.DPAD_RIGHT))
            telemetry.update()
        }
    }
}