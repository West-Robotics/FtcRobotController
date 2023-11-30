package org.firstinspires.ftc.teamcode.seventh.opmode.test

import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple

import org.firstinspires.ftc.teamcode.seventh.robot.hardware.Hardware
import org.firstinspires.ftc.teamcode.seventh.robot.hardware.Robot
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.OutputSubsystem
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.RobotState

@TeleOp(name = "OutputTest")
class OutputTest : LinearOpMode() {
    @Override
    override fun runOpMode() {
        Robot.hardwareMap = hardwareMap
        val output = OutputSubsystem(Robot.hardwareMap)
        val gamepad = GamepadEx(gamepad1)

        waitForStart()

        while (opModeIsActive() && !isStopRequested) {
            Robot.read(output)
            gamepad.readButtons()
            if (gamepad.getButton(GamepadKeys.Button.A)) {
                output.update(RobotState.INTAKE, -130.0)
            } else if (gamepad.getButton(GamepadKeys.Button.B)) {
                output.update(RobotState.LOCK, -125.0)
            } else if (gamepad.getButton(GamepadKeys.Button.X)) {
                output.update(RobotState.BACKDROP, -30.0)
            } else if (gamepad.getButton(GamepadKeys.Button.Y)) {
                output.update(RobotState.SCORE, -30.0)
            // } else if (gamepad.gamepad.guide, gamepad.gamepad.) {
                // gamepad.gamepad.ps
                // gamepad.gamepad.share
                // gamepad.gamepad.x
                // gamepad.gamepad.touchpad_finger_1_x
                // gamepad.gamepad.touchpad_finger_1_y
                // output.update(RobotState.EXTEND)
            }
            Robot.write(output)
            telemetry.addData("max pos", output.armLeft.getCommandedPosition())
            telemetry.addData("x", gamepad.gamepad.x)
            telemetry.addData("y", gamepad.gamepad.y)
            telemetry.addData("left filled", output.leftFilled)
            telemetry.addData("right filled", output.rightFilled)
            telemetry.update()
        }
    }
}
