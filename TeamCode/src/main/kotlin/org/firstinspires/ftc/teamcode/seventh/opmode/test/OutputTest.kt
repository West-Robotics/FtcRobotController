package org.firstinspires.ftc.teamcode.seventh.opmode.test

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
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
        var state = RobotState.LOCK
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        waitForStart()

        while (opModeIsActive() && !isStopRequested) {
            Robot.read(output)
            gamepad.readButtons()
            if (gamepad.getButton(GamepadKeys.Button.A)) {
                state = RobotState.INTAKE
            } else if (gamepad.getButton(GamepadKeys.Button.B)) {
                state = RobotState.LOCK
            } else if (gamepad.getButton(GamepadKeys.Button.X)) {
                state = RobotState.BACKDROP
            } else if (gamepad.getButton(GamepadKeys.Button.Y)) {
                state = RobotState.SCORE
            // } else if (gamepad.gamepad.guide, gamepad.gamepad.) {
                // gamepad.gamepad.ps
                // gamepad.gamepad.share
                // gamepad.gamepad.x
                // gamepad.gamepad.touchpad_finger_1_x
                // gamepad.gamepad.touchpad_finger_1_y
                // output.update(RobotState.EXTEND)
            }
            output.update(state, -125.0)
            Robot.write(output)
            telemetry.addData("max pos", output.armLeft.getCommandedPosition())
            telemetry.addData("x", gamepad.gamepad.x)
            telemetry.addData("y", gamepad.gamepad.y)
            telemetry.addData("left filled", output.leftFilled)
            telemetry.addData("right filled", output.rightFilled)
            telemetry.addData("profile velo", output.mpState.v)
            telemetry.update()
        }
    }
}
