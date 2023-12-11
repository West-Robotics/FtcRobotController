package org.firstinspires.ftc.teamcode.seventh.opmode.test

import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.teamcode.seventh.robot.hardware.Robot
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.IntakeSubsystem
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.RobotState

@TeleOp(name = "IntakeTest")
class IntakeTest : LinearOpMode() {
    @Override
    override fun runOpMode() {
        Robot.hardwareMap = hardwareMap
        val intake = IntakeSubsystem(Robot.hardwareMap)
        val gamepad = GamepadEx(gamepad1)
        intake.update(RobotState.LOCK)
        Robot.write(intake)

        waitForStart()

        while (opModeIsActive() && !isStopRequested) {
            gamepad.readButtons()
            Robot.read(intake)
            if (gamepad.getButton(GamepadKeys.Button.A)) {
                intake.update(RobotState.INTAKE)
            } else if (gamepad.getButton(GamepadKeys.Button.B)) {
                intake.update(RobotState.LOCK)
            } else if (gamepad.getButton(GamepadKeys.Button.LEFT_BUMPER)) {
                intake.update(RobotState.SPIT)
            } else if (gamepad.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                intake.raise()
            } else if (gamepad.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
                intake.lower()
            }
            Robot.write(intake)
        }
    }
}