package org.firstinspires.ftc.teamcode.seventh.opmode.test

import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.outoftheboxrobotics.photoncore.Photon
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.seventh.opmode.teleop.Gigapad
import org.firstinspires.ftc.teamcode.seventh.robot.hardware.Robot
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.IntakeSubsystem
import org.firstinspires.ftc.teamcode.seventh.robot.command.RobotState

@Photon
@TeleOp(name = "IntakeTest")
class IntakeTest : LinearOpMode() {
    @Override
    override fun runOpMode() {
        val gigapad = Gigapad(gamepad1)
        Robot.init(hardwareMap, telemetry, gigapad, null)
        val intake = IntakeSubsystem(Robot.getHwMap())

        gigapad.getGamepadButton(GamepadKeys.Button.A)
            .whenPressed(InstantCommand({ intake.set(IntakeSubsystem.State.INTAKE) }) )
        gigapad.getGamepadButton(GamepadKeys.Button.B)
            .whenPressed(InstantCommand({ intake.set(IntakeSubsystem.State.STOP) }) )
        gigapad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
            .whenPressed(InstantCommand({ intake.set(IntakeSubsystem.State.SPIT) }) )
        gigapad.getGamepadButton(GamepadKeys.Button.Y)
            .whenPressed(InstantCommand({ intake.raise() }) )
        gigapad.getGamepadButton(GamepadKeys.Button.X)
            .whenPressed(InstantCommand({ intake.lower() }) )
        waitForStart()

        while (opModeIsActive() && !isStopRequested) {
            Robot.read(intake)
            telemetry.addData("left break", intake.filledL)
            telemetry.addData("right break", intake.filledR)
            Robot.write(intake)
        }
    }
}