package org.firstinspires.ftc.teamcode.seventh.opmode.test

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.arcrobotics.ftclib.command.CommandScheduler
import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.PerpetualCommand
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.outoftheboxrobotics.photoncore.Photon
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.seventh.opmode.teleop.Gigapad

import org.firstinspires.ftc.teamcode.seventh.robot.hardware.Robot
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.OutputSubsystem
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.OutputSubsystem.*

@Photon
@TeleOp(name = "OutputTest")
class OutputTest : LinearOpMode() {
    @Override
    override fun runOpMode() {
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
        val gamepad = Gigapad(gamepad1)
        Robot.init(hardwareMap, telemetry, null, null)
        val output = OutputSubsystem(Robot.getHwMap())

        // gamepad.ne.whenActive(InstantCommand({ output.set(Roll.LM_I) }))
        gamepad.north.whenActive(InstantCommand({ output.set(Roll.VERT_I) }))
        gamepad.nw.whenActive(InstantCommand({ output.set(Roll.RM) }))
        gamepad.west.whenActive(InstantCommand({ output.set(Roll.HORIZ) }))
        gamepad.sw.whenActive(InstantCommand({ output.set(Roll.LM) }))
        gamepad.south.whenActive(InstantCommand({ output.set(Roll.VERT) }))
        // gamepad.se.whenActive(InstantCommand({ output.set(Roll.RM_I) }))
        gamepad.rTrig.whenActive(InstantCommand({ output.set(Claw.NONE) }))
        gamepad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
            .whenActive(InstantCommand({ output.set(Claw.RIGHT) }))
        gamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
            .whenActive(InstantCommand({ output.set(Claw.LEFT) }))
        gamepad.getGamepadButton(GamepadKeys.Button.B)
            .whenActive(InstantCommand({ output.set(Claw.BOTH) }))
        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_UP)
            .whenPressed(InstantCommand({ output.set(Arm.BACKDROP) })
                .andThen(InstantCommand({ output.set(Pitch.OUT) })))
        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
            .whenPressed(InstantCommand({ output.set(Arm.IN) })
                .andThen(InstantCommand({ output.set(Pitch.IN) })))
        // PerpetualCommand(InstantCommand({ output.set(70*-gamepad.rightX) })).schedule()
        waitForStart()

        while (opModeIsActive() && !isStopRequested) {
            Robot.read(output)
            output.set(45*-gamepad.rightX)
            // does this add latency?
            telemetry.addData("hz", 1/Robot.getDt())
            telemetry.addData("left voltage", output.endLAng)
            telemetry.addData("right voltage", output.endRAng)
            Robot.write(output)
        }
    }
}
