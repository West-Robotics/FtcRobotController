package org.firstinspires.ftc.teamcode.seventh.opmode.test;

import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.outoftheboxrobotics.photoncore.Photon
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.seventh.opmode.teleop.Gigapad
import org.firstinspires.ftc.teamcode.seventh.robot.hardware.Robot
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.DroneSubsystem

@Photon
@TeleOp(name = "DroneTest")
class DroneTest : LinearOpMode() {
    override fun runOpMode() {
        val gigapad = Gigapad(gamepad1)
        Robot.init(hardwareMap, telemetry, gigapad, null)
        val drone = DroneSubsystem(hardwareMap)

        gigapad.getGamepadButton(GamepadKeys.Button.START).whenActive(InstantCommand({ drone.set(DroneSubsystem.State.DIPER) }))
        gigapad.getGamepadButton(GamepadKeys.Button.B).whenActive(InstantCommand({ drone.set(DroneSubsystem.State.LODED) }))
        waitForStart()

        while (opModeIsActive()) {
            Robot.read(drone)
            Robot.write(drone)
        }
    }
}
