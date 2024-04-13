package org.firstinspires.ftc.teamcode.seventh.opmode.test;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.seventh.robot.hardware.Robot
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.DroneSubsystem

@TeleOp(name = "DroneTest")
class DroneTest : LinearOpMode() {
    override fun runOpMode() {
        val gamepad = GamepadEx(gamepad1)
        val drone = DroneSubsystem(hardwareMap)
        var state = DroneSubsystem.State.LODED

        Robot.write(drone)
        waitForStart()

        while (opModeIsActive()) {
            gamepad.readButtons()
            when {
                gamepad.wasJustPressed(GamepadKeys.Button.B) -> state = DroneSubsystem.State.LODED
                gamepad.wasJustPressed(GamepadKeys.Button.A) -> state = DroneSubsystem.State.DIPER
            }
            drone.set(state)
            Robot.write(drone)
            telemetry.addData("state", state)
            telemetry.update()
        }
    }
}
