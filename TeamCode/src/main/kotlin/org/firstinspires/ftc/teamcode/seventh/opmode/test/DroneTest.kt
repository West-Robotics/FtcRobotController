package org.firstinspires.ftc.teamcode.seventh.opmode.test;

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.configuration.LynxConstants
import com.scrapmetal.quackerama.hardware.QuackMotor
import com.scrapmetal.quackerama.hardware.QuackQuadrature
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
import org.firstinspires.ftc.teamcode.seventh.robot.hardware.Globals
import org.firstinspires.ftc.teamcode.seventh.robot.hardware.Robot
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.DroneSubsystem
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.LiftSubsystem
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.OutputSubsystem
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.RobotState
import kotlin.time.DurationUnit
import kotlin.time.TimeSource

@TeleOp(name = "DroneTest")
class DroneTest : LinearOpMode() {
    override fun runOpMode() {
        val gamepad = GamepadEx(gamepad1)
        val drone = DroneSubsystem(hardwareMap)
        var state = DroneSubsystem.DroneState.LODED

        Robot.write(drone)
        waitForStart()

        while (opModeIsActive()) {
            gamepad.readButtons()
            when {
                gamepad.wasJustPressed(GamepadKeys.Button.B) -> state = DroneSubsystem.DroneState.LODED
                gamepad.wasJustPressed(GamepadKeys.Button.A) -> state = DroneSubsystem.DroneState.DIPER
            }
            drone.update(state)
            Robot.write(drone)
            telemetry.addData("state", state)
            telemetry.update()
        }
    }
}
