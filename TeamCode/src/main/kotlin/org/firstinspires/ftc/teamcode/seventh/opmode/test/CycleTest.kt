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
import org.firstinspires.ftc.teamcode.seventh.robot.command.CycleCommand
import org.firstinspires.ftc.teamcode.seventh.robot.hardware.Globals
import org.firstinspires.ftc.teamcode.seventh.robot.hardware.Robot
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.IntakeSubsystem
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.LiftSubsystem
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.OutputSubsystem
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.RobotState
import kotlin.time.DurationUnit
import kotlin.time.TimeSource

@TeleOp(name = "CycleTest")
class CycleTest : LinearOpMode() {
    override fun runOpMode() {
        Robot.hardwareMap = hardwareMap
        val allHubs = hardwareMap.getAll(LynxModule::class.java)
        var CONTROL_HUB: LynxModule = allHubs[0]
        for (hub in allHubs) {
            hub.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL
            if (hub.isParent && LynxConstants.isEmbeddedSerialNumber(hub.serialNumber)) {
                CONTROL_HUB = hub
            }
        }
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
        val gamepad = GamepadEx(gamepad1)
        val lift = LiftSubsystem(hardwareMap)
        val output = OutputSubsystem(hardwareMap)
        val intake = IntakeSubsystem(hardwareMap)
        val cycle = CycleCommand(intake, lift, output)
        var height = 0

        Robot.read(intake, lift, output)
        cycle.update(RobotState.BACKDROP, height, 0.0)
        Robot.write(intake, lift, output)
        waitForStart()

        while (opModeIsActive() && !isStopRequested) {
            CONTROL_HUB.clearBulkCache()
            gamepad.readButtons()
            Robot.dtUpdate()
            when {
                gamepad.wasJustPressed(GamepadKeys.Button.DPAD_UP) -> height++
                gamepad.wasJustPressed(GamepadKeys.Button.DPAD_DOWN) -> height--
            }
            Robot.read(lift)
            cycle.update(RobotState.BACKDROP, height, 0.0)
            Robot.write(lift)
            telemetry.addData("target", lift.state.commandedExtension)
            telemetry.addData("pos", lift.state.extension)
            telemetry.update();
        }
    }
}
