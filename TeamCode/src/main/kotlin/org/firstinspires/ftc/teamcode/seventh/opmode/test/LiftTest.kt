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
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.LiftSubsystem
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.OutputSubsystem
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.RobotState
import kotlin.time.DurationUnit
import kotlin.time.TimeSource

@TeleOp(name = "LiftTest")
class LiftTest : LinearOpMode() {
    override fun runOpMode() {
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
        var height = 0

        Robot.read(output)
        output.update(RobotState.BACKDROP, -120.5)
        Robot.write(output)
        waitForStart();

        val timeSource = TimeSource.Monotonic
        while (opModeIsActive() && !isStopRequested) {
            CONTROL_HUB.clearBulkCache()
            val t1b = timeSource.markNow()
            gamepad.readButtons()
            Robot.dtUpdate()
            when {
                gamepad.wasJustPressed(GamepadKeys.Button.DPAD_UP) -> height++
                gamepad.wasJustPressed(GamepadKeys.Button.DPAD_DOWN) -> height--
            }
            val t1e = timeSource.markNow()
            val t2b = timeSource.markNow()
            Robot.read(lift)
            val t2e = timeSource.markNow()
            val t3b = timeSource.markNow()
            lift.update(Globals.LIFT_HEIGHTS[height], Robot.dt)
            val t3e = timeSource.markNow()
            val t4b = timeSource.markNow()
            Robot.write(lift)
            val t4e = timeSource.markNow()
            telemetry.addData("target", lift.state.commandedExtension)
            telemetry.addData("pos", lift.state.extension)
            telemetry.addData("power", lift.state.power)
            telemetry.addData("current", lift.state.current)
            telemetry.addData("hz", 1000/ Robot.dt)
            telemetry.addData("t1", (t1e-t1b).toDouble(DurationUnit.MILLISECONDS))
            telemetry.addData("t2", (t2e-t2b).toDouble(DurationUnit.MILLISECONDS))
            telemetry.addData("t3", (t3e-t3b).toDouble(DurationUnit.MILLISECONDS))
            telemetry.addData("t4", (t4e-t4b).toDouble(DurationUnit.MILLISECONDS))
            telemetry.update();
        }
    }
}
