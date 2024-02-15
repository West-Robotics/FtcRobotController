package org.firstinspires.ftc.teamcode.seventh.opmode.test

import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.scrapmetal.quackerama.control.Pose2d
import com.scrapmetal.quackerama.control.Rotation2d
import com.scrapmetal.quackerama.control.gvf.GG
import com.scrapmetal.quackerama.control.path.path
import org.firstinspires.ftc.teamcode.seventh.robot.hardware.Robot
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.DriveSubsystem
import java.lang.Math.toDegrees
import java.lang.Math.toRadians
import kotlin.time.DurationUnit

@Autonomous(name = "GVF Benchmark")
class GVFBenchmarkTest : LinearOpMode() {
    override fun runOpMode() {
        Robot.hardwareMap = hardwareMap
        val drive = DriveSubsystem(hardwareMap)
        val line = path { line { label("line")
                start(0.0, 0.0)
                end(80.0, 0.0)
                constraints {
                    decelDist(12.0)
                    heading(toRadians(0.0)) }}}
        val hermite = path {
            hermite {
                label("pixels to the back")
                start { pos(0.0, 0.0); ang(toRadians(0.0));  v(12.0) }
                end   { pos(40.0, 0.0); ang(toRadians(45.0)); v(50.0) }
                constraints {
                    decelDist(12.0)
                    heading(toRadians(0.0)) }}}
        val gg = GG(0.9, 0.007, 1.0, line)
        val gamepad = GamepadEx(gamepad1)

        val allHubs = hardwareMap.getAll(LynxModule::class.java)
        for (hub in allHubs) {
            hub.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL
        }
        drive.startIMUThread(this)
        while (opModeInInit()) {
            telemetry.addData("dt", Robot.dt)
            telemetry.addData("x", drive.getPoseEstimate().position.u)
            telemetry.addData("y", drive.getPoseEstimate().position.v)
            telemetry.addData("current index", gg.currentIndex)
            telemetry.addData("x", gg.update(drive.getPoseEstimate().position, drive.getVelocity().position).position.u)
            telemetry.addData("y", gg.update(drive.getPoseEstimate().position, drive.getVelocity().position).position.v)
            telemetry.update()
        }
        while (opModeIsActive()) {
            for (hub in allHubs) {
                hub.clearBulkCache()
            }
            Robot.dtUpdate()
            Robot.read(drive)
            gamepad.readButtons()
            drive.update(
                input = gg.update(drive.getPoseEstimate().position, drive.getVelocity().position),
                correcting = false,
                fieldOriented = true,
                dt = Robot.dt,
                pid = true,
            )
            if (gamepad.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                gg.currentIndex++
            } else if (gamepad.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
                gg.currentIndex--
            }
            Robot.write(drive)
            telemetry.addData("dt", Robot.dt)
            telemetry.addData("x", drive.getPoseEstimate().position.u)
            telemetry.addData("y", drive.getPoseEstimate().position.v)
            telemetry.addData("heading", toDegrees(drive.getPoseEstimate().heading.polarAngle))
            telemetry.addData("current index", gg.currentIndex)
            telemetry.addData("x power", gg.update(drive.getPoseEstimate().position, drive.getVelocity().position).position.u)
            telemetry.addData("y power", gg.update(drive.getPoseEstimate().position, drive.getVelocity().position).position.v)
            telemetry.addData("turn power", toDegrees(gg.update(drive.getPoseEstimate().position, drive.getVelocity().position).heading.polarAngle))
            telemetry.update()
        }
    }
}
