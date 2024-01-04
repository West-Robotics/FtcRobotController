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
import java.lang.Math.toRadians
import kotlin.time.DurationUnit

@Autonomous(name = "GVF Benchmark")
class GVFBenchmarkTest : LinearOpMode() {
    override fun runOpMode() {
        Robot.hardwareMap = hardwareMap
        val drive = DriveSubsystem(hardwareMap)
        val line = path {
            line {
                label("line")
                start(0.0, 0.0)
                end(40.0, 0.0)
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
        val gg = GG(0.6, 0.0004, line)
        val gamepad = GamepadEx(gamepad1)

        val allHubs = hardwareMap.getAll(LynxModule::class.java)
        for (hub in allHubs) {
            hub.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL
        }

        waitForStart()
        while (opModeIsActive() && !isStopRequested) {
            for (hub in allHubs) {
                hub.clearBulkCache()
            }
            Robot.read(drive)
            gamepad.readButtons()
            drive.update(input = gg.update(drive.getPoseEstimate().position, drive.getVelocity().position),
                         correcting = false,
                         fieldOriented = true,
                         dt = Robot.dt)
            if (gamepad.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                gg.currentIndex++
            } else if (gamepad.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
                gg.currentIndex--
            }
            Robot.write(drive)
            telemetry.addData("dt", Robot.dt)
            telemetry.addData("x", drive.getPoseEstimate().position.u)
            telemetry.addData("y", drive.getPoseEstimate().position.v)
            telemetry.addData("current index", gg.currentIndex)
            telemetry.update()
        }
    }
}
