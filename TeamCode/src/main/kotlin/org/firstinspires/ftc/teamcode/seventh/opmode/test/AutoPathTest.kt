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

@Autonomous(name = "Auto Path Test")
class AutoPathTest : LinearOpMode() {
    override fun runOpMode() {
        Robot.hardwareMap = hardwareMap
        val drive = DriveSubsystem(hardwareMap)
        // red close side pathing
        val yellow = path {
            line {
                label("start to backdrop")
                // 17.5 length
                start(12.0, -59.25)
                end(50.0, -36.0)
                constraints {
                    decelDist(24.0)
                    heading(toRadians(0.0)) }}}
        val purple = path {
            line {
                label("backdrop to spike marks")
                start(yellow.paths[0].endPose.position)
                end(12.0, -36.0)
                constraints {
                    decelDist(32.0)
                    heading(toRadians(0.0)) }}}
        val collect = path {
            hermite {
                label("spike marks to lane 3")
                start { pos(12.0, -36.0); ang(toRadians(50.0));  v(30.0) }
                end   { pos(12.0, -16.0); ang(toRadians(160.0)); v(22.0) }
                constraints { heading(toRadians(0.0)) }}
            hermite {
                label("lane 3 to pixels")
                start { pos(12.0, -16.0); ang(toRadians(160.0));  v(22.0) }
                end   { pos(-24.0, -16.0); ang(toRadians(180.0)); v(12.0) }
                constraints {
                    decelDist(18.0)
                    heading(toRadians(0.0)) }}}
        val score = path {
            hermite {
                label("pixels to the back")
                start { pos(-24.0, -16.0); ang(toRadians(0.0));  v(12.0) }
                end   { pos(23.0, -16.0); ang(toRadians(-18.0)); v(53.0) }
                constraints { heading(toRadians(0.0)) }}
            hermite {
                label("the back to backdrop")
                start { pos(23.0, -14.0); ang(toRadians(-18.0));  v(53.0) }
                end   { pos(50.0, -36.0); ang(toRadians(-45.0)); v(27.0) }
                constraints {
                    decelDist(18.0)
                    heading(toRadians(0.0)) }}}
        val gg = GG(0.4, yellow, purple, collect, score)
        drive.setPoseEstimate(Pose2d(yellow.paths[0].startPose.position, Rotation2d(toRadians(90.0))))
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

            drive.update(input = gg.update(drive.getPoseEstimate().position),
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