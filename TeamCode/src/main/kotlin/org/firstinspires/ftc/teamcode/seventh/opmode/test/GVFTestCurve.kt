package org.firstinspires.ftc.teamcode.seventh.opmode.test

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.acmerobotics.roadrunner.geometry.Pose2d as RRPose2d
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.scrapmetal.quackerama.control.Vector2d
import com.scrapmetal.quackerama.control.gvf.GG
import com.scrapmetal.quackerama.control.path.Path
import com.scrapmetal.quackerama.control.path.path
import org.firstinspires.ftc.teamcode.Datalogger
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.seventh.robot.hardware.Robot
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.DriveSubsystem
import java.lang.Math.toRadians
import java.text.SimpleDateFormat
import java.util.Date
import kotlin.math.cos
import kotlin.math.pow
import kotlin.math.sin
import kotlin.time.DurationUnit
import kotlin.time.TimeSource

@TeleOp(name = "GVFTestCurve")
class GVFTestCurve : LinearOpMode() {
    override fun runOpMode() {
        val path: Path = path {
            hermite {
                label("test")
                start { pos(0.0, 0.0); ang(toRadians(-90.0)); v(140.0) }
                end { pos(80.0, 0.0); ang(toRadians(90.0)); v(140.0) }
                constraints {
                    decelDist(12.0)
                    heading(0.0)
                }
            }
        }
        val gg = GG(0.5, 0.0, 1.0, path)
        Robot.hardwareMap = hardwareMap
        val allHubs = hardwareMap.getAll(LynxModule::class.java)
        for (hub in allHubs) {
            hub.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL
        }
        val drive = DriveSubsystem(hardwareMap)

        val primary = GamepadEx(gamepad1)

        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
        telemetry.update()
        waitForStart()

        while (opModeIsActive() && !isStopRequested) {
            for (hub in allHubs) {
                hub.clearBulkCache()
            }
            Robot.read(drive)

            val gvfState = gg.update(drive.getPoseEstimate().position)
            drive.update(input = gvfState,
                         correcting = false,
                         fieldOriented = true,
                         dt = Robot.dt)

            Robot.write(drive)
            telemetry.addData("dt", Robot.dt)
            telemetry.addData("x", drive.getPoseEstimate().position.u)
            telemetry.addData("y", drive.getPoseEstimate().position.v)
            telemetry.addData("heading", drive.getPoseEstimate().heading.polarAngle)
            telemetry.update()
        }
    }
}
