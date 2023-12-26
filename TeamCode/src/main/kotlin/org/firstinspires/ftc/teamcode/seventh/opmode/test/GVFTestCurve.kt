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
            }
        }
        val GG = GG(path, 0.5)
        Robot.hardwareMap = hardwareMap
        val allHubs = hardwareMap.getAll(LynxModule::class.java)
        for (hub in allHubs) {
            hub.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL
        }
        val drive = SampleMecanumDrive(hardwareMap)

        val timeSource = TimeSource.Monotonic
        var loopTime: TimeSource.Monotonic.ValueTimeMark = timeSource.markNow()

        drive.poseEstimate = RRPose2d(0.0, 0.0, 0.0)
        val primary = GamepadEx(gamepad1)
        var netError = 0.0
        var netSquaredError = 0.0
        val fields = listOf(Datalogger.LoggableField("error"),
                Datalogger.LoggableField("net error"),
                Datalogger.LoggableField("net sq error"),
                Datalogger.LoggableField("hz"))
        val datalog = Datalogger.Builder()
                .setFilename(SimpleDateFormat("yyyy-MM-dd-HH-mm-ss").format(Date()))
                .setAutoTimestamp(Datalogger.AutoTimestamp.DECIMAL_SECONDS)
                .setFields(fields)
                .build();

        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
        // telemetry.addData("closest t", gvf.getClosestT(Vector2d(drive.poseEstimate.x, drive.poseEstimate.y)))
        telemetry.addData("x", drive.poseEstimate.x)
        telemetry.addData("y", drive.poseEstimate.y)
        telemetry.addData("heading", drive.poseEstimate.heading)
        telemetry.update()
        waitForStart()

        datalog.writeLine();
        while (opModeIsActive() && !isStopRequested) {
            for (hub in allHubs) {
                hub.clearBulkCache()
            }
            val loop = timeSource.markNow()
            val dt = (loop - loopTime).toDouble(DurationUnit.MILLISECONDS)
            loopTime = loop

            drive.update()
            // drive.setWeightedDrivePower(RRPose2d(primary.leftY, -primary.leftX, -primary.rightX))

            val gvfState = GG.update(Vector2d(drive.poseEstimate.x, drive.poseEstimate.y))
            val rrPose = RRPose2d(gvfState.m_d.u, gvfState.m_d.v, 0.0)
            // val rrPose = RRPose2d(primary.leftY, -primary.leftX, -primary.rightX)
            val heading = drive.poseEstimate.heading
            val distToEnd = path.paths[0].endPose.position.distanceTo(Vector2d(drive.poseEstimate.x, drive.poseEstimate.y))/12
            val multiplier = if (distToEnd > 1.0) 1.0 else distToEnd
            val correctedRRPose = RRPose2d((rrPose.y*sin(heading) + rrPose.x*cos(heading))*multiplier*0.8,
                    (rrPose.y*cos(heading) - rrPose.x*sin(heading))*multiplier*1.0,
                    0.0)
            drive.setWeightedDrivePower(correctedRRPose)

            netError += gvfState.error.mag
            netSquaredError += gvfState.error.mag.pow(2)
            datalog.fields[1].set(gvfState.error.mag)
            datalog.fields[2].set(netError)
            datalog.fields[3].set(netSquaredError)
            datalog.fields[4].set(1000 / dt)
            datalog.writeLine()

            telemetry.addData("hz", 1000 / dt)
            // telemetry.addData("m_d angle", Math.toDegrees(m_d.polarAngle))
            // telemetry.addData("m_d mag", gvfState.m_d.mag)
            // telemetry.addData("closest t", gvf.getClosestT(Vector2d(drive.poseEstimate.x, drive.poseEstimate.y)))
            telemetry.addData("x", drive.poseEstimate.x)
            telemetry.addData("y", drive.poseEstimate.y)
            telemetry.addData("heading", drive.poseEstimate.heading)
            telemetry.addData("corrected x", correctedRRPose.x)
            telemetry.addData("corrected y", correctedRRPose.y)
            telemetry.update()
        }
    }
}
