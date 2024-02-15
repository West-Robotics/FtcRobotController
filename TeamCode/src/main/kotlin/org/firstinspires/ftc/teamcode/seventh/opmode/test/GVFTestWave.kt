package org.firstinspires.ftc.teamcode.seventh.opmode.test

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.acmerobotics.roadrunner.geometry.Pose2d as RRPose2d
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.scrapmetal.quackerama.control.Pose2d
import com.scrapmetal.quackerama.control.Rotation2d
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

@TeleOp(name = "GVFTestWave")
class GVFTestWave : LinearOpMode() {
    override fun runOpMode() {
        val path: Path = path {
            // hermite {
            //     label("test")
            //     start { pos(0.0, 0.0); ang(toRadians(00.0)); v(20.0) }
            //     end { pos(40.0, -20.0); ang(toRadians(00.0)); v(20.0) }
            //     constraints {
            //         decelDist(12.0)
            //         heading(0.0)
            //     }
            // }
            // line {
            //     label("test")
            //     start(0.0, 0.0)
            //     end(40.0, -20.0)
            //     constraints {
            //         decelDist(12.0)
            //         heading(0.0)
            //     }
            // }
            line {
                label("start to backdrop")
                // 17.5 length
                start(12.0, -63.25)
                end(50.0, -36.0)
                constraints {
                    decelDist(24.0)
                    heading(toRadians(0.0)) }}
        }
        val gg = GG(0.4, 0.0, 1.0, path)
        Robot.hardwareMap = hardwareMap
        val allHubs = hardwareMap.getAll(LynxModule::class.java)
        for (hub in allHubs) {
            hub.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL
        }
        val drive = DriveSubsystem(hardwareMap)
        drive.setPoseEstimate(Pose2d(path.paths[0].startPose.position, Rotation2d(toRadians(90.0))))

        val timeSource = TimeSource.Monotonic
        var loopTime: TimeSource.Monotonic.ValueTimeMark = timeSource.markNow()

        val primary = GamepadEx(gamepad1)
        var netError = 0.0
        var netSquaredError = 0.0
        // val fields = listOf(Datalogger.LoggableField("error"),
        //         Datalogger.LoggableField("net error"),
        //         Datalogger.LoggableField("net sq error"),
        //         Datalogger.LoggableField("hz"))
        // val datalog = Datalogger.Builder()
        //         .setFilename(SimpleDateFormat("yyyy-MM-dd-HH-mm-ss").format(Date()))
        //         .setAutoTimestamp(Datalogger.AutoTimestamp.DECIMAL_SECONDS)
        //         .setFields(fields)
        //         .build();

        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
        // telemetry.addData("closest t", gvf.getClosestT(Vector2d(drive.poseEstimate.x, drive.poseEstimate.y)))
        telemetry.update()
        waitForStart()

        // datalog.writeLine();
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
            // val distToEnd = path.paths[0].endPose.position.distanceTo(Vector2d(drive.poseEstimate.x, drive.poseEstimate.y))/24
            // val multiplier = if (distToEnd > 1.0) 1.0 else distToEnd
            // val correctedRRPose = RRPose2d((rrPose.y*sin(heading) + rrPose.x*cos(heading))*multiplier*0.8,
            //         (rrPose.y*cos(heading) - rrPose.x*sin(heading))*multiplier*1.0,
            //         0.0)
            // drive.setWeightedDrivePower(correctedRRPose)

            // netError += gvfState.error.mag
            // netSquaredError += gvfState.error.mag.pow(2)
            // datalog.fields[1].set(gvfState.error.mag)
            // datalog.fields[2].set(netError)
            // datalog.fields[3].set(netSquaredError)
            // datalog.fields[4].set(1000 / dt)
            // datalog.writeLine()

            telemetry.addData("dt", Robot.dt)
            // telemetry.addData("m_d angle", Math.toDegrees(m_d.polarAngle))
            // telemetry.addData("m_d mag", m_d.mag)
            // telemetry.addData("closest t", gvf.getClosestT(Vector2d(drive.poseEstimate.x, drive.poseEstimate.y)))
            telemetry.addData("x", drive.getPoseEstimate().position.u)
            telemetry.addData("y", drive.getPoseEstimate().position.v)
            telemetry.update()
        }
    }
}
