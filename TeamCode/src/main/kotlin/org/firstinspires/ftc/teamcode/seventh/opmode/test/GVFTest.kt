package org.firstinspires.ftc.teamcode.seventh.opmode.test

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.acmerobotics.roadrunner.geometry.Pose2d as RRPose2d
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.scrapmetal.quackerama.control.Rotation2d
import com.scrapmetal.quackerama.control.Vector2d
import com.scrapmetal.quackerama.control.Pose2d
import com.scrapmetal.quackerama.control.gvf.GVF
import com.scrapmetal.quackerama.control.path.CubicHermite
import com.scrapmetal.quackerama.control.path.Path
import com.scrapmetal.quackerama.control.path.PathSegment
import com.scrapmetal.quackerama.control.path.path
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.seventh.robot.hardware.Robot
import kotlin.math.cos
import kotlin.math.sin
import kotlin.time.DurationUnit
import kotlin.time.TimeSource

@TeleOp(name = "GVFTest")
class GVFTest : LinearOpMode() {
    override fun runOpMode() {
        val path: Path = path {
            hermite {
                label("test")
                start { pos(0.0, 0.0); ang(0.0); v(40.0) }
                end { pos(20.0, 20.0); ang(0.0); v(40.0) }
            }
        }
        // val path: Path = path {
        //     hermite {
        //         label("test")
        //         start { pos(0.0, 0.0); ang(0.0); v(1.0) }
        //         end { pos(100.0, 00.0); ang(0.0); v(1.0) }
        //     }
        // }
        // val path: Path = path {
        //     hermite {
        //         label("test")
        //         start { pos(0.0, 0.0); ang(0.0); v(180.0) }
        //         end { pos(00.0, 80.0); ang(180.0); v(180.0) }
        //     }
        // }
        // val path: Path = path {
        //     hermite {
        //         label("test")
        //         start { pos(0.0, 0.0); ang(0.0); v(500.0) }
        //         end { pos(60.0, 80.0); ang(0.0); v(500.0) }
        //     }
        // }
        val gvf = GVF(path, 0.2)
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
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        telemetry.addData("closest t", gvf.getClosestT(Vector2d(drive.poseEstimate.x, drive.poseEstimate.y)))
        telemetry.addData("x", drive.poseEstimate.x)
        telemetry.addData("y", drive.poseEstimate.y)
        telemetry.addData("heading", drive.poseEstimate.heading)
        telemetry.update()
        waitForStart()

        while (opModeIsActive() && !isStopRequested) {
            for (hub in allHubs) {
                hub.clearBulkCache()
            }
            val loop = timeSource.markNow()
            val dt = (loop - loopTime).toDouble(DurationUnit.MILLISECONDS)
            loopTime = loop

            drive.update()
            // drive.setWeightedDrivePower(RRPose2d(primary.leftY, -primary.leftX, -primary.rightX))

            val m_d = gvf.getVector(Vector2d(drive.poseEstimate.x, drive.poseEstimate.y))
            val rrPose = RRPose2d(m_d.u, m_d.v, 0.0)
            // val rrPose = RRPose2d(primary.leftY, -primary.leftX, -primary.rightX)
            val heading = drive.poseEstimate.heading
            val distToEnd = path.paths[0].endPose.position.distanceTo(Vector2d(drive.poseEstimate.x, drive.poseEstimate.y))/24
            val multiplier = if (distToEnd > 1.0) 1.0 else distToEnd
            val correctedRRPose = RRPose2d((rrPose.y*sin(heading) + rrPose.x*cos(heading))*multiplier*0.8,
                    (rrPose.y*cos(heading) - rrPose.x*sin(heading))*multiplier*1.0,
                    0.0)
            drive.setWeightedDrivePower(correctedRRPose)

            telemetry.addData("hz", 1000 / dt)
            telemetry.addData("m_d angle", Math.toDegrees(m_d.polarAngle))
            telemetry.addData("m_d mag", m_d.mag)
            telemetry.addData("closest t", gvf.getClosestT(Vector2d(drive.poseEstimate.x, drive.poseEstimate.y)))
            telemetry.addData("x", drive.poseEstimate.x)
            telemetry.addData("y", drive.poseEstimate.y)
            telemetry.addData("heading", drive.poseEstimate.heading)
            telemetry.addData("corrected x", correctedRRPose.x)
            telemetry.addData("corrected y", correctedRRPose.y)
            telemetry.update()
        }
    }
}