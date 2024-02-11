package org.firstinspires.ftc.teamcode.seventh.robot.subsystem

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.HardwareMap
import com.scrapmetal.quackerama.control.Pose2d
import com.scrapmetal.quackerama.control.Rotation2d
import com.scrapmetal.quackerama.control.Vector2d
import com.scrapmetal.quackerama.control.controller.PDF
import com.scrapmetal.quackerama.control.controller.Utils
import com.scrapmetal.quackerama.hardware.QuackAnalog
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive
import java.lang.Math.toRadians
import kotlin.math.PI
import kotlin.math.absoluteValue
import kotlin.math.cos
import kotlin.math.sin

class DriveSubsystem(hardwareMap: HardwareMap) : Subsystem {
    var wallLeft = 0.0
        private set
    var wallRight = 0.0
        private set
    var wallDist = 0.0
        private set
    var correcting = false
    private var input = Pose2d()
    private var lastInput = input
    private val drive = SampleMecanumDrive(hardwareMap)
    private val distLeft = QuackAnalog(hardwareMap, "distLeft")
    private val distRight = QuackAnalog(hardwareMap, "distRight")
    private val headingPDF = PDF(
            p = 0.55,
            d = 0.0,
            f = { _: Double -> 0.0 },
            minPowerToMove = 0.2,
            deadzone = 0.04,
            maxMagnitude = 1.0,
            continuous = true,
    )

    override fun read() {
        drive.updatePoseEstimate()
        // this is okay to do every loop since we're bulkreading both hubs anyway
        // TODO: convert to inches
        wallLeft = distLeft.getRawVoltage()
        wallRight = distRight.getRawVoltage()
        wallDist = ((4.83055/wallLeft + 0.270722) + (5.18763/wallRight + 0.0107079))/2
        // old iffy regression: 4.6613/wallRight + 0.579777
    }

    fun update(input: Pose2d, correcting: Boolean, fieldOriented: Boolean, dt: Double, pid: Boolean = true) {
        this.input = input
        this.correcting = correcting
        if (fieldOriented) {
            this.input = Pose2d(Vector2d(
                    (input.position.v * sin(drive.poseEstimate.heading) + input.position.u * cos(drive.poseEstimate.heading)),
                    (input.position.v * cos(drive.poseEstimate.heading) - input.position.u * sin(drive.poseEstimate.heading))),
                    input.heading)
        }
        this.input = Pose2d(
            input.position.unit*Utils.correctWithMinPower(
                u0 = input.position.mag,
                // magic regression for measured minimum powers
                uMin = 0.1*sin(toRadians(2*input.position.polarAngle - 90)) + 0.3,
                deadzone = 0.05,
                max = 1.0
            ),
            if (pid) {
                Rotation2d(headingPDF.update(drive.poseEstimate.heading, input.heading.polarAngle, dt))
            } else {
                input.heading
            }
        )
    }

    override fun write() {
        // if (
        //     (lastInput.position.u - input.position.u).absoluteValue > 0.005 ||
        //     (lastInput.position.v - input.position.v).absoluteValue > 0.005 ||
        //     (lastInput.heading.polarAngle - input.heading.polarAngle).absoluteValue > 0.005
        // ) {
            drive.setWeightedDrivePower(com.acmerobotics.roadrunner.geometry.Pose2d(
                    input.position.u, input.position.v,
                    input.heading.polarAngle))
        // }
    }

    fun startIMUThread(opMode: LinearOpMode) {
        drive.startIMUThread(opMode)
    }

    fun setPoseEstimate(pose: Pose2d) {
        drive.poseEstimate = com.acmerobotics.roadrunner.geometry.Pose2d(
                pose.position.u,
                pose.position.v,
                pose.heading.polarAngle
        )
    }

    fun resetHeading() {
        drive.poseEstimate = com.acmerobotics.roadrunner.geometry.Pose2d(
                drive.poseEstimate.x,
                drive.poseEstimate.y,
                0.0
        )
    }

    fun getPoseEstimate(): Pose2d {
        return Pose2d(Vector2d(drive.poseEstimate.x, drive.poseEstimate.y), Rotation2d(drive.poseEstimate.heading))
    }

    // WARNING: this returns no movement if Roadrunner's velocity estimate is null
    fun getVelocity(): Pose2d {
        return drive.poseVelocity?.let {
            Pose2d(Vector2d(it.x, it.y) , Rotation2d(it.heading))
        } ?: Pose2d(Vector2d(), Rotation2d())
    }
}