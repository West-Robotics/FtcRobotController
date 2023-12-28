package org.firstinspires.ftc.teamcode.seventh.robot.subsystem

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
import kotlin.math.cos
import kotlin.math.sin

class DriveSubsystem(hardwareMap: HardwareMap) : Subsystem {
    var wallLeft = 0.0
        private set
    var wallRight = 0.0
        private set
    var correcting = false
    var input = Pose2d()
    val drive = SampleMecanumDrive(hardwareMap)
    val distLeft = QuackAnalog(hardwareMap, "distLeft")
    val distRight = QuackAnalog(hardwareMap, "distRight")
    val headingPDF = PDF(p = 0.9,
                         d = 1.0,
                         f = { _: Double -> 0.0 },
                         minPowerToMove = { _: Double -> 0.2 },
                         deadzone = 0.1,
                         maxMagnitude = 1.0,
                         continuous = true)

    override fun read() {
        drive.updatePoseEstimate()
        // this is okay to do every loop since we're bulkreading both hubs anyway
        // TODO: convert to inches
        wallLeft = distLeft.getRawVoltage()
        wallRight = distRight.getRawVoltage()
    }

    fun update(input: Pose2d, correcting: Boolean, fieldOriented: Boolean, dt: Double) {
        this.input = input
        this.correcting = correcting
        if (fieldOriented) {
            this.input = Pose2d(Vector2d(
                    (input.position.v * sin(drive.poseEstimate.heading) + input.position.u * cos(drive.poseEstimate.heading)),
                    (input.position.v * cos(drive.poseEstimate.heading) - input.position.u * sin(drive.poseEstimate.heading))),
                    input.heading)
        }
        this.input = Pose2d(input.position.unit*Utils.correctWithMinPower(
                    input.position.mag,
                    // mega magic, regression for measured minimum powers
                    { x: Double -> 0.1*sin(toRadians(2*PI*x/180 - 1.57)) + 0.2 },
                    deadzone = 0.05,
                    maxMagnitude = 1.0),
            Rotation2d(headingPDF.update(drive.poseEstimate.heading, input.heading.polarAngle, dt)))
    }

    override fun write() {
        drive.setWeightedDrivePower(com.acmerobotics.roadrunner.geometry.Pose2d(
                com.acmerobotics.roadrunner.geometry.Vector2d(input.position.u, input.position.v),
                input.heading.polarAngle))
    }

    fun setPoseEstimate(pose: Pose2d) {
        drive.poseEstimate = com.acmerobotics.roadrunner.geometry.Pose2d(
                pose.position.u,
                pose.position.v,
                pose.heading.polarAngle)
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