package org.firstinspires.ftc.teamcode.seventh.robot.subsystem

import com.qualcomm.robotcore.hardware.HardwareMap
import com.scrapmetal.quackerama.control.Pose2d
import com.scrapmetal.quackerama.control.Rotation2d
import com.scrapmetal.quackerama.control.Vector2d
import com.scrapmetal.quackerama.control.controller.PDF
import com.scrapmetal.quackerama.control.controller.PDFState
import com.scrapmetal.quackerama.hardware.QuackAnalog
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.seventh.robot.hardware.Globals
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
    val headingPDF = PDF(0.9, 1.0, { x: Double -> 0.0 }, 1.0, continuous = true)
    var headingState = headingPDF.updateWithState(0.0, 0.0, 0.0)

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
                    Rotation2d(headingPDF.update(drive.poseEstimate.heading, input.heading.polarAngle, dt)))
        } else {
            // this.input = Pose2d(input.position, Rotation2d(headingPDF.update(drive.poseEstimate.heading, input.heading.polarAngle, dt)))
            headingState = headingPDF.updateWithState(drive.poseEstimate.heading, input.heading.polarAngle, dt)
            this.input = Pose2d(input.position, Rotation2d(headingPDF.update(drive.poseEstimate.heading, input.heading.polarAngle, dt)))
        }
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
}