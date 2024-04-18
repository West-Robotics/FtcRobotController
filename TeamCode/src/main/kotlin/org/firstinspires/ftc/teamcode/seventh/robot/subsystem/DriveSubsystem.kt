package org.firstinspires.ftc.teamcode.seventh.robot.subsystem

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.HardwareMap
import com.scrapmetal.quackerama.control.Pose2d
import com.scrapmetal.quackerama.control.Rotation2d
import com.scrapmetal.quackerama.control.Vector2d
import com.scrapmetal.quackerama.control.controller.PIDF
import com.scrapmetal.quackerama.control.controller.Utils
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.seventh.robot.hardware.Robot
import java.lang.Math.toRadians
import kotlin.math.PI
import kotlin.math.sin

class DriveSubsystem(hardwareMap: HardwareMap) : Subsystem {
    private var input = Pose2d()
    val positionPIDF = PIDF(
        p = 0.07,
        i = 0.0,
        d = 0.02,
        f = { _: Double -> 0.0 },
        minPowerToMove = 0.0,
        // minPowerToMove = 0.1*sin(toRadians(2*input.position.polarAngle - 90)) + 0.2,
        deadzone = 0.01,
        maxMagnitude = 1.0,
        continuous = false,
        iZone = 0.05,
        maxIOutput = 0.2,
    )
    private val headingPIDF = PIDF(
        p = 0.55,
        i = 0.0,
        d = 0.0,
        f = { _: Double -> 0.0 },
        minPowerToMove = 0.1,
        deadzone = 0.01,
        maxMagnitude = 1.0,
        continuous = true,
        iZone = 0.05,
        maxIOutput = 0.2,
    )
    private val drive = SampleMecanumDrive(hardwareMap)
    // private val drive = MecanumDrive(hardwareMap, com.acmerobotics.roadrunner.Pose2d(0.0, 0.0, 0.0))

    override fun read() { drive.updatePoseEstimate() }

    /**
     * Update drivetrain powers open loop, minimum power correction included
     *
     * (technically only translationally)
     */
    fun updateOpen(ref: Pose2d, fieldOriented: Boolean, headingPID: Boolean) {
        input = if (!fieldOriented) ref else {
            Pose2d(getPoseEstimate().heading.inverse*ref.position, ref.heading)
        }
        input = Pose2d(
            // WARNING: how did this ever work in field-oriented with unrotated input???
            input.position.unit*Utils.correctWithMinPower(
                u0 = input.position.mag,
                // magic regression for measured minimum powers
                uMin = if (!fieldOriented) {
                    0.1*sin(2*input.position.polarAngle - PI/2) + 0.15
                } else {
                    0.1*sin(2*(input.position.polarAngle-getPoseEstimate().heading.theta) - PI/2) + 0.15
               },
                deadzone = 0.03,
                max = 1.0
            ),
            if (headingPID) {
                Rotation2d(headingPIDF.update(
                    getPoseEstimate().heading.theta,
                    input.heading.theta,
                    Robot.getDt()
                ))
            } else {
                input.heading
            }
        )
    }

    override fun write() {
        // uhhh this did not work well
        // if (
        //     (lastInput.position.u - input.position.u).absoluteValue > 0.005 ||
        //     (lastInput.position.v - input.position.v).absoluteValue > 0.005 ||
        //     (lastInput.heading.polarAngle - input.heading.polarAngle).absoluteValue > 0.005
        // ) {
        drive.setWeightedDrivePower(com.acmerobotics.roadrunner.geometry.Pose2d(
                input.position.x, input.position.y,
                input.heading.theta))
        //! drive.setDrivePowers(com.acmerobotics.roadrunner.PoseVelocity2d(
        //!         com.acmerobotics.roadrunner.Vector2d(input.position.x, input.position.y),
        //!         input.heading.theta)
        //! )
        // }
    }

    fun startIMUThread(opMode: LinearOpMode) { drive.startIMUThread(opMode) }

    fun setPoseEstimate(pose: Pose2d) {
        drive.poseEstimate = com.acmerobotics.roadrunner.geometry.Pose2d(
            pose.position.x,
            pose.position.y,
            pose.heading.theta
        )
    }

    fun resetHeading() {
        with (drive.poseEstimate) {
            drive.poseEstimate = com.acmerobotics.roadrunner.geometry.Pose2d(
                x,
                y,
                0.0,
            )
        }
    }

    fun getPoseEstimate(): Pose2d =
        with (drive.poseEstimate) {
            Pose2d(x, y, heading)
        }

    // WARNING: this returns no movement if Roadrunner's velocity estimate is null
    fun getVelocity(): Pose2d =
        drive.poseVelocity?.let {
            Pose2d(Vector2d(it.x, it.y) , Rotation2d(it.heading))
        } ?: Pose2d(Vector2d(), Rotation2d())
}