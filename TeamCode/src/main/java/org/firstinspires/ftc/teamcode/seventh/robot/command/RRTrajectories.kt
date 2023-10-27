package org.firstinspires.ftc.teamcode.seventh.robot.command

import com.acmerobotics.roadrunner.geometry.Pose2d
import org.firstinspires.ftc.teamcode.drive.DriveConstants
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.seventh.robot.hardware.Globals
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.PropPositionProcessor
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder

class RRTrajectories(drive: SampleMecanumDrive,
                     side: Globals.Side,
                     start: Globals.Start,
                     lane: Globals.Lane,
                     prop: PropPositionProcessor.PropPosition) {
    val dropOff: TrajectorySequence
    val score: TrajectorySequence
    val park: TrajectorySequence
    val startPose: Pose2d

    init {
        when {
            side == Globals.Side.RED && start == Globals.Start.CLOSE ->     startPose = Pose2d( 36.0, -65.6,-90.0)
            side == Globals.Side.RED && start == Globals.Start.FAR ->       startPose = Pose2d(-36.0, -65.6,-90.0)
            side == Globals.Side.BLUE && start == Globals.Start.CLOSE ->    startPose = Pose2d( 36.0,  65.6, 90.0)
            side == Globals.Side.BLUE && start == Globals.Start.FAR ->      startPose = Pose2d(-36.0,  65.6, 90.0)
            else -> startPose = Pose2d(36.0, -65.6, -90.0)
        }
        drive.poseEstimate = startPose
        dropOff = with (drive.trajectorySequenceBuilder(startPose)) {
            when (prop) {
                PropPositionProcessor.PropPosition.LEFT -> {
                    this.back(24.0, SampleMecanumDrive.getVelocityConstraint(10.0, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(10.0))
                        .turn(-20.0)
                }
                PropPositionProcessor.PropPosition.MIDDLE -> {
                    this.back(28.0, SampleMecanumDrive.getVelocityConstraint(10.0, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(10.0))
                }
                PropPositionProcessor.PropPosition.RIGHT -> {
                    this.back(24.0, SampleMecanumDrive.getVelocityConstraint(10.0, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(10.0))
                        .turn(20.0)
                }
            }
            this.build()
        }
        score = with (drive.trajectorySequenceBuilder(dropOff.end())) {
            when (prop) {
                PropPositionProcessor.PropPosition.LEFT -> {
                    this.turn(20.0)
                        .forward(20.0, SampleMecanumDrive.getVelocityConstraint(10.0, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(10.0))
                }
                PropPositionProcessor.PropPosition.MIDDLE -> {
                    this.forward(24.0, SampleMecanumDrive.getVelocityConstraint(10.0, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(10.0))
                }
                PropPositionProcessor.PropPosition.RIGHT -> {
                    this.turn(-20.0)
                        .forward(20.0, SampleMecanumDrive.getVelocityConstraint(10.0, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(10.0))
                }
            }
            when (side) {
                Globals.Side.RED -> 1.0
                Globals.Side.BLUE -> -1.0
            }.let {
                when (lane) {
                    Globals.Lane.LANE_1 -> {
                        this.turn(-it*90)
                    }
                    Globals.Lane.LANE_2 -> {
                        TODO("lane 2 not existing")
                    }
                    Globals.Lane.LANE_3 -> {
                        this.forward(52.0)
                            .turn(-it*90)
                    }
                }
                when (start) {
                    Globals.Start.CLOSE -> {
                        this.forward(36.0)
                            .turn(it*90.0)
                            .forward(24.0)
                            .turn(-it*90.0)
                            .forward(4.0)
                    }
                    Globals.Start.FAR -> {
                        this.forward(78.0)
                            .turn(-it*90.0)
                            .forward(24.0)
                            .turn(it*90.0)
                            .forward(4.0)
                    }
                }
            }
            this.build()
        }
        park = with (drive.trajectorySequenceBuilder(score.end())) {
            when (side) {
                Globals.Side.RED -> 1.0
                Globals.Side.BLUE -> -1.0
            }.let {
                when (lane) {
                    Globals.Lane.LANE_1 -> {
                        this.back(6.0)
                            .strafeRight(it*24.0)
                    }
                    Globals.Lane.LANE_2 -> {
                        TODO("lane 2 not existing")
                    }
                    Globals.Lane.LANE_3 -> {
                        this.back(6.0)
                            .strafeLeft(it*24.0)
                    }
                }
            }
            this.build()
        }
    }
}