package org.firstinspires.ftc.teamcode.seventh.robot.command

import com.acmerobotics.roadrunner.geometry.Pose2d
import org.firstinspires.ftc.teamcode.drive.DriveConstants
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.seventh.robot.hardware.Globals
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.GetPropPositionPipeline
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence
import java.lang.Math.toRadians

class RRTrajectories(drive: SampleMecanumDrive,
                     side: Globals.Side,
                     start: Globals.Start,
                     lane: Globals.Lane,
                     val prop: GetPropPositionPipeline.PropPosition) {
    val collect: TrajectorySequence
    val dropOff: TrajectorySequence
    val score: TrajectorySequence
    val park: TrajectorySequence
    val startPose: Pose2d
    val multiplier: Double

    init {
        multiplier = if (side == Globals.Side.RED) 1.0 else -1.0
        when {
            // TODO: y spacing needs to change
            side == Globals.Side.RED && start == Globals.Start.CLOSE
                -> startPose = Pose2d( 36.0, -65.6, toRadians(90.0))
            side == Globals.Side.RED && start == Globals.Start.FAR
                -> startPose = Pose2d(-36.0, -65.6, toRadians(90.0))
            side == Globals.Side.BLUE && start == Globals.Start.CLOSE
                -> startPose = Pose2d( 36.0,  65.6, toRadians(-90.0))
            side == Globals.Side.BLUE && start == Globals.Start.FAR
                -> startPose = Pose2d(-36.0,  65.6, toRadians(-90.0))
            else -> startPose = Pose2d(36.0, -65.6, toRadians(-90.0))
        }
        drive.poseEstimate = startPose
        collect = drive.trajectorySequenceBuilder(startPose)
                .forward(3.0, SampleMecanumDrive.getVelocityConstraint(20.0, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build()
        dropOff = with (drive.trajectorySequenceBuilder(collect.end())) {
            if (side == Globals.Side.RED) {
                when (prop) {
                    GetPropPositionPipeline.PropPosition.LEFT -> {
                        this.strafeRight(10.0)
                        this.lineToLinearHeading(Pose2d(36.0, -36.0, toRadians(0.0)))
                                .back(1.0)
                    }
                    GetPropPositionPipeline.PropPosition.MIDDLE -> {
                        this.strafeRight(10.0)
                        this.lineToLinearHeading(Pose2d(36.0, -39.0, toRadians(-90.0)))
                    }
                    GetPropPositionPipeline.PropPosition.RIGHT -> {
                        this.lineToLinearHeading(Pose2d(57.0, -33.0, toRadians(0.0)))
                    }
                }
            } else {
                when (prop) {
                    GetPropPositionPipeline.PropPosition.LEFT -> {
                        this.forward(3.0, SampleMecanumDrive.getVelocityConstraint(20.0, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                            .lineToLinearHeading(Pose2d(57.0, 33.0, toRadians(0.0)))
                    }
                    GetPropPositionPipeline.PropPosition.MIDDLE -> {
                        this.forward(3.0, SampleMecanumDrive.getVelocityConstraint(20.0, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                        this.strafeLeft(10.0)
                            .lineToLinearHeading(Pose2d(36.0, 39.0, toRadians(90.0)))
                    }
                    GetPropPositionPipeline.PropPosition.RIGHT -> {
                        this.forward(3.0, SampleMecanumDrive.getVelocityConstraint(20.0, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                        this.strafeLeft(10.0)
                        this.lineToLinearHeading(Pose2d(36.0, 36.0, toRadians(0.0)))
                        this.back(1.0)
                    }
                }
            }
            this.build()
        }
        score = with (drive.trajectorySequenceBuilder(dropOff.end())) {
            if (side == Globals.Side.RED) {
                when (prop) {
                    GetPropPositionPipeline.PropPosition.LEFT -> {
                        this.lineToLinearHeading(Pose2d(72.5, -35.0, toRadians(0.0)),
                                SampleMecanumDrive.getVelocityConstraint(20.0, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    }
                    GetPropPositionPipeline.PropPosition.MIDDLE -> {
                        this.lineToLinearHeading(Pose2d(72.5, -40.0, toRadians(0.0)),
                                SampleMecanumDrive.getVelocityConstraint(20.0, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    }
                    GetPropPositionPipeline.PropPosition.RIGHT -> {
                        this.lineToLinearHeading(Pose2d(72.5, -48.0, toRadians(0.0)),
                                SampleMecanumDrive.getVelocityConstraint(20.0, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    }
                }
            } else {
                when (prop) {
                    GetPropPositionPipeline.PropPosition.LEFT -> {
                        this.lineToLinearHeading(Pose2d(72.5, 48.0, toRadians(0.0)),
                                SampleMecanumDrive.getVelocityConstraint(20.0, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    }
                    GetPropPositionPipeline.PropPosition.MIDDLE -> {
                        this.lineToLinearHeading(Pose2d(72.5, 40.0, toRadians(0.0)),
                                SampleMecanumDrive.getVelocityConstraint(20.0, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    }
                    GetPropPositionPipeline.PropPosition.RIGHT -> {
                        this.lineToLinearHeading(Pose2d(72.5, 35.0, toRadians(0.0)),
                                SampleMecanumDrive.getVelocityConstraint(20.0, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    }
                }
            }
            this.build()
        }
        park = with (drive.trajectorySequenceBuilder(score.end())) {
            this.back(4.0)
                .strafeRight(multiplier*28.0)
                .forward(5.0)
            this.build()
        }
    }
}