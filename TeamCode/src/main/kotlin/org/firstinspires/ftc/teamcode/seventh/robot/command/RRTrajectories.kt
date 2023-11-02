package org.firstinspires.ftc.teamcode.seventh.robot.command

import com.acmerobotics.roadrunner.geometry.Pose2d
import org.firstinspires.ftc.teamcode.seventh.drive.DriveConstants
import org.firstinspires.ftc.teamcode.seventh.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.seventh.robot.hardware.Globals
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.GetPropPositionPipeline
import org.firstinspires.ftc.teamcode.seventh.trajectorysequence.TrajectorySequence
import java.lang.Math.toRadians

class RRTrajectories(drive: SampleMecanumDrive,
                     side: Globals.Side,
                     start: Globals.Start,
                     lane: Globals.Lane,
                     val prop: GetPropPositionPipeline.PropPosition) {
    val dropOff: TrajectorySequence
    val score: TrajectorySequence
    val park: TrajectorySequence
    val startPose: Pose2d

    init {
        when {
            side == Globals.Side.RED && start == Globals.Start.CLOSE ->     startPose = Pose2d( 36.0, -65.6, toRadians(90.0))
            side == Globals.Side.RED && start == Globals.Start.FAR ->       startPose = Pose2d(-36.0, -65.6, toRadians(90.0))
            side == Globals.Side.BLUE && start == Globals.Start.CLOSE ->    startPose = Pose2d( 36.0,  65.6, toRadians(-90.0))
            side == Globals.Side.BLUE && start == Globals.Start.FAR ->      startPose = Pose2d(-36.0,  65.6, toRadians(-90.0))
            else -> startPose = Pose2d(36.0, -65.6, toRadians(-90.0))
        }
        drive.poseEstimate = startPose
        dropOff = with (drive.trajectorySequenceBuilder(startPose)) {
            when (prop) {
                GetPropPositionPipeline.PropPosition.LEFT -> {
                    this.forward(18.0, SampleMecanumDrive.getVelocityConstraint(40.0, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(40.0))
                        .turn(toRadians(45.0))
                        .forward(11.0)
                }
                GetPropPositionPipeline.PropPosition.MIDDLE -> {
                    this.forward(30.0, SampleMecanumDrive.getVelocityConstraint(40.0, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(40.0))
                }
                GetPropPositionPipeline.PropPosition.RIGHT -> {
                    this.forward(18.0, SampleMecanumDrive.getVelocityConstraint(40.0, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(40.0))
                        .turn(toRadians(-45.0))
                        .forward(11.0)
                }
            }
            this.build()
        }
        score = with (drive.trajectorySequenceBuilder(dropOff.end())) {
            when (prop) {
                GetPropPositionPipeline.PropPosition.LEFT -> {
                    this.back(11.0)
                        .turn(toRadians(-45.0))
                }
                GetPropPositionPipeline.PropPosition.MIDDLE -> {
                    Unit
                }
                GetPropPositionPipeline.PropPosition.RIGHT -> {
                    this.back(11.0)
                        .turn(toRadians(45.0))
                }
            }
            when (lane) {
                Globals.Lane.LANE_1 -> when (prop) {
                    GetPropPositionPipeline.PropPosition.LEFT -> {
                        this.back(12.0, SampleMecanumDrive.getVelocityConstraint(40.0, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(10.0))
                    }
                    GetPropPositionPipeline.PropPosition.MIDDLE -> {
                        this.back(24.0, SampleMecanumDrive.getVelocityConstraint(40.0, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(40.0))
                    }
                    GetPropPositionPipeline.PropPosition.RIGHT -> {
                        this.back(12.0, SampleMecanumDrive.getVelocityConstraint(40.0, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(10.0))
                    }
                }
                Globals.Lane.LANE_2 -> TODO("asfd")
                Globals.Lane.LANE_3 -> when (prop) {
                    GetPropPositionPipeline.PropPosition.LEFT -> {
                        when (side) {
                            Globals.Side.RED -> 1.0
                            Globals.Side.BLUE -> -1.0
                        }.let {
                            this.forward(36.0)
                                .turn(-it*toRadians(90.0))
                                .forward(24.0)
                        }
                    }
                    GetPropPositionPipeline.PropPosition.MIDDLE -> {
                        when (side) {
                            Globals.Side.RED -> 1.0
                            Globals.Side.BLUE -> -1.0
                        }.let {
                            this.turn(-it*toRadians(90.0))
                                .forward(24.0)
                        }
                    }
                    GetPropPositionPipeline.PropPosition.RIGHT -> {
                        when (side) {
                            Globals.Side.RED -> 1.0
                            Globals.Side.BLUE -> -1.0
                        }.let {
                            this.forward(36.0)
                                .turn(-it*toRadians(90.0))
                                .forward(24.0)
                        }
                    }
                }
            }
            when (side) {
                Globals.Side.RED -> 1.0
                Globals.Side.BLUE -> -1.0
            }.let {
                when (lane) {
                    Globals.Lane.LANE_1 -> {
                        this.turn(toRadians(-it*90))
                    }
                    Globals.Lane.LANE_2 -> {
                        TODO("lane 2 not existing")
                    }
                    Globals.Lane.LANE_3 -> {
                        this.strafeLeft(it*24.0)
                            .forward(48.0)
                            .forward(52.0)
                            .turn(toRadians(-it*90))
                    }
                }
                when (start) {
                    Globals.Start.CLOSE -> {
                        this.forward(32.0)
                            .turn(toRadians(it*90.0))
                            if (side == Globals.Side.BLUE) {
                                this.forward(19.0)
                            } else {
                                this.forward(23.0)
                            }
                        this.turn(toRadians(-it*90.0))
                            .forward(14.0, SampleMecanumDrive.getVelocityConstraint(10.0, toRadians(60.0), DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(10.0))
                            // .back(4.0, SampleMecanumDrive.getVelocityConstraint(10.0, toRadians(60.0), DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(10.0))
                    }
                    Globals.Start.FAR -> {
                        this.forward(78.0)
                            .turn(toRadians(-it*90.0))
                            .forward(20.0)
                            .turn(toRadians(it*90.0))
                            .forward(8.0, SampleMecanumDrive.getVelocityConstraint(10.0, toRadians(60.0), DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(10.0))
                    }
                }
                when (prop) {
                    // GetPropPositionPipeline.PropPosition.LEFT -> this.strafeLeft(10.0)
                    GetPropPositionPipeline.PropPosition.LEFT -> this.strafeLeft(2.0)
                    GetPropPositionPipeline.PropPosition.MIDDLE -> this.strafeLeft(2.0)
                    GetPropPositionPipeline.PropPosition.RIGHT -> this.strafeRight(8.0)
                }
                this.forward(3.0, SampleMecanumDrive.getVelocityConstraint(10.0, toRadians(60.0), DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(10.0))
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
                        this.back(8.0)
                            .strafeRight(it*28.0)
                    }
                    Globals.Lane.LANE_2 -> {
                        TODO("lane 2 not existing")
                    }
                    Globals.Lane.LANE_3 -> {
                        this.back(8.0)
                            .strafeLeft(it*28.0)
                    }
                }
            }
            this.build()
        }
    }
}