package org.firstinspires.ftc.teamcode.util.control.path

import org.firstinspires.ftc.teamcode.util.control.Pose2d
import org.firstinspires.ftc.teamcode.util.control.Rotation2d
import org.firstinspires.ftc.teamcode.util.control.Vector2d

interface PathSegment {
    val label: String
    val startPose: Pose2d
    val endPose: Pose2d
    operator fun invoke(t: Double): Vector2d
    fun tauOf(p: Vector2d): Rotation2d
    fun eOf(p: Vector2d): Vector2d
    fun closestT(p: Vector2d): Double
    fun closestPoint(p: Vector2d): Vector2d
}