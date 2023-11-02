package org.firstinspires.ftc.teamcode.util.control.path

import org.firstinspires.ftc.teamcode.util.control.Pose2d
import org.firstinspires.ftc.teamcode.util.control.Rotation2d
import org.firstinspires.ftc.teamcode.util.control.Vector2d

class Line(override val label:      String,
           override val startPose:  Pose2d,
           override val endPose:    Pose2d) : PathSegment {
    val tau = Rotation2d((endPose.position - startPose.position).unit)
    override fun invoke(t: Double): Vector2d {
        // LERP
        return startPose.position*(1-t) + endPose.position*t
    }

    override fun tauOf(p: Vector2d) = tau

    override fun eOf(p: Vector2d): Vector2d {
        TODO("Not yet implemented")
    }

    override fun closestT(p: Vector2d): Double {
        TODO("Not yet implemented")
    }

    override fun closestPoint(p: Vector2d): Vector2d {
        return Vector2d()
    }
}