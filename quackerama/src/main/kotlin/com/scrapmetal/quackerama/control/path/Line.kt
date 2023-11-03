package com.scrapmetal.quackerama.control.path

import com.scrapmetal.quackerama.control.Pose2d
import com.scrapmetal.quackerama.control.Rotation2d
import com.scrapmetal.quackerama.control.Vector2d

/**
 * A line segment for a path
 *
 * Path start and end headings inherently cannot be controlled
 */
class Line(override val label:      String,
           override val startPose:  Pose2d,
           override val endPose:    Pose2d
) : PathSegment {
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