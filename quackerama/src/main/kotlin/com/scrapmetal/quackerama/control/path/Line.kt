package com.scrapmetal.quackerama.control.path

import com.scrapmetal.quackerama.control.Pose2d
import com.scrapmetal.quackerama.control.Rotation2d
import com.scrapmetal.quackerama.control.Vector2d
import com.scrapmetal.quackerama.control.gvf.GVFState
import kotlin.math.PI
import kotlin.math.atan2
import kotlin.math.sin

/**
 * A line segment for a path
 *
 * Path start and end headings inherently cannot be independently controlled from position, so they
 * are effectively useless here
 */
class Line(override val label:       String,
           override val startPose:   Pose2d,
           override val endPose:     Pose2d,
           override val constraints: MovementConstraints,
) : PathSegment {
    val tau = (endPose.position - startPose.position).unit
    val length = (endPose.position - startPose.position).mag
    override fun invoke(t: Double): Vector2d {
        // LERP
        return startPose.position*(1-t) + endPose.position*t
    }

    override fun dpdt(t: Double): Vector2d {
        return tau
    }

    override fun closestT(p: Vector2d): Double {
        val c = (p - startPose.position).mag
        // magic to find the length of the side that lays on the path for the triangle described by
        // the start point, current point, and closest point on the path
        val a = (c * sin(PI/2 - (atan2(p.v, p.u)
                                    - atan2(startPose.position.v, startPose.position.u))))
        return a / length
    }

    override fun update(p: Vector2d): GVFState {
        val t = closestT(p).let {
            when {
                it < 0.0 -> 0.0
                1.0 < it -> 1.0
                else -> it
            }
        }
        return GVFState(tau, invoke(t) - p, t)
    }
}