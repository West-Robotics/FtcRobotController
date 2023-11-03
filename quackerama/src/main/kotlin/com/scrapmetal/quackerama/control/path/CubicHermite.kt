package com.scrapmetal.quackerama.control.path

import com.scrapmetal.quackerama.control.Pose2d
import com.scrapmetal.quackerama.control.Rotation2d
import com.scrapmetal.quackerama.control.Vector2d
import kotlin.math.pow

/**
 * A cubic Hermite spline for paths
 *
 * @param v0 initial Hermite velocity
 * @param v1 end Hermite velocity
 */
class CubicHermite(override val label:      String,
                   override val startPose: Pose2d,
                            val v0: Vector2d,
                   override val endPose: Pose2d,
                            val v1: Vector2d ) : PathSegment {
    val p_a: Vector2d
    val p_b: Vector2d
    val coef = Array(4) { Vector2d() }

    init {
        p_a = startPose.position + (v0-startPose.position)/3.0
        p_b = endPose.position - (v1-startPose.position)/3.0
        coef[0] =  startPose.position
        coef[1] = -startPose.position*3.0   + p_a*3.0
        coef[2] =  startPose.position*3.0   - p_a*6.0 + p_b*3.0
        coef[3] = -startPose.position       + p_a*3.0 - p_b*3.0 + endPose.position
    }

    override fun invoke(t: Double): Vector2d {
        return coef[0] + coef[1]*t + coef[2]*t.pow(2) + coef[3]*t.pow(3)
    }
    override fun tauOf(p: Vector2d): Rotation2d {
        TODO("Not yet implemented")
    }

    override fun eOf(p: Vector2d): Vector2d {
        TODO("Not yet implemented")
    }

    override fun closestT(p: Vector2d): Double {
        TODO("Not yet implemented")
    }

    override fun closestPoint(p: Vector2d): Vector2d {
        TODO("Not yet implemented")
    }
}

