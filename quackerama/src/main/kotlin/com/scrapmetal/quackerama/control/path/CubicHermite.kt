package com.scrapmetal.quackerama.control.path

import com.scrapmetal.quackerama.control.Pose2d
import com.scrapmetal.quackerama.control.Rotation2d
import com.scrapmetal.quackerama.control.Vector2d
import com.scrapmetal.quackerama.control.gvf.GVFState
import kotlin.math.absoluteValue
import kotlin.math.pow

/**
 * A cubic Hermite spline for paths generated with standard [PathSegment] constraints and initial
 * and end Hermite velocites [v0] and [v1] respectively
 */
class CubicHermite(
    override val label: String,
    override val startPose: Pose2d,
    val v0: Vector2d,
    override val endPose: Pose2d,
    val v1: Vector2d,
    override val constraints: MovementConstraints,
) : PathSegment {
    val p_a: Vector2d
    val p_b: Vector2d
    val coef: Array<Vector2d>
    val positions: Array<Vector2d>

    init {
        p_a = startPose.position + v0/3.0
        p_b = endPose.position - v1/3.0
        coef = Array(4) {
            when (it) {
                0 ->  startPose.position
                1 -> -startPose.position*3.0   + p_a*3.0
                2 ->  startPose.position*3.0   - p_a*6.0 + p_b*3.0
                3 -> -startPose.position       + p_a*3.0 - p_b*3.0 + endPose.position
                else -> Vector2d()
            }
        }
        positions = Array(101) { t: Int -> invoke(t/100.0) }
    }

    override fun invoke(t: Double): Vector2d {
        return coef[0] + coef[1]*t + coef[2]*t.pow(2) + coef[3]*t.pow(3)
    }

    override fun dpdt(t: Double): Vector2d {
        return coef[1] + coef[2]*2.0*t + coef[3]*3.0*t.pow(2)
    }

    override fun closestT(p: Vector2d): Double {
        return positions.indexOf(positions.minBy { q: Vector2d -> q.distanceTo(p) })/100.0
    }

    override fun update(p: Vector2d): GVFState {
        val t = closestT(p)
        return GVFState(dpdt(t).unit, invoke(t) - p, t)
    }

    private val dx = 0.01
    private val tolerance = 0.02
    fun deriv(f: (Double) -> Double): (Double) -> Double {
        return { x: Double -> ((f(x+dx) - f(x))/dx).let {
            if (it == 0.0) {
                0.000001
            } else {
                it
            }
        }}
    }
}

