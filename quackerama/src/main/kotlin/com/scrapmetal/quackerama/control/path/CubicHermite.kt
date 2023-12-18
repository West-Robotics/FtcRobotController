package com.scrapmetal.quackerama.control.path

import com.scrapmetal.quackerama.control.Pose2d
import com.scrapmetal.quackerama.control.Rotation2d
import com.scrapmetal.quackerama.control.Vector2d
import kotlin.math.absoluteValue
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
    val coef: Array<Vector2d>

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
    }

    override fun invoke(t: Double): Vector2d {
        return coef[0] + coef[1]*t + coef[2]*t.pow(2) + coef[3]*t.pow(3)
    }

    override fun dpdt(t: Double): Vector2d {
        return coef[1] + coef[2]*2.0*t + coef[3]*3.0*t.pow(2)
    }

    override fun closestT(p: Vector2d): Double {
        println("closestTing")
        var t = newton(deriv { t: Double -> invoke(t).distanceTo(p) }, 0.5 )
        // var t = DoubleArray(4) { x: Int
        //     -> newton(deriv { t: Double -> invoke(t).distanceTo(p) }, (x + 1.0) / 5.0) }
        //     // .filter { 0.0 < it && it < 1.0 }
        //     .minBy { t: Double -> invoke(t).distanceTo(p) }
        // if (t < 0.0) {
        //     t = 0.0
        // } else if (t > 1.0) {
        //     t = 1.0
        // }
        return t
    }

    override fun update(p: Vector2d): Pair<Vector2d, Vector2d> {
        println("updating")
        val t = closestT(p)
        return Pair(dpdt(t).unit, invoke(t) - p)
    }

    private val dx = 0.01
    private val tolerance = 0.02
    fun closeEnough(a: Double, b: Double): Boolean {
        return (a-b).absoluteValue < tolerance
    }
    fun deriv(f: (Double) -> Double): (Double) -> Double {
        return { x: Double -> ((f(x+dx) - f(x))/dx).let {
            if (it == 0.0) {
                0.000001
            } else {
                it
            }
        }}
    }
    fun newton(f: (Double) -> Double, x0: Double): Double {
        fun transform(xn: Double): Double {
            return xn - f(xn) / deriv(f)(xn)
        }

        var lastGuess = x0
        var currentGuess = transform(lastGuess)
        var counter = 0
        while (!closeEnough(currentGuess, lastGuess) && counter < 6) {
            lastGuess = currentGuess
            currentGuess = transform(currentGuess)
            counter++
            println("newtoning")
        }
        return currentGuess
    }
}
