package org.firstinspires.ftc.teamcode.util.control.path

import org.firstinspires.ftc.teamcode.util.control.Pose2d
import org.firstinspires.ftc.teamcode.util.control.Rotation2d
import org.firstinspires.ftc.teamcode.util.control.Vector2d
import kotlin.math.cos
import kotlin.math.pow
import kotlin.math.sin

class CubicHermite(override val label:      String,
                   override val startPose:  Pose2d,
                            val v0:         Vector2d,
                   override val endPose:    Pose2d,
                            val v1:         Vector2d) : Path {
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

    override infix fun closestTo(p: Vector2d): Vector2d {
        return this(0.0)
    }

    // calculate p(t)
    operator fun invoke(t: Double): Vector2d {
        return coef[0] + coef[1]*t + coef[2]*t.pow(2) + coef[3]*t.pow(3)
    }
}

