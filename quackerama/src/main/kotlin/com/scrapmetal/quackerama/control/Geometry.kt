package com.scrapmetal.quackerama.control

import kotlin.math.PI
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.sign
import kotlin.math.sin

data class Vector2d(val x: Double = 0.0, val y: Double = 0.0) {
    // TODO: add polar coordinates?
    val mag by lazy { hypot(x, y) }
    val polarAngle by lazy { atan2(y, x) }
    val unit by lazy { if (mag != 0.0) Vector2d(x/mag, y/mag) else Vector2d(0.0, 0.0) }
    val normal by lazy { Vector2d(-y, x) }
    // companion object {
    //     val comparator = Comparator<Vector2d> { a, b ->
    //         when {
    //             a.mag < b.mag -> -1
    //             a.mag == b.mag -> 0
    //             else -> 1
    //         }
    //     }
    // }
    operator fun times(s: Double) = Vector2d(x*s, y*s)
    operator fun div(s: Double) = Vector2d(x/s, y/s)
    operator fun plus(v: Vector2d) = Vector2d(x+v.x, y+v.y)
    operator fun minus(v: Vector2d) = Vector2d(x-v.x, y-v.y)
    operator fun unaryMinus() = Vector2d(-x, -y)
    operator fun compareTo(w: Vector2d) = sign(mag - w.mag).toInt()
    fun distanceTo(w: Vector2d) = (w-this).let { hypot(it.x, it.y) }
}

/**
 * A rotation on a vector, rotation, or a pose
 */
data class Rotation2d(val theta: Double = 0.0) {
    val normal by lazy { Rotation2d(theta + PI/2) }
    val vector by lazy { Vector2d(cos(theta), sin(theta)) }
    val inverse by lazy { Rotation2d(-theta) }
    operator fun times(s: Double) = Rotation2d(s*theta)
    operator fun times(v: Vector2d) =
        Vector2d(
            v.x*cos(theta) - v.y*sin(theta),
            v.x*sin(theta) + v.y*cos(theta)
        )
    operator fun div(s: Double) = Rotation2d(theta/s)
    operator fun plus(r: Rotation2d) = Rotation2d(theta + r.theta)
    operator fun minus(r: Rotation2d) = Rotation2d(theta - r.theta)
    // NOTE: originally had unary minus, removed due to being confusing between flipping by 180 deg
    // and the inverse
    // operator fun compareTo(r: Rotation2d) = sign(mag - w.mag).toInt()
}

/**
 * A combination of position and heading
 */
data class Pose2d(val position: Vector2d = Vector2d(), val heading: Rotation2d = Rotation2d()) {
    constructor(x: Double = 0.0, y: Double = 0.0, theta: Double = 0.0) : this(Vector2d(x, y), Rotation2d(theta))
    operator fun plus(p: Pose2d) = Pose2d(position + p.position, heading + p.heading)
    operator fun minus(p: Pose2d) = Pose2d(position - p.position, heading - p.heading)
}
