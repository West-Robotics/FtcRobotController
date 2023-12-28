package com.scrapmetal.quackerama.control

import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.pow
import kotlin.math.sign
import kotlin.math.sin
import kotlin.math.sqrt

/**
 * A vector in both the mathematical and physical interpretation of the word
 */
data class Vector2d(val u: Double = 0.0, val v: Double = 0.0) {
    constructor(magnitude: Double, theta: Rotation2d) : this(magnitude*cos(theta.polarAngle), magnitude*sin(theta.polarAngle))
    val mag by lazy { hypot(u, v) }
    val polarAngle by lazy { atan2(v, u) }
    val unit by lazy { Vector2d(u/mag, v/mag) }
    val normal by lazy { Vector2d(-v, u) }
    // maybe some of these make lazy and don't calculate until you need it
    // companion object {
    //     val comparator = Comparator<Vector2d> { a, b ->
    //         when {
    //             a.mag < b.mag -> -1
    //             a.mag == b.mag -> 0
    //             else -> 1
    //         }
    //     }
    // }
    operator fun times(s: Double) = Vector2d(u*s, v*s)
    operator fun div(s: Double) = Vector2d(u/s, v/s)
    operator fun plus(w: Vector2d) = Vector2d(u+w.u, v+w.v)
    operator fun minus(w: Vector2d) = Vector2d(u-w.u, v-w.v)
    operator fun unaryMinus() = Vector2d(-u, -v)
    operator fun compareTo(w: Vector2d) = sign(mag - w.mag).toInt()
    fun distanceTo(w: Vector2d) = (w-this).let { hypot(it.u, it.v) }
}

// TODO: mega pain
/**
 * A rotation on a vector, rotation, or a pose
 */
data class Rotation2d(val u: Double = 1.0, val v: Double = 0.0) {
    constructor(v: Vector2d) : this(v.unit.u, v.unit.v)
    constructor(theta: Double) : this(cos(theta), sin(theta))
    val mag by lazy { hypot(u, v) }
    val polarAngle by lazy { atan2(v, u) }
    val normal by lazy { Rotation2d(-v, u) }
    val vector by lazy { Vector2d(u, v) }
    operator fun times(s: Double) = Vector2d(u*s, v*s)
    operator fun div(s: Double) = Vector2d(u/s, v/s)
    operator fun plus(w: Rotation2d) = Rotation2d(polarAngle + w.polarAngle)
    operator fun minus(w: Rotation2d) = Rotation2d(polarAngle - w.polarAngle)
    operator fun unaryMinus() = Vector2d(-u, -v)
    // what is this lol
    operator fun compareTo(w: Vector2d) = sign(mag - w.mag).toInt()
}

// TODO: mega pain
/**
 * A combination of position and heading
 */
data class Pose2d(val position: Vector2d = Vector2d(), val heading: Rotation2d = Rotation2d()) {
    operator fun plus(p: Pose2d) = Pose2d(position + p.position, heading + p.heading)
    operator fun minus(p: Pose2d) = Pose2d(position - p.position, heading - p.heading)
}
