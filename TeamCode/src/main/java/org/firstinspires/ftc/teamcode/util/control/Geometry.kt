package org.firstinspires.ftc.teamcode.util.control

import java.util.Vector
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.sign
import kotlin.math.sin

data class Vector2d(val u: Double = 0.0, val v: Double = 0.0) {
    val mag = hypot(u, v)
    val unit = Vector2d(u/mag, v/mag)
    val normal = Vector2d(-v, u)
    // maybe some of these make lazy and don't calculate until you need it
    val polarAngle = atan2(v, u)
    companion object {
        val comparator = Comparator<Vector2d> { a, b ->
            when {
                a.mag < b.mag -> -1
                a.mag == b.mag -> 0
                else -> 1
            }
        }
    }
    operator fun times(s: Double) = Vector2d(u*s, v*s)
    operator fun div(s: Double) = Vector2d(u/s, v/s)
    operator fun plus(w: Vector2d) = Vector2d(u+w.u, v+w.v)
    operator fun minus(w: Vector2d) = Vector2d(u-w.u, v-w.v)
    operator fun unaryMinus() = Vector2d(-u, -v)
    operator fun compareTo(w: Vector2d) = sign(mag - w.mag).toInt()
}

// TODO: mega pain
data class Rotation2d(val u: Double = 0.0, val v: Double = 0.0) {
    constructor(v: Vector2d) : this(v.u, v.v)
    constructor(theta: Double) : this(toVector(theta).u, toVector(theta).v)
    companion object {
        fun toVector(theta: Double): Vector2d {
            return Vector2d(cos(theta), sin(theta))
        }
    }
    val mag = hypot(u, v)
    val polarAngle = atan2(v, u)
    val normal = Vector2d(-v, u)
    val vector = Vector2d(u, v)
    operator fun times(s: Double) = Vector2d(u*s, v*s)
    operator fun div(s: Double) = Vector2d(u/s, v/s)
    operator fun plus(w: Rotation2d) = Rotation2d(u+w.u, v+w.v)
    operator fun minus(w: Rotation2d) = Rotation2d(u-w.u, v-w.v)
    operator fun unaryMinus() = Vector2d(-u, -v)
    // what is this lol
    operator fun compareTo(w: Vector2d) = sign(mag - w.mag).toInt()
}

// TODO: mega pain
data class Pose2d(val position: Vector2d = Vector2d(), val heading: Rotation2d = Rotation2d()) {
    operator fun plus(p: Pose2d) = Pose2d(position + p.position, heading + p.heading)
    operator fun minus(p: Pose2d) = Pose2d(position - p.position, heading - p.heading)
}
