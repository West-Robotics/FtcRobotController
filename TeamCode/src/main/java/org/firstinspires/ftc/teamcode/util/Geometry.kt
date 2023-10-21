package org.firstinspires.ftc.teamcode.util

import kotlin.math.atan2
import kotlin.math.hypot
import kotlin.math.sign

data class Vector2d(val u: Double = 0.0, val v: Double = 0.0) {
    fun mag() = hypot(u, v)
    fun toPolar() = atan2(v, u)
    fun unit() = mag().let { Vector2d(u / it, v / it) }
    fun normal() = Vector2d(-v, u)
    operator fun times(s: Double) = Vector2d(u*s, v*s)
    operator fun div(s: Double) = Vector2d(u/s, v/s)
    operator fun plus(w: Vector2d) = Vector2d(u+w.u, v+w.v)
    operator fun minus(w: Vector2d) = Vector2d(u-w.u, v-w.v)
    operator fun unaryMinus() = Vector2d(-u, -v)
    operator fun compareTo(w: Vector2d) = sign(mag() - w.mag()).toInt()
    // does equals already exist?
    // other is Vector2d && u == other.u && v == other.v
}

// TODO: mega pain
data class Rotation2d(val u: Double = 0.0, val v: Double = 0.0) {
    fun mag() = hypot(u, v)
    fun toPolar() = atan2(v, u)
    fun normal() = Vector2d(-v, u)
    operator fun times(s: Double) = Vector2d(u*s, v*s)
    operator fun div(s: Double) = Vector2d(u/s, v/s)
    operator fun plus(w: Rotation2d) = Rotation2d(u+w.u, v+w.v)
    operator fun minus(w: Rotation2d) = Rotation2d(u-w.u, v-w.v)
    operator fun unaryMinus() = Vector2d(-u, -v)
    operator fun compareTo(w: Vector2d) = sign(mag() - w.mag()).toInt()
}

// TODO: mega pain
data class Pose2d(val position: Vector2d, val heading: Rotation2d) {
    operator fun plus(p: Pose2d) = Pose2d(position + p.position, heading + p.heading)
    operator fun minus(p: Pose2d) = Pose2d(position - p.position, heading - p.heading)
}
