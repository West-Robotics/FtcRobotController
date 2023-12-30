package com.scrapmetal.quackerama.control.controller

import kotlin.math.absoluteValue
import kotlin.math.pow
import kotlin.math.sqrt

data class MPState(val s: Double, val v: Double, val a: Double)

/**
 * Generates an asymmetrical trapezoidal motion profile given constraints
 *
 * Note that [decel] is negative
 *
 * Create new motion profiles instead of mutating an existing one
 * and always generate from current position
 */
// TODO: handle when you can't accel to max velo
class AsymTrapezoidMP(
        val start: Double,
        val end: Double,
        val accel: Double,
        val decel: Double,
        var v_max: Double
) {
    var t_accel: Double
    var t_decel: Double
    val t_cruise: Double
    val t_a: Double
    val t_b: Double
    val t_total: Double
    val s_a: Double
    val s_b: Double
    val direction = if (start < end) 1 else -1

    init {
        t_accel = if (accel != 0.0) v_max/accel else 0.0
        t_decel = if (decel != 0.0) -v_max/decel else 0.0
        val accelRatioedDistance = (1.0-(accel/(accel+(-decel))))*(end-start).absoluteValue
        if (0.5*accel*t_accel.pow(2) > accelRatioedDistance) {
            t_accel = sqrt(accelRatioedDistance/(0.5*accel))
            t_decel = sqrt(((end-start).absoluteValue - accelRatioedDistance)/(0.5*(-decel)))
            v_max = accel*t_accel
        }
        t_cruise = if (v_max != 0.0) ((end-start).absoluteValue-(0.5*accel*t_accel.pow(2)-0.5*decel*t_decel.pow(2)))/v_max else 0.0
        t_a     = t_accel
        t_b     = t_accel + t_cruise
        t_total = t_accel + t_cruise + t_decel
        s_a = direction*0.5*accel*t_accel.pow(2)
        s_b = s_a + direction*v_max*t_cruise
    }

    fun update(t: Double): MPState {
        val a = direction*when {
            t <= t_a -> accel
            t < t_b -> 0.0
            t <= t_total -> decel
            t_total < t -> 0.0
            else -> throw IllegalArgumentException("Received t < 0 as an argument")
        }
        val v = when {
            t <= t_a -> a*t
            t < t_b -> direction*v_max
            t <= t_total -> direction*v_max + a*(t-t_b)
            t_total < t -> 0.0
            else -> throw IllegalArgumentException("Received t < 0 as an argument")
        }
        val s = start + when {
            t <= t_a -> 0.5*a*t.pow(2)
            t < t_b -> s_a + v*(t-t_a)
            t <= t_total -> s_b + direction*v_max*(t-t_b) + 0.5*a*(t-t_b).pow(2)
            t_total < t -> end - start
            else -> throw IllegalArgumentException("Received t < 0 as an argument")
        }
        return MPState(s, v, a)
    }
}