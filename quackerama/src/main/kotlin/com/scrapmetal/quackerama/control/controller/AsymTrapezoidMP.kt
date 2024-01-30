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
        private val accel: Double,
        private val decel: Double,
        private var v_max: Double,
) {
    private var tAccel: Double
    private var tDecel: Double
    private val tCruise: Double
    private val tA: Double
    private val tB: Double
    val tTotal: Double
    private val sA: Double
    private val sB: Double
    private val direction = if (start < end) 1 else -1

    init {
        tAccel = if (accel != 0.0) v_max/accel else 0.0
        tDecel = if (decel != 0.0) -v_max/decel else 0.0
        val accelRatioedDistance = (1.0-(accel/(accel+(-decel))))*(end-start).absoluteValue
        if (0.5*accel*tAccel.pow(2) > accelRatioedDistance) {
            tAccel = sqrt(accelRatioedDistance/(0.5*accel))
            tDecel = sqrt(((end-start).absoluteValue - accelRatioedDistance)/(0.5*(-decel)))
            v_max = accel*tAccel
        }
        tCruise = if (v_max != 0.0) {
            (((end-start).absoluteValue - (0.5*accel*tAccel.pow(2)
                                           - 0.5*decel*tDecel.pow(2)))
             / v_max)
        } else {
            0.0
        }
        tA = tAccel
        tB = tAccel + tCruise
        tTotal = tAccel + tCruise + tDecel
        sA = direction*0.5*accel*tAccel.pow(2)
        sB = sA + direction*v_max*tCruise
    }

    fun update(t: Double): MPState {
        // TODO: huh???
        val a = direction*when {
            t <= tA -> accel
            t < tB -> 0.0
            t <= tTotal -> decel
            tTotal < t -> 0.0
            else -> throw IllegalArgumentException("Received t < 0 as an argument")
        }
        val v = when {
            t <= tA -> a*t
            t < tB -> direction*v_max
            t <= tTotal -> direction*v_max + a*(t-tB)
            tTotal < t -> 0.0
            else -> throw IllegalArgumentException("Received t < 0 as an argument")
        }
        val s = start + when {
            t <= tA -> 0.5*a*t.pow(2)
            t < tB -> sA + v*(t-tA)
            t <= tTotal -> sB + direction*v_max*(t-tB) + 0.5*a*(t-tB).pow(2)
            tTotal < t -> end - start
            else -> throw IllegalArgumentException("Received t < 0 as an argument")
        }
        return MPState(s, v, a)
    }
}