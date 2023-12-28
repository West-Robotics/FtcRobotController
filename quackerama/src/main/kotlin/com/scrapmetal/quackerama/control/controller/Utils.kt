package com.scrapmetal.quackerama.control.controller

import kotlin.math.absoluteValue
import kotlin.math.sign

object Utils {
    fun correctWithMinPower(u: Double,
                            minPowerToMove: (x: Double) -> Double,
                            deadzone: Double,
                            maxMagnitude: Double): Double {
        return minPowerToMove(u).let { p0 ->
            when {
                u.absoluteValue > deadzone
                -> (maxMagnitude-p0)/(maxMagnitude-deadzone) * (u - u.sign*deadzone) + u.sign*p0
                else
                -> (p0/deadzone) * u
            }
        }.coerceIn(-maxMagnitude, maxMagnitude)
    }
}