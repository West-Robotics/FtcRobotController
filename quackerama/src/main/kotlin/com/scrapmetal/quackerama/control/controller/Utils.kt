package com.scrapmetal.quackerama.control.controller

import kotlin.math.absoluteValue
import kotlin.math.sign

object Utils {
    /**
     * Once outside the [deadzone], correct initial controller effort [u0] with minimum required
     * effort to move the actuator [uMin] without exerting more than the [max] effort
     */
    fun correctWithMinPower(
        u0: Double,
        uMin: Double,
        deadzone: Double,
        max: Double
    ): Double {
        return when {
            u0.absoluteValue > deadzone
                -> (max-uMin)/(max-deadzone) * (u0 - u0.sign*deadzone) + u0.sign*uMin
            else -> (uMin/deadzone) * u0
        }.coerceIn(-max, max)
    }
}