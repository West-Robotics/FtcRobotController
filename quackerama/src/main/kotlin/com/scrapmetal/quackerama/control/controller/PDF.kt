package com.scrapmetal.quackerama.control.controller

import kotlin.math.PI
import kotlin.math.absoluteValue
import kotlin.math.sign

/**
 * A PDF controller with feedforward given as a function
 *
 * Examples: give cos for arm feedforward, overcome varying friction in linear slides
 * especially for small inputs
 * There is no integral term because I am too lazy implement the proper safeties for that
 * And I don't see a good use in FTC to use integral when a proper feedforward performs better
 *
 * Parameters are variables because there are some use cases where changing the gains is valid
 *
 * @param p proportional gain
 * @param d derivative gain
 * @param f feedforward function
 * @param minPowerToMove minimum output necessary to just barely adjust the state
 * @param deadzone zone beneath which [minPowerToMove] is not applied
 * @param maxMagnitude maximum output for both + and -
 * @param continuous whether or not the input is wrapped around (e.g. rotational actuators)
 */
class PDF(
    private var p: Double,
    private var d: Double,
    private var f: (x: Double) -> Double = { _: Double -> 0.0 },
    private var minPowerToMove: Double = 0.0,
    private var deadzone: Double = 0.0,
    private var maxMagnitude: Double = 1.0,
    private val continuous: Boolean = false,
) {
    // L statefulness
    private var lastX = 0.0

    /**
     * Calculate PDF output
     *
     * @param x current state x
     * @param setpoint setpoint
     * @param dt timestep in milliseconds
     */
    fun update(x: Double, setpoint: Double, dt: Double): Double {
        val error = (setpoint - x).let {
            // TODO: generalize beyond assuming radians + full circle
            if (continuous && it.absoluteValue > PI) {
                it + if (it > 0) -2*PI else 2*PI
            } else {
                it
            }
        }
        val dxdt = (x - lastX)/dt
        lastX = x
        return Utils.correctWithMinPower(p*error + -d*dxdt + f(x), minPowerToMove, deadzone, maxMagnitude)
    }
}