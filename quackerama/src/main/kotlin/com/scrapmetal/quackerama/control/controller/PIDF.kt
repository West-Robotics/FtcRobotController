package com.scrapmetal.quackerama.control.controller

import kotlin.math.PI
import kotlin.math.absoluteValue

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
class PIDF(
    private var p: Double,
    private var i: Double,
    private var d: Double,
    private var f: (x: Double) -> Double = { _: Double -> 0.0 },
    private var minPowerToMove: Double = 0.0,
    private var deadzone: Double = 0.0,
    val maxMagnitude: Double = 1.0,
    private val continuous: Boolean = false,
    private var iZone: Double = 0.0,
    private var maxIOutput: Double = 0.0,
) {
    // L statefulness
    private var lastY = 0.0
    private var eIntegral = 0.0
    private val dxdtList = ArrayList<Double>()

    /**
     * Calculate PIDF output
     *
     * @param y state output y
     * @param reference reference
     * @param dt timestep in seconds
     */
    // NOTE: i think a lot of people usually use x, not y cause x is the state variable. i think y
    //   is technically more correct since the plant output is what is fed back into the loop
    fun update(y: Double, reference: Double, dt: Double): Double {
        val error = (reference - y).let {
            // TODO: generalize beyond assuming radians + full circle
            if (continuous && it.absoluteValue > PI) {
                it + if (it > 0) -2*PI else 2*PI
            } else {
                it
            }
        }
        val dxdt = (y - lastY)/dt
        dxdtList.add(dxdt)
        if (dxdtList.size > 2) dxdtList.removeAt(0)
        lastY = y
        eIntegral += if (error < iZone) error else 0.0
        return Utils.correctWithMinPower(
            p*error + (i*eIntegral).coerceIn(-maxIOutput..maxIOutput) - d*dxdtList.average() + f(y),
            minPowerToMove,
            deadzone,
            maxMagnitude
        )
    }
}