package com.scrapmetal.quackerama.control.controller

import kotlin.math.PI
import kotlin.math.absoluteValue

data class PDFState(val output: Double, val p: Double, val d: Double, val error: Double, val dxdt: Double)
/**
 * A PDF controller with feedforward given as a function
 *
 * Examples: give cos for arm feedforward, overcome varying friction in linear slides
 * especially for small inputs
 * There is no integral term because I am too lazy implement the proper safeties for that
 * And I don't see a good use in FTC to use integral when a proper feedforward performs better
 *
 * Parameters are variables because there are some use cases where changing the gains is valid
 */
class PDF(var p: Double, var d: Double, var f: (x: Double) -> Double, var maxMagnitude: Double, val continuous: Boolean = false) {
    var error = 0.0
    var dxdt = 0.0
    // L statefulness
    var lastX = 0.0

    /**
     * Calculate PDF output
     *
     * @param x current state x
     * @param setpoint setpoint
     * @param dt timestep in milliseconds
     */
    fun update(x: Double, setpoint: Double, dt: Double): Double {
        error = setpoint - x
        dxdt = (x - lastX)/dt
        lastX = x
        if (continuous) {
            if (error.absoluteValue > PI) {
                error += if (error > 0) -2*PI else 2*PI
            }
        }
        return ((p*error) + (-d*dxdt) + (f(x))).coerceIn(-maxMagnitude, maxMagnitude)
    }
    fun updateWithState(x: Double, setpoint: Double, dt: Double): PDFState {
        error = setpoint - x
        dxdt = (x - lastX)/dt
        lastX = x
        if (continuous) {
            if (error.absoluteValue > PI) {
                error += if (error > 0) -2*PI else 2*PI
            }
        }
        return PDFState(((p*error) + (-d*dxdt) + (f(x))).coerceIn(-maxMagnitude, maxMagnitude), p, d, error, dxdt)
    }
}