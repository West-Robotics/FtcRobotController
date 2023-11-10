package com.scrapmetal.quackerama.control.controller

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
class PDF(var p: Double, var d: Double, var f: (x: Double) -> Double, var maxOutput: Double) {
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
    operator fun invoke(x: Double, setpoint: Double, dt: Double): Double {
        error = setpoint - x
        dxdt = (x - lastX)/dt
        lastX = x
        return minOf((p*error) + (-d*dxdt) + (f(x)), maxOutput)
    }
}