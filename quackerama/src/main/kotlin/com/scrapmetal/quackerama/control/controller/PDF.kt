package com.scrapmetal.quackerama.control.controller

/**
 * A PDF controller with variable F to account for the designer's choice
 *
 * Examples: give cos for arm feedforward, overcome varying friction in linear slides
 * especially for small inputs
 */
class PDF(var p: Double, var d: Double, var f: (x: Double) -> Double) {
    var error = 0.0
    var dx = 0.0

    operator fun invoke(x: Double): Double {
        
        return p*x + - d*dx + f(x)
    }
}