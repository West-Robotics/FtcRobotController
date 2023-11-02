package com.scrapmetal.quackerama.control.controller

class PDF(var p: Double, var d: Double, var f: (x: Double) -> Double) {
    var error = 0.0
    var dx = 0.0

    operator fun invoke(x: Double): Double {
        
        return p*x + - d*dx + f(x)
    }
}