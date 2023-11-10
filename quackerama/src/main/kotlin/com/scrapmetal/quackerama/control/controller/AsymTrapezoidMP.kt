// package com.scrapmetal.quackerama.control.controller
//
// import kotlin.math.pow
//
// /**
//  * Asymmetrical trapezoidal motion profile
//  *
//  * Note: create new motion profiles instead of mutating an existing one
//  * @param start start position
//  * @param end end position
//  * @param accel maximum acceleration during ramp up phase
//  * @param decel maximum deceleration during ramp down phase (negative number)
//  * @param v_max maximum velocity
//  */
// class AsymTrapezoidMP(val start: Double, val end: Double, val accel: Double, val decel: Double, val v_max: Double) {
//     val t_accel: Double
//     val t_decel: Double
//     val flipped = start > end
//
//     init {
//         if (!flipped) {
//             t_accel = v_max/accel
//             t_decel = -v_max/decel
//         } else {
//             t_accel
//         }
//     }
//
//     fun a(t: Double): Double {
//         return when {
//             t <= t_a -> accel
//             t < t_b -> 0.0
//             t <= t_total -> decel
//             t_total < t -> 0.0
//             else -> throw IllegalArgumentException("Received t < 0 as an argument")
//         }
//     }
//
//     fun v(t: Double): Double {
//         return when {
//             t <= t_a -> a(t)*t
//             t < t_b -> v_max
//             t <= t_total -> v_max + a(t)*(t-t_b)
//             t_total < t -> 0.0
//             else -> throw IllegalArgumentException("Received t < 0 as an argument")
//         }
//     }
//
//     fun s(t: Double): Double {
//         return start + when {
//             // i am not entirely sure why integrating velocity does not work
//             t <= t_a -> 0.5*a(t)*t.pow(2)
//             t < t_b -> s_a + v(t)*(t-t_a)
//             t <= t_total -> s_b + v(t-t_b)+ 0.5*a(t)*(t-t_b).pow(2)
//             t_total < t -> end
//             else -> throw IllegalArgumentException("Received t < 0 as an argument")
//         }
//     }
// }