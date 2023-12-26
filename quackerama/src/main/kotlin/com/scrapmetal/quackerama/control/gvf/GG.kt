package com.scrapmetal.quackerama.control.gvf

import com.scrapmetal.quackerama.control.Vector2d
import com.scrapmetal.quackerama.control.path.Path

// TODO: include info like curvature so other controller can handle centripetal, GVF just handles m_d
data class GVFState(val m_d: Vector2d, val error: Vector2d, val closestT: Double)
class GVF(val path: Path, val kN: Double) {
    // TODO: add ability for path sequences

    fun update(p: Vector2d): GVFState {
        val closestT = path.paths[0].closestT(p)
        if (closestT > 0.99) {
            path.update(p).let {
                return GVFState(it.error.unit, it.error, it.closestT)
            }
        } else {
            path.update(p).let {
                return GVFState((it.m_d + it.error*kN).unit, it.error, it.closestT)
            }
        }
    }
    // fun getVector(p: Vector2d): Vector2d {
    //     // TODO: figure out how much to push closestT
    //     if (path.paths[0].closestT(p) > 0.99) {
    //         return path.update(p).second.unit
    //     } else {
    //         return path.update(p).let { it.first + it.second * kN }.unit
    //     }
    // }
    fun getClosestT(p: Vector2d): Double {
        return path.paths[0].closestT(p)
    }
}