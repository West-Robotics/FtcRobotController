package com.scrapmetal.quackerama.control.gvf

import com.scrapmetal.quackerama.control.Vector2d
import com.scrapmetal.quackerama.control.path.Path

class GVF(val path: Path, val kN: Double) {
    // TODO: add ability for path sequences

    fun getVector(p: Vector2d): Vector2d {
        // TODO: figure out how much to push closestT
        if (path.paths[0].closestT(p) > 0.99) {
            return path.update(p).second.unit
        } else {
            return path.update(p).let { it.first + it.second * kN }.unit
        }
    }
    fun getClosestT(p: Vector2d): Double {
        return path.paths[0].closestT(p)
    }
}