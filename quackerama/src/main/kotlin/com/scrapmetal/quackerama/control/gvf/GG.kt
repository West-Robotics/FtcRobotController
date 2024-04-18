package com.scrapmetal.quackerama.control.gvf

import com.scrapmetal.quackerama.control.Pose2d
import com.scrapmetal.quackerama.control.Rotation2d
import com.scrapmetal.quackerama.control.Vector2d
import com.scrapmetal.quackerama.control.controller.PIDF
import com.scrapmetal.quackerama.control.path.Path
import kotlin.math.PI
import kotlin.math.absoluteValue

// TODO: include info like curvature so other controller can handle centripetal, GVF just handles m_d
data class GVFState(val tau: Vector2d, val error: Vector2d, val closestT: Double)
class GG(var kN: Double, val kD: Double, var maxVel: Double = 0.7, vararg val paths: Path) {
    var currentIndex = 0
    /**
     * Run every loop to receive desired positional velocities and heading direction
     */
    // TODO: should velocity control and other stuff such as curvature belong in here as an
    // integrated solution? or should it be separate and modularized
    fun update(p: Vector2d, v: Vector2d = Vector2d()): Pose2d {
        paths[currentIndex].let {
            val state = it.update(p)
            val distanceToEnd = p.distanceTo(it.current().endPose.position)
            return Pose2d(
                if (it.last() && distanceToEnd < it.current().constraints.decelDistance) {
                    (it.current().endPose.position - p).unit *
                        it.current().constraints.maxVel * distanceToEnd / it.current().constraints.decelDistance -
                        v * kD
                } else {
                    (state.tau + state.error * kN).unit * it.current().constraints.maxVel
                },
                Rotation2d(
                    if (it.current().constraints.tangentHeading) {
                        state.tau.polarAngle + if (it.current().constraints.reverseHeading) PI else 0.0
                    } else {
                        it.current().constraints.heading
                    }
                )
            )
        }
    }

    fun onTarget(p: Vector2d): Boolean {
        return p.distanceTo(paths[currentIndex].paths.last().endPose.position) < 0.25
    }

    fun error(p: Vector2d): Double {
        return p.distanceTo(paths[currentIndex].current().endPose.position)
    }
}