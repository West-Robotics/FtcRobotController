package com.scrapmetal.quackerama.control.gvf

import com.scrapmetal.quackerama.control.Pose2d
import com.scrapmetal.quackerama.control.Rotation2d
import com.scrapmetal.quackerama.control.Vector2d
import com.scrapmetal.quackerama.control.controller.PIDF
import com.scrapmetal.quackerama.control.path.Path

// TODO: include info like curvature so other controller can handle centripetal, GVF just handles m_d
data class GVFState(val tau: Vector2d, val error: Vector2d, val closestT: Double)
class GG(var kN: Double, val pidf: PIDF, var maxVel: Double = 0.8, vararg val paths: Path) {
    var currentIndex = 0
    /**
     * Run every loop to receive desired positional velocities and heading direction
     */
    // TODO: should velocity control and other stuff such as curvature belong in here as an
    // integrated solution? or should it be separate and modularized
    fun update(p: Vector2d, dt: Double): Pose2d {
        val state = paths[currentIndex].update(p)
        val distanceToEnd = p.distanceTo(paths[currentIndex].current().endPose.position)
        val distanceToBeg = p.distanceTo(paths[currentIndex].current().startPose.position)
        return Pose2d(
            if (paths[currentIndex].last() && distanceToEnd < paths[currentIndex].current().constraints.decelDistance) {
                (paths[currentIndex].current().endPose.position - p).unit *
                        pidf.update(distanceToEnd, 0.0, dt)
            } else {
                (state.tau + state.error * kN).unit * pidf.maxMagnitude
            },
            Rotation2d(
                if (paths[currentIndex].current().constraints.tangentHeading) {
                    state.tau.polarAngle
                } else {
                    paths[currentIndex].current().constraints.heading
                }
            )
        )
    }

    fun onTarget(p: Vector2d): Boolean {
        return p.distanceTo(paths[currentIndex].paths.last().endPose.position) < 0.8
    }

    fun error(p: Vector2d): Double {
        return p.distanceTo(paths[currentIndex].current().endPose.position)
    }
}