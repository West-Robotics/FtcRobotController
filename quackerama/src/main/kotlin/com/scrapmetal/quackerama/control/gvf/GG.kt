package com.scrapmetal.quackerama.control.gvf

import com.scrapmetal.quackerama.control.Pose2d
import com.scrapmetal.quackerama.control.Rotation2d
import com.scrapmetal.quackerama.control.Vector2d
import com.scrapmetal.quackerama.control.path.Path

// TODO: include info like curvature so other controller can handle centripetal, GVF just handles m_d
data class GVFState(val tau: Vector2d, val error: Vector2d, val closestT: Double)
class GG(val kN: Double, val kD: Double = 0.0, val maxVel: Double = 1.0, vararg val paths: Path) {
    var currentIndex = 0
    /**
     * Run every loop to receive desired positional velocities and heading direction
     */
    // TODO: should velocity control and other stuff such as curvature belong in here as an
    // integrated solution? or should it be separate and modularized
    fun update(p: Vector2d, v: Vector2d = Vector2d()): Pose2d {
        val state = paths[currentIndex].update(p)
        val distanceToEnd = p.distanceTo(paths[currentIndex].current().endPose.position)
        val distanceToBeg = p.distanceTo(paths[currentIndex].current().startPose.position)
        return if (paths[currentIndex].last() && distanceToEnd < paths[currentIndex].current().constraints.decelDistance) {
            // println("decel thing" + v*kA*v.mag.pow(2)/(2*distanceToEnd))
            Pose2d((paths[currentIndex].current().endPose.position - p).unit * 0.8
                    * distanceToEnd / paths[currentIndex].current().constraints.decelDistance
                    - v * kD,
                    // - v*kA*v.mag.pow(2)/(2*distanceToEnd),
                    Rotation2d(paths[currentIndex].current().constraints.heading))
        } else {
            Pose2d((state.tau + state.error * kN).unit * 0.8,
                    Rotation2d(paths[currentIndex].current().constraints.heading))
            // else if (paths[currentIndex].first() && distanceToBeg < paths[currentIndex].current().constraints.decelDistance) {       }
            //   Pose2d((paths[currentIndex].current().startPose.position-p).unit*0.8                                               }
            //           * distanceToBeg / (paths[currentIndex].current().constraints.decelDistance/4),
            //           // - v*kA*v.mag.pow(2)/(2*distanceToEnd),
            //           Rotation2d(paths[currentIndex].current().constraints.heading))
        }
    }

    fun onTarget(v: Vector2d): Boolean {
        return v.distanceTo(paths[currentIndex].paths.last().endPose.position) < 0.8
    }

    fun error(v: Vector2d): Double {
        return v.distanceTo(paths[currentIndex].current().endPose.position)
    }
}