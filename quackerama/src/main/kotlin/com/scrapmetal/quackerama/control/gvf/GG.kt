package com.scrapmetal.quackerama.control.gvf

import com.scrapmetal.quackerama.control.Pose2d
import com.scrapmetal.quackerama.control.Rotation2d
import com.scrapmetal.quackerama.control.Vector2d
import com.scrapmetal.quackerama.control.path.Path
import org.firstinspires.ftc.robotcore.external.navigation.Rotation
import kotlin.math.pow

// TODO: include info like curvature so other controller can handle centripetal, GVF just handles m_d
data class GVFState(val tau: Vector2d, val error: Vector2d, val closestT: Double)
class GG(val kN: Double, val kA: Double = 0.0, vararg val paths: Path) {
    var currentIndex = 0
    /**
     * Run every loop to receive desired positional velocities and heading direction
     */
    // TODO: should velocity control and other stuff such as curvature belong in here as an
    // integrated solution? or should it be separate and modularized
    fun update(p: Vector2d, v: Vector2d = Vector2d()): Pose2d {
        val state = paths[currentIndex].update(p)
        val distanceToEnd = p.distanceTo(paths[currentIndex].current().endPose.position)
        return if (paths[currentIndex].last() && distanceToEnd < paths[currentIndex].current().constraints.decelDistance) {
            // println("decel thing" + v*kA*v.mag.pow(2)/(2*distanceToEnd))
            Pose2d((paths[currentIndex].current().endPose.position-p).unit
                    * distanceToEnd / paths[currentIndex].current().constraints.decelDistance
                    - v*kA*v.mag.pow(2)/(2*distanceToEnd),
                    Rotation2d(paths[currentIndex].current().constraints.heading))
        } else {
            Pose2d((state.tau + state.error * kN).unit,
                    Rotation2d(paths[currentIndex].current().constraints.heading))
        }
    }
}