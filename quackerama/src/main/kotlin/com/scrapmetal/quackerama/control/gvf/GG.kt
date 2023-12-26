package com.scrapmetal.quackerama.control.gvf

import com.scrapmetal.quackerama.control.Pose2d
import com.scrapmetal.quackerama.control.Rotation2d
import com.scrapmetal.quackerama.control.Vector2d
import com.scrapmetal.quackerama.control.path.Path
import org.firstinspires.ftc.robotcore.external.navigation.Rotation

// TODO: include info like curvature so other controller can handle centripetal, GVF just handles m_d
data class GVFState(val tau: Vector2d, val error: Vector2d, val closestT: Double)
class GG(val path: Path, val kN: Double) {
    /**
     * Run every loop to receive dt velocity info
     *
     * @return returns desired global x, y velocity, as well as heading
     */
    fun update(p: Vector2d): Pose2d {
        val state = path.update(p)
        val distanceToEnd = p.distanceTo(path.current().endPose.position)
        return if (path.last() && distanceToEnd < path.current().constraints.decelDistance) {
            Pose2d(state.error.unit * distanceToEnd
                    / path.current().constraints.decelDistance,
                    Rotation2d(path.current().constraints.heading))
        } else {
            // WARNING: very dirty edge case here if you just want to keep continuous velocity
            Pose2d((state.tau + state.error * kN).unit,
                    Rotation2d(path.current().constraints.heading))
        }
    }
}