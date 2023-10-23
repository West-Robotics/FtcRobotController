package org.firstinspires.ftc.teamcode.util.control.path

import org.firstinspires.ftc.teamcode.util.control.Pose2d
import org.firstinspires.ftc.teamcode.util.control.Vector2d

class Line(override val label:      String,
           override val startPose:  Pose2d,
           override val endPose:    Pose2d) : Path {

    override infix fun closestTo(p: Vector2d): Vector2d {
        return Vector2d()
    }
}