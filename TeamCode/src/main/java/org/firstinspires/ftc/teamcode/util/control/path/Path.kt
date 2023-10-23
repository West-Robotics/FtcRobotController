package org.firstinspires.ftc.teamcode.util.control.path

import org.firstinspires.ftc.teamcode.util.control.Pose2d
import org.firstinspires.ftc.teamcode.util.control.Vector2d

interface Path {
    val label: String
    val startPose: Pose2d
    val endPose: Pose2d
    infix fun closestTo(p: Vector2d): Vector2d
}