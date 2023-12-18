package com.scrapmetal.quackerama.control.path

import com.scrapmetal.quackerama.control.Pose2d
import com.scrapmetal.quackerama.control.Rotation2d
import com.scrapmetal.quackerama.control.Vector2d
import org.firstinspires.ftc.robotcore.external.navigation.Rotation

interface PathSegment {
    val label: String
    val startPose: Pose2d
    val endPose: Pose2d

    /**
     * Get all the info needed for GVF
     *
     * @return first vector is tau, second vector is raw error
     */
    fun update(p: Vector2d): Pair<Vector2d, Vector2d>

    /**
     * Calculate parametric output of the path
     *
     * @param t the parametric input t
     * @return returns a position/vector
     */
    operator fun invoke(t: Double): Vector2d

    /**
     * Calculate derivative of path
     *
     * @param t the parametric input t
     * @return returns a vector
     */
    fun dpdt(t: Double): Vector2d

    /**
     * Calculate the parametric input t that gives the closest point of the path
     */
    fun closestT(p: Vector2d): Double
}