package com.scrapmetal.quackerama.control.path

import com.scrapmetal.quackerama.control.Pose2d
import com.scrapmetal.quackerama.control.Rotation2d
import com.scrapmetal.quackerama.control.Vector2d
import com.scrapmetal.quackerama.control.gvf.GVFState
import org.firstinspires.ftc.robotcore.external.navigation.Rotation

/**
 * Constraints on robot movement throughout the path
 *
 * @param decelDistance distance before the endpoint to begin decelerating
 * @param heading heading to maintain in radians
 */
data class MovementConstraints(val decelDistance: Double = 0.0, val heading: Double = 0.0)
interface PathSegment {
    val label: String
    val startPose: Pose2d
    val endPose: Pose2d
    val constraints: MovementConstraints

    /**
     * Get all the info needed for GVF
     */
    fun update(p: Vector2d): GVFState

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