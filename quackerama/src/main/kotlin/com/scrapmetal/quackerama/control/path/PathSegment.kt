package com.scrapmetal.quackerama.control.path

import com.scrapmetal.quackerama.control.Pose2d
import com.scrapmetal.quackerama.control.Rotation2d
import com.scrapmetal.quackerama.control.Vector2d

interface PathSegment {
    val label: String
    val startPose: Pose2d
    val endPose: Pose2d

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
     * Calculate tangent of the closest point on the path relative to the input
     *
     * @param p current position
     * @return returns a rotation as the tangent is an angle
     */
    fun tauOf(p: Vector2d): Rotation2d

    /**
     * Calculate the error relative to the path
     *
     * Custom error functions like e^2 or 1/e can be used for desirable behaviors
     */
    fun eOf(p: Vector2d): Vector2d

    /**
     * Calculate the parametric input t that gives the closest point of the path
     */
    fun closestT(p: Vector2d): Double

    /**
     * Calculate the closest point of the path
     */
    fun closestPoint(p: Vector2d): Vector2d
}