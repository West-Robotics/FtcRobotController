package org.firstinspires.ftc.teamcode.seventh.robot.command

import com.qualcomm.robotcore.util.ElapsedTime
import com.scrapmetal.quackerama.control.controller.AsymTrapezoidMP
import org.firstinspires.ftc.teamcode.seventh.robot.hardware.Globals
import org.firstinspires.ftc.teamcode.seventh.robot.hardware.Globals.LIFT_HEIGHTS
import org.firstinspires.ftc.teamcode.seventh.robot.hardware.Robot
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.IntakeSubsystem
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.LiftSubsystem
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.OutputSubsystem
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.RobotState
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.RobotState.*
import java.lang.Math.toDegrees
import kotlin.math.absoluteValue
import kotlin.math.asin
import kotlin.math.pow
import kotlin.math.sqrt

class CycleCommand(val intake: IntakeSubsystem, val lift: LiftSubsystem, val out: OutputSubsystem) {
    companion object {
        public val TRUE_LIFT_HEIGHTS = doubleArrayOf(-0.2) + DoubleArray(5) { 2.6*it + 14.5 } + 10.9
    }
    var robotState = LOCK
    var lastState = robotState
    var height = 0
    private var armMP = AsymTrapezoidMP(-120.5, -120.5, 0.0, 0.0, 0.0)
    private var armTimer = ElapsedTime()
    private var liftMP = AsymTrapezoidMP(TRUE_LIFT_HEIGHTS[0], TRUE_LIFT_HEIGHTS[0], 0.0, 0.0, 0.0)
    private var liftTimer = ElapsedTime()

    // okay so to be clear all this does is update the states (and also other computations, hm maybe
    // i should change that) of each subsystem not actually cause any hardware change
    fun update(s: RobotState, h: Int, x: Double, high: Boolean = false) {
        val extension = x.coerceIn(0.0, 6.4) + if (Globals.AUTO) 4.3 else 4.0
        // val extension = x.coerceIn(0.0, 6.4) + 4.2
        // only update states on transitions
        if (s != robotState) {
            lastState = robotState
            if (!(s == LOCK && robotState == BACKDROP)) {
                armMP = AsymTrapezoidMP(
                    start = out.outState.arm,
                    end = when (s) {
                        EXTEND, SCORE, SCORE_L, SCORE_R
                            -> toDegrees(asin(extension * sqrt(3.0) / (2 * 9.25))) - 120
                        LOCK, BACKDROP -> -123.0
                        INTAKE, PRELOCK, SPIT -> -127.0
                        else -> -123.0
                    },
                    accel = 6000.0,
                    decel = -950.0, // old: -1400.0
                    v_max = 500.0
                )
                armTimer = ElapsedTime()
            }
            // killer
            // if (h != height) {
            liftMP = AsymTrapezoidMP(
                    start = lift.state.extension,
                    end = when (s) {
                        INTAKE, LOCK, PRELOCK, SPIT, ALIGN -> TRUE_LIFT_HEIGHTS[0]
                        BACKDROP, EXTEND, SCORE, SCORE_L, SCORE_R
                        -> TRUE_LIFT_HEIGHTS[h] +
                                0.5 * extension - 9.25 +
                                sqrt(9.25.pow(2) - (sqrt(3.0) / 2).pow(2) * extension.pow(2))
                    },
                    accel = if (Globals.AUTO) 100.0 else 150.0,
                    decel = if (s == LOCK) -1000.0 else -400.0, // old: -1400.0
                    v_max = 100.0
            )
            liftTimer = ElapsedTime()
            // }
            // println("${
            //     when (s) {
            //         INTAKE, LOCK, PRELOCK, SPIT, ALIGN -> TRUE_LIFT_HEIGHTS[0]
            //         BACKDROP, EXTEND, SCORE, SCORE_L, SCORE_R
            //         -> TRUE_LIFT_HEIGHTS[h] +
            //                 0.5 * extension - 9.25 +
            //                 sqrt(9.25.pow(2) - (sqrt(3.0) / 2).pow(2) * extension.pow(2))
            //     }
            // }")
            robotState = s
        }
        height = h
        // TODO: worry about only starting intake when lift reaches bottom
        if ((s == EXTEND || s == SCORE_L || s == SCORE || s == SCORE_R)) {
            if (!high) {
                // if we're already extended, track backdrop distance
                if (armTimer.seconds() > armMP.tTotal) {
                    out.update(robotState, toDegrees(asin(extension * sqrt(3.0) / (2 * 9.25))) - 120)
                } else {
                    // otherwise, continue extending via the mp
                    out.update(robotState, armMP.update(armTimer.seconds()).s)
                }
                if (liftTimer.seconds() > liftMP.tTotal) {
                    lift.update(
                            TRUE_LIFT_HEIGHTS[h] +
                                    0.5*extension - 9.25 +
                                    sqrt(9.25.pow(2) - (sqrt(3.0)/2).pow(2)*extension.pow(2)),
                            Robot.dt
                    )
                    // lift.update(TRUE_LIFT_HEIGHTS[h], Robot.dt)
                } else {
                    lift.update(liftMP.update(liftTimer.seconds()).s, Robot.dt)
                    // lift.update(TRUE_LIFT_HEIGHTS[h], Robot.dt)
                }
            } else {
                out.update(robotState, -5.0)
                lift.update(TRUE_LIFT_HEIGHTS[h], Robot.dt)
            }
        } else {
            if (robotState == LOCK && height == 0) {
                out.update(robotState, -123.0)
            } else {
                out.update(robotState, armMP.update(armTimer.seconds()).s)
            }
            lift.update(liftMP.update(liftTimer.seconds()).s, Robot.dt, s == LOCK)
        }
        intake.update(s)
    }

    fun armOnTarget(): Boolean {
        return (armMP.end - out.curArmAng).absoluteValue < 4.0
    }

    fun liftOnTarget(): Boolean {
        return (liftMP.end - lift.state.extension).absoluteValue < 0.5
    }

    fun endGoal() = armMP.end

    fun curAng() = out.curArmAng
}
