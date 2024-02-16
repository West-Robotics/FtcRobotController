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
    var robotState = LOCK
    var lastState = robotState
    var height = 0
    private var armMP = AsymTrapezoidMP(-120.5, -120.5, 0.0, 0.0, 0.0)
    private var armTimer = ElapsedTime()
    private var liftMP = AsymTrapezoidMP(LIFT_HEIGHTS[0], LIFT_HEIGHTS[0], 0.0, 0.0, 0.0)
    private var liftTimer = ElapsedTime()

    // okay so to be clear all this does is update the states (and also other computations, hm maybe
    // i should change that) of each subsystem not actually cause any hardware change
    fun update(s: RobotState, h: Int, x: Double) {
        val extension = x.coerceIn(0.0, 6.4) + if (Globals.AUTO) 4.2 else 4.0
        // val extension = x.coerceIn(0.0, 6.4) + 4.2
        // only update states on transitions
        if (s != robotState) {
            lastState = robotState
            if (!(s == LOCK && robotState == BACKDROP)) {
                armMP = AsymTrapezoidMP(
                    start = out.outState.arm,
                    end = when (s) {
                        EXTEND, SCORE, SCORE_L, SCORE_R
                            -> if (h != 0) toDegrees(asin(extension * sqrt(3.0) / (2 * 9.25))) - 120 else -120.5
                        LOCK, BACKDROP -> -120.5
                        INTAKE, PRELOCK, SPIT -> -127.0
                        else -> -123.0
                    },
                    accel = 6000.0,
                    decel = -1000.0, // old: -1400.0
                    v_max = 500.0
                )
                armTimer = ElapsedTime()
            }
            liftMP = AsymTrapezoidMP(
                    start = lift.state.extension,
                    end = when (s) {
                        INTAKE, LOCK, PRELOCK, SPIT, ALIGN -> LIFT_HEIGHTS[0]
                        BACKDROP, EXTEND, SCORE, SCORE_L, SCORE_R
                        -> LIFT_HEIGHTS[height] +
                                0.5*extension - 9.25 +
                                sqrt(9.25.pow(2) - (sqrt(3.0)/2).pow(2)*extension.pow(2))
                    },
                    accel = 400.0,
                    decel = if (robotState == LOCK) -1000000.0 else -400.0, // old: -1400.0
                    v_max = 100.0
            )
            liftTimer = ElapsedTime()
            robotState = s
        }
        height = h
        // TODO: worry about only starting intake when lift reaches bottom
        if ((s == EXTEND || s == SCORE_L || s == SCORE || s == SCORE_R)) {
            // if we're already extended, track backdrop distance
            if (armTimer.seconds() > armMP.tTotal) {
                out.update(robotState, toDegrees(asin(extension * sqrt(3.0) / (2 * 9.25))) - 120)
            } else {
                // otherwise, continue extending via the mp
                out.update(robotState, armMP.update(armTimer.seconds()).s)
            }
            if (liftTimer.seconds() > liftMP.tTotal) {
                lift.update(
                    LIFT_HEIGHTS[height] +
                    0.5*extension - 9.25 +
                    sqrt(9.25.pow(2) - (sqrt(3.0)/2).pow(2)*extension.pow(2)),
                    Robot.dt
                )
            } else {
                lift.update(liftMP.update(liftTimer.seconds()).s, Robot.dt)
            }
        } else {
            if (robotState == LOCK && height == 0) {
                out.update(robotState, -123.0)
            } else {
                out.update(robotState, armMP.update(armTimer.seconds()).s)
            }
            lift.update(liftMP.update(liftTimer.seconds()).s, Robot.dt)
        }
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
