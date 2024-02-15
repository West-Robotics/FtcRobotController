package org.firstinspires.ftc.teamcode.seventh.robot.command

import com.qualcomm.robotcore.util.ElapsedTime
import com.scrapmetal.quackerama.control.controller.AsymTrapezoidMP
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
    private var timer = ElapsedTime()

    // okay so to be clear all this does is update the states (and also other computations, hm maybe
    // i should change that) of each subsystem not actually cause any hardware change
    fun update(s: RobotState, h: Int, x: Double) {
        val extension = x.coerceIn(0.0, 6.4) + 4.2
        // val extension = x.coerceIn(0.0, 6.4) + 4.2
        // only update states on transitions
        if (s != robotState) {
            lastState = robotState
            robotState = s
            if (!(s == LOCK && robotState == BACKDROP) && !(s == BACKDROP && robotState == LOCK)) {
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
                timer = ElapsedTime()
            }
        }
        height = h
        // TODO: worry about only starting intake when lift reaches bottom
        intake.update(robotState)
        lift.update(
            when (robotState) {
                INTAKE, LOCK, PRELOCK, SPIT, ALIGN -> LIFT_HEIGHTS[0]
                BACKDROP, EXTEND, SCORE, SCORE_L, SCORE_R
                    -> LIFT_HEIGHTS[height] +
                       0.5*extension - 9.25 +
                       sqrt(9.25.pow(2) - (sqrt(3.0)/2).pow(2)*extension.pow(2))
            },
            Robot.dt
        )
        if ((s == EXTEND || s == SCORE_L || s == SCORE || s == SCORE_R)) {
            // if we're already extended, track backdrop distance
            if (timer.seconds() > armMP.tTotal) {
                out.update(robotState, toDegrees(asin(extension * sqrt(3.0) / (2 * 9.25))) - 120)
            } else if (h == 0) {
                // if we're still inside the bot don't extend out
                // TODO: do based on position not desired height
                out.update(BACKDROP, armMP.update(timer.seconds()).s)
            } else {
                // otherwise, continue extending via the mp
                out.update(robotState, armMP.update(timer.seconds()).s)
            }
        } else {
            out.update(robotState, armMP.update(timer.seconds()).s)
        }
    }

    fun onTarget(): Boolean {
        return (armMP.end - out.curArmAng).absoluteValue < 4.0
    }

    fun endGoal() = armMP.end

    fun curAng() = out.curArmAng
}
