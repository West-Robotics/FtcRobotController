package org.firstinspires.ftc.teamcode.seventh.robot.command

import com.qualcomm.robotcore.util.ElapsedTime
import com.scrapmetal.quackerama.control.controller.AsymTrapezoidMP
import org.firstinspires.ftc.teamcode.seventh.robot.hardware.Globals.LIFT_HEIGHTS
import org.firstinspires.ftc.teamcode.seventh.robot.hardware.Robot
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.IntakeSubsystem
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.LiftSubsystem
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.OutputSubsystem
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.RobotState
import java.lang.Math.toDegrees
import kotlin.math.asin
import kotlin.math.pow
import kotlin.math.sqrt

class CycleCommand(val intake: IntakeSubsystem, val lift: LiftSubsystem, val out: OutputSubsystem) {
    var robotState = RobotState.LOCK
    var lastState = robotState
    var height = 0
    private var armMP = AsymTrapezoidMP(-125.0, -125.0, 0.0, 0.0, 0.0)
    private var timer = ElapsedTime()

    // okay so to be clear all this does is update the states (and also other computations, hm maybe
    // i should change that) of each subsystem not actually cause any hardware change
    fun update(s: RobotState, h: Int, x: Double) {
        val extension = x.coerceIn(0.0, 11.5)
        // only update states on transitions
        // if (s != robotState) {
            lastState = robotState
            robotState = s
            // armMP = AsymTrapezoidMP(
            //     start = out.outState.arm,
            //     end = when(s) {
            //         RobotState.EXTEND, RobotState.SCORE, RobotState.SCORE_L, RobotState.SCORE_R
            //             // TOOD: replace 10.0 with actual length
            //             -> toDegrees(asin(horizDist*sqrt(3.0)/(2*9.5))) - 120
            //         RobotState.LOCK, RobotState.BACKDROP -> -120.0
            //         RobotState.INTAKE, RobotState.SPIT -> -125.0
            //         else -> -120.0
            //     },
            //     accel = 6000.0,
            //     decel = -1400.0,
            //     v_max = 500.0
            // )
        val armPos = when(s) {
            RobotState.EXTEND, RobotState.SCORE, RobotState.SCORE_L, RobotState.SCORE_R
            -> toDegrees(asin(extension*sqrt(3.0)/(2*9.25))) - 120
            RobotState.LOCK, RobotState.BACKDROP -> -120.0
            RobotState.INTAKE, RobotState.SPIT -> -125.0
            else -> -120.0
        }
            timer = ElapsedTime()
        // }
        height = h
        // TODO: worry about only starting intake when lift reaches bottom
        intake.update(robotState)
        lift.update(
            when (robotState) {
                RobotState.INTAKE, RobotState.LOCK, RobotState.SPIT -> LIFT_HEIGHTS[0]
                RobotState.BACKDROP -> LIFT_HEIGHTS[height]
                RobotState.EXTEND, RobotState.SCORE, RobotState.SCORE_L, RobotState.SCORE_R
                    // TODO: replace 10.0 with actual length
                    -> LIFT_HEIGHTS[height] +
                       0.5*extension - 9.25 +
                       sqrt(9.25.pow(2) - (sqrt(3.0)/2).pow(2)*extension.pow(2))
            },
            Robot.dt
        )
        // out.update(robotState, armMP.update(timer.seconds()).s)
        out.update(robotState, armPos)
    }
}