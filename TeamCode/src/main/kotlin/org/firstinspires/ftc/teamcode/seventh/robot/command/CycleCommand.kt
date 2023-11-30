package org.firstinspires.ftc.teamcode.seventh.robot.command

import org.firstinspires.ftc.teamcode.seventh.robot.hardware.Globals
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.IntakeSubsystem
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.LiftSubsystem
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.OutputSubsystem
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.RobotState

class CycleCommand(val intake: IntakeSubsystem, val lift: LiftSubsystem, val out: OutputSubsystem) {
    var robotState = RobotState.LOCK
    var lastState = robotState
    var height = 0

    // okay so to be clear all this does is update the states (and also other computations, hm maybe
    // i should change that) of each subsystem not actually cause any hardware change
    fun update(s: RobotState, h: Int) {
        // only update states on transitions
        if (s != robotState) {
            lastState = robotState
            robotState = s
        }
        height = h
        // TODO: worry about only starting intake when lift reaches bottom
        intake.update(robotState)
        lift.update(when (robotState) {
            RobotState.INTAKE, RobotState.LOCK, RobotState.SPIT -> 0
            RobotState.BACKDROP, RobotState.EXTEND -> height
            RobotState.GROUND -> -1
            RobotState.SCORE, RobotState.SCORE_L, RobotState.SCORE_R
                -> when (lastState) { RobotState.GROUND -> -1
                                      RobotState.EXTEND -> height
                                      else -> height}
        })
        out.update(robotState, 0.0)
    }
}