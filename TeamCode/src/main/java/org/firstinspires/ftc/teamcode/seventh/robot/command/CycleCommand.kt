package org.firstinspires.ftc.teamcode.seventh.robot.command

import org.firstinspires.ftc.teamcode.seventh.robot.hardware.Globals
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.IntakeSubsystem
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.LiftSubsystem
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.OutputSubsystem

class CycleCommand(val intake: IntakeSubsystem, val lift: LiftSubsystem, val out: OutputSubsystem) {
    enum class CycleState {
        INTAKE,
        LOCK,
        READY,
        GROUND,
        SPIT,
    }

    // okay so to be clear all this does is update the states (and also other computations, hm maybe
    // i should change that) of each subsystem not actually cause any hardware change
    fun update(cs: CycleState, os: OutputSubsystem.OutputState) {
        // TODO: worry about only starting intake when lift reaches bottom
        with(when (cs) {
            CycleState.INTAKE -> Pair(IntakeSubsystem.IntakeState.INTAKE, LiftSubsystem.LiftState.DOWN)
            CycleState.LOCK -> Pair(IntakeSubsystem.IntakeState.STOP, LiftSubsystem.LiftState.DOWN)
            CycleState.READY -> Pair(IntakeSubsystem.IntakeState.STOP, LiftSubsystem.LiftState.UP)
            CycleState.SPIT -> Pair(IntakeSubsystem.IntakeState.SPIT, LiftSubsystem.LiftState.DOWN)
            else -> Pair(IntakeSubsystem.IntakeState.STOP, LiftSubsystem.LiftState.DOWN)
        }) {
            intake.update(this.first)
            lift.update(this.second)
        }

        out.update(when {
            lift.distance < when (lift.state) { LiftSubsystem.LiftState.UP -> Globals.INTERMEDIARY_ZONE_1
                                                LiftSubsystem.LiftState.DOWN -> Globals.INTERMEDIARY_ZONE_3 }
                -> when (os) {
                            OutputSubsystem.OutputState.INTAKE -> OutputSubsystem.OutputState.INTAKE
                            // always assume we want to lock unless specifically requested to open
                            else -> OutputSubsystem.OutputState.LOCK }
            lift.distance < Globals.INTERMEDIARY_ZONE_2
                -> OutputSubsystem.OutputState.INTERMEDIARY
            Globals.INTERMEDIARY_ZONE_2 <= lift.distance
                -> when (os) {
                        OutputSubsystem.OutputState.LOCK,
                        OutputSubsystem.OutputState.INTAKE -> OutputSubsystem.OutputState.INTERMEDIARY
                        else -> os }
            else -> OutputSubsystem.OutputState.LOCK
        })
    }
}
