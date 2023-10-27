package org.firstinspires.ftc.teamcode.seventh.robot.command

import com.sfdev.assembly.state.StateMachine
import com.sfdev.assembly.state.StateMachineBuilder
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.OutputSubsystem

object AutoMachines {
    enum class AutoStates {
        INIT,
        DROP_OFF,
        TO_BACKDROP,
        SCORE,
        PARK,
    }
    fun getAutoMachine(drive: SampleMecanumDrive, cycle: CycleCommand, traj: RRTrajectories): StateMachine {
        return StateMachineBuilder()
            .state(AutoStates.INIT)
                .onEnter { drive.followTrajectorySequenceAsync(traj.dropOff) }
                .transition { !drive.isBusy }
            .state(AutoStates.DROP_OFF)
                // .onEnter { cycle.update(CycleCommand.CycleState.SPIT, OutputSubsystem.OutputState.LOCK) }
                .transitionTimed(3.0)
            .state(AutoStates.TO_BACKDROP)
                .onEnter { drive.followTrajectorySequenceAsync(traj.score) }
            //     .transition { !drive.isBusy }
            // .state(AutoStates.SCORE)
            //     .onEnter { cycle.update(CycleCommand.CycleState.READY, OutputSubsystem.OutputState.READY) }
            //     .transitionTimed(2.0)
            //     .onExit { cycle.update(CycleCommand.CycleState.LOCK, OutputSubsystem.OutputState.LOCK) }
            // .state(AutoStates.PARK)
            //     .onEnter { drive.followTrajectorySequenceAsync(traj.park) }
            .build()
    }
}