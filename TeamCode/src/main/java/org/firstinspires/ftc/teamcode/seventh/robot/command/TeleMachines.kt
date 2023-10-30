package org.firstinspires.ftc.teamcode.seventh.robot.command

import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.sfdev.assembly.state.StateMachine
import com.sfdev.assembly.state.StateMachineBuilder
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.HangSubsystem
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.OutputSubsystem

object TeleMachines {
    // TODO: PLEASE VALIDATE THE STATE MACHINES
    fun getCycleMachine(primary: GamepadEx, secondary: GamepadEx): StateMachine {
        return StateMachineBuilder()
                .state(CycleCommand.CycleState.LOCK)
                        .transition({ secondary.wasJustPressed(GamepadKeys.Button.A) }, CycleCommand.CycleState.INTAKE)
                        .transition({ secondary.wasJustPressed(GamepadKeys.Button.DPAD_UP) }, CycleCommand.CycleState.READY)
                        .transition({ secondary.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER) }, CycleCommand.CycleState.SPIT)
                .state(CycleCommand.CycleState.INTAKE)
                        .transition({ secondary.wasJustPressed(GamepadKeys.Button.B) }, CycleCommand.CycleState.LOCK)
                        .transition({ secondary.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER) }, CycleCommand.CycleState.SPIT)
                .state(CycleCommand.CycleState.READY)
                        .transition({ secondary.wasJustPressed(GamepadKeys.Button.DPAD_DOWN) }, CycleCommand.CycleState.LOCK)
                .state(CycleCommand.CycleState.SPIT)
                        .transition({ secondary.wasJustPressed(GamepadKeys.Button.A) }, CycleCommand.CycleState.INTAKE)
                        .transition({ secondary.wasJustPressed(GamepadKeys.Button.B) }, CycleCommand.CycleState.LOCK)
                .build()
    }

    fun getOutMachine(primary: GamepadEx, secondary: GamepadEx): StateMachine {
        return StateMachineBuilder()
                .state(OutputSubsystem.OutputState.LOCK)
                        .transition({ secondary.wasJustPressed(GamepadKeys.Button.A) }, OutputSubsystem.OutputState.INTAKE)
                        .transition({ secondary.wasJustPressed(GamepadKeys.Button.DPAD_UP) }, OutputSubsystem.OutputState.READY)
                        // this is actually just spit
                        .transition({ secondary.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER) }, OutputSubsystem.OutputState.INTAKE)
                .state(OutputSubsystem.OutputState.INTAKE)
                        .transition({ secondary.wasJustPressed(GamepadKeys.Button.B) }, OutputSubsystem.OutputState.LOCK)
                .state(OutputSubsystem.OutputState.READY)
                        .onEnter { secondary.gamepad.rumbleBlips(1) }
                        .transition({ secondary.wasJustPressed(GamepadKeys.Button.DPAD_DOWN) }, OutputSubsystem.OutputState.LOCK)
                        .transition({ primary.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER) }, OutputSubsystem.OutputState.DROP)
                        .transition({ primary.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.9 }, OutputSubsystem.OutputState.DROP_L)
                        .transition({ primary.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.9 }, OutputSubsystem.OutputState.DROP_R)
                .state(OutputSubsystem.OutputState.DROP)
                        .transition({ secondary.wasJustPressed(GamepadKeys.Button.DPAD_DOWN) }, OutputSubsystem.OutputState.LOCK)
                        .transition({ primary.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER) }, OutputSubsystem.OutputState.READY)
                        .transition({ primary.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.9 }, OutputSubsystem.OutputState.DROP_L)
                        .transition({ primary.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.9 }, OutputSubsystem.OutputState.DROP_R)
                .state(OutputSubsystem.OutputState.DROP_L)
                        .transition({ secondary.wasJustPressed(GamepadKeys.Button.DPAD_DOWN) }, OutputSubsystem.OutputState.LOCK)
                        .transition({ primary.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER) }, OutputSubsystem.OutputState.DROP)
                        .transition({ primary.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.9 }, OutputSubsystem.OutputState.DROP_R)
                .state(OutputSubsystem.OutputState.DROP_R)
                        .transition({ secondary.wasJustPressed(GamepadKeys.Button.DPAD_DOWN) }, OutputSubsystem.OutputState.LOCK)
                        .transition({ primary.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER) }, OutputSubsystem.OutputState.DROP)
                        .transition({ primary.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.9 }, OutputSubsystem.OutputState.DROP_L)
                .build()
    }

    fun getHangMachine(primary: GamepadEx, secondary: GamepadEx): StateMachine {
        return StateMachineBuilder()
                .state(HangSubsystem.HangState.STOP)
                        .transition({ secondary.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.9}, HangSubsystem.HangState.RAISE)
                        .transition({ secondary.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.9}, HangSubsystem.HangState.LOWER)
                .state(HangSubsystem.HangState.RAISE)
                        .transition({ secondary.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) < 0.1 }, HangSubsystem.HangState.STOP)
                        .transition({ secondary.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.9}, HangSubsystem.HangState.LOWER)
                .state(HangSubsystem.HangState.LOWER)
                        .transition({ secondary.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) < 0.1}, HangSubsystem.HangState.STOP)
                        .transition({ secondary.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.9}, HangSubsystem.HangState.RAISE)
                .build()
    }
}