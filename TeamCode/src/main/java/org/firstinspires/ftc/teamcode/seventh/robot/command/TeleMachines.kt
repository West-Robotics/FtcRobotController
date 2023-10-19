package org.firstinspires.ftc.teamcode.seventh.robot.command

import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.sfdev.assembly.state.StateMachine
import com.sfdev.assembly.state.StateMachineBuilder
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.OutputSubsystem

class TeleMachines(primary: GamepadEx, secondary: GamepadEx) {
    val cycle = StateMachineBuilder()
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
    val out = StateMachineBuilder()
                .state(OutputSubsystem.OutputState.LOCK)
                .transition({ secondary.wasJustPressed(GamepadKeys.Button.A) }, OutputSubsystem.OutputState.INTAKE)
                .transition({ secondary.wasJustPressed(GamepadKeys.Button.DPAD_UP) }, OutputSubsystem.OutputState.READY)
                // this is actually just spit
                .transition({ secondary.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER) }, CycleCommand.CycleState.INTAKE)
                .state(OutputSubsystem.OutputState.INTAKE)
                .transition({ secondary.wasJustPressed(GamepadKeys.Button.B) }, OutputSubsystem.OutputState.LOCK)
                .state(OutputSubsystem.OutputState.READY)
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