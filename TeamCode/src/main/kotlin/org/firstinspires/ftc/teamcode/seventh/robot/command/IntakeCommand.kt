package org.firstinspires.ftc.teamcode.seventh.robot.command

import androidx.core.view.accessibility.AccessibilityViewCommand.ScrollToPositionArguments
import com.arcrobotics.ftclib.command.CommandBase
import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.arcrobotics.ftclib.command.WaitCommand
import com.arcrobotics.ftclib.command.WaitUntilCommand
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.IntakeSubsystem
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.LiftSubsystem
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.OutputSubsystem

/**
 * LOCK -> INTAKE
 */
class IntakeCommand(val intake: IntakeSubsystem, val output: OutputSubsystem, val lift: LiftSubsystem) : SequentialCommandGroup() {
    init {
        addCommands(
            InstantCommand({ output.set(OutputSubsystem.Claw.NONE) }),
            // InstantCommand({ output.set(OutputSubsystem.Arm.INTAKE) }),
            InstantCommand({ output.set(OutputSubsystem.Pitch.IN) }),
            InstantCommand({ intake.set(IntakeSubsystem.State.INTAKE) }),
            InstantCommand({ lift.set(0.25) }),
            WaitUntilCommand { intake.filledL && intake.filledR },
            WaitCommand(500),
        )
        addRequirements(intake, output)
    }

    override fun end(interrupted: Boolean) {
        SequentialCommandGroup(
            InstantCommand({ intake.set(IntakeSubsystem.State.STOP) }),
            InstantCommand({ output.set(OutputSubsystem.Arm.IN) }),
            InstantCommand({ lift.set(0.0) }),
            WaitCommand(500),
            InstantCommand({ output.set(OutputSubsystem.Claw.BOTH) }),
        ).schedule(true)
    }
}