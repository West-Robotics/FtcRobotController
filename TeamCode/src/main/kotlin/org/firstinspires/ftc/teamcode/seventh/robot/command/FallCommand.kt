package org.firstinspires.ftc.teamcode.seventh.robot.command

import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.RunCommand
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.arcrobotics.ftclib.command.WaitCommand
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.LiftSubsystem
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.OutputSubsystem
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.OutputSubsystem.*

/**
 * RISEN -> LOCK
 */
class FallCommand(lift: LiftSubsystem, output: OutputSubsystem) : SequentialCommandGroup() {
    init {
        addCommands(
            RunCommand({ lift.set(5.0) }).interruptOn { lift.onTarget() },
            InstantCommand({ output.set(State(Arm.IN, 0.0, Pitch.IN, Roll.HORIZ, Claw.BOTH)) }),
            WaitCommand(1000),
            InstantCommand({ lift.set(0.0) }),
            InstantCommand({ output.set(0.0) }),
        )
        addRequirements(lift, output)
    }
}
