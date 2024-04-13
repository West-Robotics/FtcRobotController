package org.firstinspires.ftc.teamcode.seventh.robot.command

import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.RunCommand
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.arcrobotics.ftclib.command.WaitCommand
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.LiftSubsystem
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.OutputSubsystem
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.OutputSubsystem.*
import java.util.function.IntSupplier

/**
 * LOCK -> RISEN
 */
class RiseCommand(
        getHeight: () -> Int,
        getRoll: () -> Roll,
        lift: LiftSubsystem,
        output: OutputSubsystem,
) : SequentialCommandGroup() {
    init {
        addCommands(
            InstantCommand({ output.set(Claw.BOTH) }),
            // InstantCommand({ output.set(State(Arm.PULLOUT, 0.0, Pitch.PULLOUT, Roll.HORIZ, Claw.BOTH)) }),
            WaitCommand(50),
            RunCommand({ lift.set(5.0) }).interruptOn { lift.onTarget() },
            InstantCommand({ output.set(State(Arm.BACKDROP, 0.0, Pitch.OUT, getRoll(), Claw.BOTH)) }),
            WaitCommand(1000),
            InstantCommand({ lift.set(getHeight().toDouble()) }),
            InstantCommand({ lift.set(getRoll()) }),
            // InstantCommand({ print("height: ${getHeight()}\nroll: ${getRoll()}")})
        )
        addRequirements(lift, output)
    }
}