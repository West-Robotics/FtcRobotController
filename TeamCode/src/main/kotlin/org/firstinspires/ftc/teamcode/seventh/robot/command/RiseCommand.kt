package org.firstinspires.ftc.teamcode.seventh.robot.command

import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.RunCommand
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.arcrobotics.ftclib.command.WaitCommand
import com.arcrobotics.ftclib.command.WaitUntilCommand
import org.firstinspires.ftc.teamcode.seventh.robot.hardware.Globals
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
            InstantCommand({ lift.set(if (!Globals.AUTO) 5.0 else 5.5) }),
            WaitUntilCommand { lift.onTarget() }.withTimeout(2000),
            InstantCommand({ output.set(State(Arm.BACKDROP, 0.0, Pitch.OUT, Roll.HORIZ, Claw.BOTH)) }),
            WaitCommand(800),
            InstantCommand({ output.set(getRoll()) }),
            InstantCommand({ lift.set(getHeight().toDouble()) }),
            InstantCommand({ lift.set(getRoll()) }),
            // InstantCommand({ print("height: ${getHeight()}\nroll: ${getRoll()}")})
        )
        addRequirements(lift, output)
    }
}