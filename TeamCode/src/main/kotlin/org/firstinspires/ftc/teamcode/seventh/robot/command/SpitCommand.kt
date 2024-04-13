package org.firstinspires.ftc.teamcode.seventh.robot.command

import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.IntakeSubsystem
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.OutputSubsystem

class SpitCommand(val intake: IntakeSubsystem, val output: OutputSubsystem) : CommandBase() {
    override fun initialize() {
        output.set(OutputSubsystem.Claw.NONE)
        intake.setHeight(5)
        intake.set(IntakeSubsystem.State.SPIT)
        addRequirements(intake, output)
    }

    override fun end(interrupted: Boolean) {
        if (interrupted) {
            intake.set(IntakeSubsystem.State.STOP)
            output.set(OutputSubsystem.Claw.BOTH)
        }
    }
}