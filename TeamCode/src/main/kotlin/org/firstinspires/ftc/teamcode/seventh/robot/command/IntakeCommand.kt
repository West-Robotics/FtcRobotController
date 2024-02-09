package org.firstinspires.ftc.teamcode.seventh.robot.command

import com.arcrobotics.ftclib.command.CommandBase
import com.arcrobotics.ftclib.command.InstantCommand
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.IntakeSubsystem
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.LiftSubsystem
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.OutputSubsystem

class IntakeCommand(
    val intake: IntakeSubsystem,
    val lift: LiftSubsystem,
    val output: OutputSubsystem
) : InstantCommand({ })