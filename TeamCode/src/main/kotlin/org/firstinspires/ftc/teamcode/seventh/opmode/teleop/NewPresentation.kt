package org.firstinspires.ftc.teamcode.seventh.opmode.teleop

import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.ParallelCommandGroup
import com.arcrobotics.ftclib.command.RunCommand
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.arcrobotics.ftclib.command.WaitCommand
import com.outoftheboxrobotics.photoncore.Photon
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.seventh.robot.command.FallCommand
import org.firstinspires.ftc.teamcode.seventh.robot.command.IntakeCommand
import org.firstinspires.ftc.teamcode.seventh.robot.command.RiseCommand
import org.firstinspires.ftc.teamcode.seventh.robot.command.RobotState
import org.firstinspires.ftc.teamcode.seventh.robot.hardware.Robot
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.HangSubsystem
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.IntakeSubsystem
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.LiftSubsystem
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.OutputSubsystem

@Photon
@Autonomous(name = "New Presentation")
class NewPresentation : LinearOpMode() {
    override fun runOpMode() {
        Robot.init(hardwareMap, telemetry, null, null)
        val intake = IntakeSubsystem(hardwareMap)
        val lift = LiftSubsystem(hardwareMap)
        val output = OutputSubsystem(hardwareMap)
        val hang = HangSubsystem(hardwareMap)

        SequentialCommandGroup(
            InstantCommand({ intake.set(IntakeSubsystem.State.INTAKE) }),
            WaitCommand(1000),
            InstantCommand({ intake.set(IntakeSubsystem.State.STOP) }),
            InstantCommand({ intake.setHeight(5) }),
            WaitCommand(200),
            InstantCommand({ intake.setHeight(1) }),
            WaitCommand(200),
            InstantCommand({ intake.setHeight(5) }),
            WaitCommand(200),
            RiseCommand({0}, {OutputSubsystem.Roll.HORIZ}, lift, output),
            WaitCommand(1250),
            InstantCommand({ output.set(45.0); output.set(OutputSubsystem.Roll.VERT) }),
            WaitCommand(300),
            InstantCommand({ output.set(-45.0); output.set(OutputSubsystem.Roll.VERT_I) }),
            WaitCommand(500),
            InstantCommand({ output.set(45.0); output.set(OutputSubsystem.Roll.VERT) }),
            WaitCommand(500),
            InstantCommand({ output.set(0.0); output.set(OutputSubsystem.Roll.HORIZ) }),
            InstantCommand({ lift.set(7.5); hang.set(HangSubsystem.State.RAISE) }),
            WaitCommand(2000),
            ParallelCommandGroup(
                FallCommand(lift, output),
                RunCommand({ hang.set(HangSubsystem.State.LOWER) }).withTimeout(2000)
                    .andThen(InstantCommand({ hang.set(HangSubsystem.State.STOP) }))
            ),
        ).schedule()

        waitForStart()
        while (opModeIsActive()) {
            Robot.read(intake, lift, output)
            Robot.write(output, lift, intake, hang)
        }
    }
}