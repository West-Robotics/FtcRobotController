package org.firstinspires.ftc.teamcode.seventh.opmode.auto

import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.ParallelCommandGroup
import com.arcrobotics.ftclib.command.RunCommand
import com.arcrobotics.ftclib.command.ScheduleCommand
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.arcrobotics.ftclib.command.WaitCommand
import com.arcrobotics.ftclib.command.WaitUntilCommand
import com.outoftheboxrobotics.photoncore.Photon
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.scrapmetal.quackerama.control.Pose2d
import com.scrapmetal.quackerama.control.gvf.GG
import org.firstinspires.ftc.teamcode.seventh.opmode.teleop.Gigapad
import org.firstinspires.ftc.teamcode.seventh.robot.command.FallCommand
import org.firstinspires.ftc.teamcode.seventh.robot.command.IntakeCommand
import org.firstinspires.ftc.teamcode.seventh.robot.command.RiseCommand
import org.firstinspires.ftc.teamcode.seventh.robot.hardware.Globals
import org.firstinspires.ftc.teamcode.seventh.robot.hardware.Robot
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.DriveSubsystem
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.IntakeSubsystem
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.LiftSubsystem
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.OutputSubsystem
import org.firstinspires.ftc.teamcode.seventh.robot.vision.GetPropPositionPipeline
import org.firstinspires.ftc.teamcode.seventh.robot.vision.Vision
import java.lang.Math.toDegrees
import kotlin.time.TimeSource

@Photon
@Autonomous(name =
"""
Red20Audience
"""
)
class Red20Audience : LinearOpMode() {
    override fun runOpMode() {
        Globals.AUTO = true
        Globals.alliance = Globals.Alliance.RED
        Globals.lane = Globals.Lane.LANE_3
        Globals.start = Globals.Start.AUDIENCE
        val drive = DriveSubsystem(hardwareMap)
        val intake = IntakeSubsystem(hardwareMap)
        val lift = LiftSubsystem(hardwareMap)
        val output = OutputSubsystem(hardwareMap)
        val vision = Vision(hardwareMap)
        val gamepad = Gigapad(gamepad1)
        Robot.init(hardwareMap, telemetry, gamepad, null)

        var stackCount = 5

        intake.setHeight(1)

        // vision.enableProp()
        drive.startIMUThread(this)
        vision.initProp()
        vision.enableProp()
        while (opModeInInit()) {
            telemetry.addData("prop", vision.getPropPosition())
            telemetry.update()
        }
        val propPos = vision.getPropPosition()
        vision.closeProp()
        vision.initAtag()
        val paths = AutoPositions(
                Globals.Alliance.RED,
                Globals.Start.AUDIENCE,
                Globals.Lane.LANE_3,
                Globals.YellowSide.LEFT,
                Globals.Stack.FAR,
                Globals.Park.INNER,
                propPos,
        )
        drive.setPoseEstimate(paths.initPose)
        val gg = GG(
                kN = 0.5,
                kD = 0.006,
                maxVel = 0.7,
                paths.purpleAudience,
                paths.yellow,
        )

        SequentialCommandGroup(
                // score purple
                InstantCommand({ gg.currentIndex = 0 }),
                ParallelCommandGroup(
                        RiseCommand({ 0 }, { OutputSubsystem.Roll.VERT_I }, lift, output),
                        SequentialCommandGroup(
                                WaitCommand(2000),
                                InstantCommand({ intake.setHeight(5) }),
                                InstantCommand({ gg.currentIndex++ }),
                                WaitCommand(2000),
                        ),
                ),
                // WaitUntilCommand { gg.onTarget(drive.getPoseEstimate().position) }.withTimeout(2000),
                // InstantCommand({ vision.getPosition(drive.getPoseEstimate().heading)?.let { drive.setPoseEstimate(Pose2d(it, drive.getPoseEstimate().heading)) } }),
                // score yellow
                // WaitUntilCommand { gg.onTarget(drive.getPoseEstimate().position) }.withTimeout(2000),
                InstantCommand({ output.set(OutputSubsystem.Claw.LEFT)} ),
                InstantCommand({ vision.getPosition(drive.getPoseEstimate().heading)?.let { drive.setPoseEstimate(Pose2d(it, drive.getPoseEstimate().heading)) } }),
                WaitCommand(250),
                InstantCommand({ gg.currentIndex++ }),
                FallCommand(lift, output),
                // intake from stacks
                WaitUntilCommand { drive.getPoseEstimate().position.x < -50.0 }.withTimeout(3000),
                ScheduleCommand(
                        IntakeCommand(intake, output, lift, true).withTimeout(6000),
                        WaitCommand(3000).andThen(InstantCommand({ intake.setHeight(4) }))
                ),
                WaitUntilCommand { gg.onTarget(drive.getPoseEstimate().position) }.withTimeout(3000),
                WaitCommand(2000),
                // score on backdrop
                InstantCommand({ gg.currentIndex++ }),
                WaitUntilCommand { drive.getPoseEstimate().position.x > 5.0 }.withTimeout(3000),
                RiseCommand({ 1 }, { OutputSubsystem.Roll.VERT_I }, lift, output)
                        .andThen(InstantCommand({ output.set(-45.0) })),
                WaitUntilCommand { gg.onTarget(drive.getPoseEstimate().position) }.withTimeout(2000),
                InstantCommand({ output.set(OutputSubsystem.Claw.NONE)} ),
                InstantCommand({ vision.getPosition(drive.getPoseEstimate().heading)?.let { drive.setPoseEstimate(Pose2d(it, drive.getPoseEstimate().heading)) } }),
                WaitCommand(250),
                FallCommand(lift, output),
        ).schedule()

        while (opModeIsActive()) {
            // update all subsystems
            Robot.read(intake, output, lift, drive)

            val input = gg.update(drive.getPoseEstimate().position, drive.getVelocity().position)
            drive.updateOpen(input, fieldOriented = true, headingPID = true)

            telemetry.addData("hz", 1 / Robot.getDt())
            telemetry.addData("error", gg.error(drive.getPoseEstimate().position))
            telemetry.addData("onTarget", gg.onTarget(drive.getPoseEstimate().position))
            telemetry.addData("x", drive.getPoseEstimate().position.x)
            telemetry.addData("y", drive.getPoseEstimate().position.y)
            Robot.write(drive, lift, output, intake)
        }
    }
}
