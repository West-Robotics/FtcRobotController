package org.firstinspires.ftc.teamcode.seventh.opmode.auto

import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.ParallelCommandGroup
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

@Photon
@Autonomous(name =
"""
Blue20Backdrop
"""
)
class Blue20Backdrop : LinearOpMode() {
    override fun runOpMode() {
        Globals.AUTO = true
        Globals.alliance = Globals.Alliance.BLUE
        Globals.lane = Globals.Lane.LANE_3
        Globals.start = Globals.Start.BACKDROP
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
        output.set(OutputSubsystem.Claw.BOTH)
        while (opModeInInit()) {
            // telemetry.addData("prop", vision.getPropPosition())
            telemetry.update()

            Robot.write(output)
        }
        val propPos = vision.getPropPosition()
        vision.closeProp()
        vision.initAtag()
        val paths = AutoPositions(
            Globals.Alliance.BLUE,
            Globals.Start.BACKDROP,
            Globals.Lane.LANE_3,
            Globals.YellowSide.LEFT,
            Globals.Stack.FAR,
            Globals.Park.OUTER,
            propPos,
        )
        drive.setPoseEstimate(paths.initPose)
        val gg = GG(
            kN = 0.5,
            kD = 0.006,
            maxVel = 0.6,
            paths.purpleBackdrop,
            paths.yellow,
            paths.parkPath,
        )

        SequentialCommandGroup(
                // score purple
                InstantCommand({ gg.currentIndex = 0 }),
                ParallelCommandGroup(
                    WaitCommand(2000).andThen(RiseCommand({ 0 }, { OutputSubsystem.Roll.HORIZ }, lift, output)),
                    SequentialCommandGroup(
                        WaitCommand(3000),
                        InstantCommand({ intake.setHeight(5) }),
                        WaitCommand(2000),
                        InstantCommand({ gg.currentIndex++ }),
                        WaitCommand(3000),
                    ),
                ),
                InstantCommand({ output.set(OutputSubsystem.Claw.NONE) }),
                // InstantCommand({ vision.getPosition(drive.getPoseEstimate().heading)?.let { drive.setPoseEstimate(Pose2d(it, drive.getPoseEstimate().heading)) } }),
                WaitCommand(500),
                InstantCommand({ gg.currentIndex++ }),
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
