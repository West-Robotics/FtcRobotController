package org.firstinspires.ftc.teamcode.seventh.opmode.test

import com.arcrobotics.ftclib.command.PerpetualCommand
import com.arcrobotics.ftclib.command.RunCommand
import com.outoftheboxrobotics.photoncore.Photon
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.scrapmetal.quackerama.control.Pose2d
import org.firstinspires.ftc.teamcode.seventh.opmode.teleop.Gigapad
import org.firstinspires.ftc.teamcode.seventh.robot.hardware.Robot
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.DriveSubsystem
import java.lang.Math.toDegrees

@Photon
@TeleOp(name = "Drive Test")
// to test heading pdf + distance sensors
class DriveTest : LinearOpMode() {
    override fun runOpMode() {
        val gigapad = Gigapad(gamepad1)
        Robot.init(hardwareMap, telemetry, gigapad, null)
        val drive = DriveSubsystem(hardwareMap)

        PerpetualCommand(RunCommand({
            drive.updateOpen(
                Pose2d(gigapad.leftY, -gigapad.leftX, -gigapad.rightX),
                fieldOriented = false,
                headingPID = false,
            )
        })).schedule(false)
        waitForStart()

        drive.startIMUThread(this)
        while (opModeIsActive() && !isStopRequested) {
            Robot.read(drive)

            telemetry.addData("hz", 1 / Robot.getDt())
            telemetry.addData("x", drive.getPoseEstimate().position.x)
            telemetry.addData("y", drive.getPoseEstimate().position.y)
            telemetry.addData("heading", toDegrees(drive.getPoseEstimate().heading.theta))
            Robot.write(drive)
            // telemetry.addData("commanded angle", Rotation2d(gamepad.rightX, -gamepad.rightY).polarAngle)
            // telemetry.addData("current angle", drive.getPoseEstimate().heading.polarAngle)
            // telemetry.addData("error", Rotation2d(gamepad.rightX, -gamepad.rightY).polarAngle - drive.getPoseEstimate().heading.polarAngle)
            // telemetry.addData("d factor", drive.headingState.d * drive.headingState.dxdt)
            // telemetry.addData("output", drive.headingState.output)
        }
    }
}