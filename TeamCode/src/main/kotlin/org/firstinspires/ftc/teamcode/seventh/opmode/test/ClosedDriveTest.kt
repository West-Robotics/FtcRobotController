package org.firstinspires.ftc.teamcode.seventh.opmode.test

import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.PerpetualCommand
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.scrapmetal.quackerama.control.Pose2d
import com.scrapmetal.quackerama.control.Rotation2d
import com.scrapmetal.quackerama.control.Vector2d
import org.firstinspires.ftc.teamcode.seventh.opmode.teleop.Gigapad
import org.firstinspires.ftc.teamcode.seventh.robot.hardware.Robot
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.DriveSubsystem
import kotlin.math.atan2

@TeleOp(name = "Closed Drive Test")
// to test heading pdf + distance sensors
class ClosedDriveTest : LinearOpMode() {
    override fun runOpMode() {
        val gigapad = Gigapad(gamepad1)
        Robot.init(hardwareMap, telemetry, gigapad, null)
        val drive = DriveSubsystem(hardwareMap)

        PerpetualCommand(InstantCommand({
            // drive.updateClosed(
            //     Pose2d(
            //         60*gigapad.leftY,
            //         30*-gigapad.leftX,
            //         atan2(-gigapad.rightY, gigapad.rightX)
            //     ),
            //     true
            // )
        })).schedule()
        waitForStart()

        drive.startIMUThread(this)
        while (opModeIsActive() && !isStopRequested) {
            Robot.read(drive)

            telemetry.addData("hz", 1 / Robot.getDt())
            telemetry.addData("x", drive.getPoseEstimate().position.x)
            telemetry.addData("y", drive.getPoseEstimate().position.y)
            telemetry.addData("heading", drive.getPoseEstimate().heading.theta)
            Robot.write(drive)
            // telemetry.addData("commanded angle", Rotation2d(gamepad.rightX, -gamepad.rightY).polarAngle)
            // telemetry.addData("current angle", drive.getPoseEstimate().heading.polarAngle)
            // telemetry.addData("error", Rotation2d(gamepad.rightX, -gamepad.rightY).polarAngle - drive.getPoseEstimate().heading.polarAngle)
            // telemetry.addData("d factor", drive.headingState.d * drive.headingState.dxdt)
            // telemetry.addData("output", drive.headingState.output)
        }
    }
}
