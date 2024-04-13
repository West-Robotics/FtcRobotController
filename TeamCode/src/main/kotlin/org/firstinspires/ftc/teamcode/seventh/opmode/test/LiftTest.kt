package org.firstinspires.ftc.teamcode.seventh.opmode.test;

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.arcrobotics.ftclib.command.CommandScheduler
import com.arcrobotics.ftclib.command.InstantCommand
import com.outoftheboxrobotics.photoncore.Photon
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.seventh.opmode.teleop.Gigapad
import org.firstinspires.ftc.teamcode.seventh.robot.hardware.Robot
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.LiftSubsystem

@Photon
@TeleOp(name = "LiftTest")
class LiftTest : LinearOpMode() {
    override fun runOpMode() {
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
        val gigapad = Gigapad(gamepad1)
        Robot.init(hardwareMap, telemetry, gigapad, null)
        val lift = LiftSubsystem(hardwareMap)

        gigapad.inc.whenActive(InstantCommand({ lift.raise() }))
        gigapad.dec.whenActive(InstantCommand({ lift.lower() }))
        waitForStart()

        while (opModeIsActive() && !isStopRequested) {
            Robot.read(lift)
            telemetry.addData("level", lift.boardLevel)
            telemetry.addData("extension", lift.extension)
            telemetry.addData("grounded", lift.grounded)
            telemetry.addData("overCurrent", lift.overCurrent)
            telemetry.addData("power", lift.power)
            telemetry.addData("hz", 1/Robot.getDt())
            Robot.write(lift)
        }
    }
}
