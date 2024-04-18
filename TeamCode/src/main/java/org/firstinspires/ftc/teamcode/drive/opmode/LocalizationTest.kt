package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.drive.Drive
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.scrapmetal.quackerama.control.Pose2d
import com.scrapmetal.quackerama.control.Rotation2d

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.seventh.opmode.teleop.Gigapad
import org.firstinspires.ftc.teamcode.seventh.robot.hardware.Globals;
import org.firstinspires.ftc.teamcode.seventh.robot.hardware.Robot
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.DriveSubsystem
import org.firstinspires.ftc.teamcode.seventh.robot.vision.Vision;
import java.lang.Math.toDegrees
import kotlin.math.pow
import kotlin.math.sign

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@Photon
@TeleOp(group = "drive")
public class LocalizationTest : LinearOpMode() {
    override fun runOpMode() {
        Globals.alliance = Globals.Alliance.RED
        val gigapad = Gigapad(gamepad1)
        Robot.init(hardwareMap, telemetry, gigapad, null)
        val drive = DriveSubsystem(hardwareMap);
        val vision = Vision(hardwareMap);

        vision.initAtag()

        drive.startIMUThread(this);
        waitForStart();
        drive.setPoseEstimate(Pose2d())

        while (!isStopRequested) {
            Robot.read(drive)
            drive.updateOpen(
                Pose2d(
                    gigapad.leftY,
                    -gigapad.leftX,
                    -gigapad.rightX.let { it.sign*it.pow(2) },
                ),
                fieldOriented = false,
                headingPID = false,
            )

            if (gigapad.wasJustPressed(GamepadKeys.Button.A)) {
                vision.getPosition(drive.getPoseEstimate().heading)?.let {
                    drive.setPoseEstimate(Pose2d(it, drive.getPoseEstimate().heading))
                }
            }

            telemetry.addData("hz", 1 / Robot.getDt());
            with (drive.getPoseEstimate()) {
                telemetry.addData("x", position.x);
                telemetry.addData("y", position.y);
                telemetry.addData("heading", toDegrees(heading.theta))
            }
            Robot.write(drive)
        }
    }
}
