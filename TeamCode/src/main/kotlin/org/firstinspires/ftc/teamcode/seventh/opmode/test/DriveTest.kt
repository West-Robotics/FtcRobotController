package org.firstinspires.ftc.teamcode.seventh.opmode.test

import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.scrapmetal.quackerama.control.Pose2d
import com.scrapmetal.quackerama.control.Rotation2d
import com.scrapmetal.quackerama.control.Vector2d
import org.firstinspires.ftc.teamcode.seventh.robot.hardware.Robot
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.DriveSubsystem

@TeleOp(name = "Drive Test")
// to test heading pdf + distance sensors
class DriveTest : LinearOpMode() {
    override fun runOpMode() {
        val drive = DriveSubsystem(hardwareMap)
        val gamepad = GamepadEx(gamepad1)

        Robot.hardwareMap = hardwareMap
        val allHubs = hardwareMap.getAll(LynxModule::class.java)
        for (hub in allHubs) {
            hub.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL
        }

        waitForStart()
        while (opModeIsActive() && !isStopRequested) {
            for (hub in allHubs) {
                hub.clearBulkCache()
            }
            Robot.read(drive)

            drive.update(input = Pose2d(Vector2d(gamepad.leftY, -gamepad.leftX), Rotation2d(-gamepad.rightX)),
                         correcting = false,
                         fieldOriented = false,
                         dt = Robot.dt)

            Robot.write(drive)
            telemetry.addData("loop time", Robot.dt)
            telemetry.addData("wall left voltage", drive.wallLeft)
            telemetry.addData("wall right voltage", drive.wallRight)
            telemetry.update()
        }
    }
}