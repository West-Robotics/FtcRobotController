package org.firstinspires.ftc.teamcode.seventh.opmode.test

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.seventh.robot.hardware.Globals
import org.firstinspires.ftc.teamcode.seventh.robot.hardware.Hardware

@Autonomous(name = "CameraTest")
class CameraTest : LinearOpMode() {
    override fun runOpMode() {
        Globals.AUTO = true
        val hardware = Hardware.getInstance(hardwareMap)

        while (opModeInInit()) {
            telemetry.addData("prop position", hardware.propPosition.getPosition())
            telemetry.update()
        }
    }
}