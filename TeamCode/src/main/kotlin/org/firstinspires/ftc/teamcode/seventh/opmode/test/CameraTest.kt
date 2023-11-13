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
        var x = 0.0
        var y = 0.0
        var avgX = 0.0
        var avgY = 0.0
        var atagCount = 0

        while (opModeInInit()) {
            // for (det in hardware.aprilTag.detections) {
            //     atagCount++
            // }
            // telemetry.addData("prop position", hardware.propProcessor.getPosition())
            // telemetry.update()
        }
    }
}