package org.firstinspires.ftc.teamcode.seventh.opmode.test

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.seventh.robot.hardware.Globals
import org.firstinspires.ftc.teamcode.seventh.robot.hardware.Hardware
import org.firstinspires.ftc.teamcode.seventh.robot.vision.Vision

@Autonomous(name = "CameraTest")
class CameraTest : LinearOpMode() {
    override fun runOpMode() {
        Globals.AUTO = true
        val vision = Vision(hardwareMap)
        // vision.disableProp()
        vision.enableAtag()

        while (opModeInInit()) {
            // telemetry.addData("fps", vision.getFps())
            // telemetry.addData("count", vision.getDets()?.size)
            // telemetry.addData("average dist", vision.getDistance())
            // vision.getDets()?.forEach { telemetry.addData("det ${it.id}", it.ftcPose.y) }
            telemetry.update()
        }
        return
    }
}