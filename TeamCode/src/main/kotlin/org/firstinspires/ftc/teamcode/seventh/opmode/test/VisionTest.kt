package org.firstinspires.ftc.teamcode.seventh.opmode.test

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.scrapmetal.quackerama.control.Rotation2d
import org.firstinspires.ftc.teamcode.seventh.robot.hardware.Globals
import org.firstinspires.ftc.teamcode.seventh.robot.vision.Vision

@Autonomous(name = "Vision Test")
class VisionTest : LinearOpMode() {
    override fun runOpMode() {
        Globals.AUTO = true
        Globals.alliance = Globals.Alliance.BLUE
        val vision = Vision(hardwareMap)
        // vision.closeProp()
        vision.initAtag()

        while (opModeInInit()) {
            telemetry.addData("fps", vision.getFps())
            telemetry.addData("count", vision.getDets()?.size)
            vision.getPosition(Rotation2d()).let {
                telemetry.addData("x", it?.x)
                telemetry.addData("y", it?.y)
            }
            // telemetry.addData("average dist", vision.getDistance())
            vision.getDets()?.forEach { telemetry.addLine("det ${it.id}") }
            telemetry.update()
        }
    }
}