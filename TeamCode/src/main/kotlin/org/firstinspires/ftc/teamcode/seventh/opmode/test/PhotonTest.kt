package org.firstinspires.ftc.teamcode.seventh.opmode.test

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.outoftheboxrobotics.photoncore.Photon
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.util.ElapsedTime
import kotlin.math.sin

@Photon
@TeleOp(name = "Photon Test")
class PhotonTest : LinearOpMode() {
    override fun runOpMode() {
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
        val lf = hardwareMap.get(DcMotor::class.java, "leftFront")
        val lr = hardwareMap.get(DcMotor::class.java, "leftRear")
        val rr = hardwareMap.get(DcMotor::class.java, "rightRear")
        val rf = hardwareMap.get(DcMotor::class.java, "rightFront")

        waitForStart()
        val time = ElapsedTime()
        var lastTime = System.nanoTime()
        while (opModeIsActive() && !isStopRequested) {
            val currentTime = System.nanoTime()
            val dt = currentTime - lastTime
            lastTime = currentTime
            lf.power = sin(time.seconds())
            lr.power = sin(time.seconds())
            rr.power = sin(time.seconds())
            rf.power = sin(time.seconds())
            telemetry.addData("hz", 1e9 / dt)
            telemetry.update()
        }
    }
}