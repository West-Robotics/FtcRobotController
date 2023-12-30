package org.firstinspires.ftc.teamcode.seventh.opmode.test

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.scrapmetal.quackerama.hardware.QuackMotor

@TeleOp(name = "Hang Test")
class HangTest : LinearOpMode() {
    override fun runOpMode() {
        val hang = QuackMotor(hardwareMap, "hang")

        waitForStart()
        while (opModeIsActive() && !isStopRequested) {
            hang.setPower((-gamepad1.left_trigger + gamepad1.right_trigger).toDouble())
        }
    }
}