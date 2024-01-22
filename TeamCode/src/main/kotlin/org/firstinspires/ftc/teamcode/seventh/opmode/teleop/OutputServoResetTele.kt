package org.firstinspires.ftc.teamcode.seventh.opmode.teleop

import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.ColorRangeSensor
import com.qualcomm.robotcore.hardware.Servo
import com.scrapmetal.quackerama.hardware.QuackServo
import org.firstinspires.ftc.teamcode.seventh.robot.hardware.Globals

@TeleOp(name = "Reset Output Servos")
class OutputServoResetTele : LinearOpMode() {

    override fun runOpMode() {
        val armLeft = QuackServo(hardwareMap, "armLeft", QuackServo.ModelPWM.AXON_MAX)
        val armRight = QuackServo(hardwareMap, "armRight", QuackServo.ModelPWM.AXON_MAX)
        val pitch = QuackServo(hardwareMap, "pitch", QuackServo.ModelPWM.GENERIC)
        val fingerLeft = QuackServo(hardwareMap, "fingerLeft", QuackServo.ModelPWM.GOBILDA_SPEED)
        val fingerRight = QuackServo(hardwareMap, "fingerRight", QuackServo.ModelPWM.GOBILDA_SPEED)

        armLeft.setDirection(Servo.Direction.REVERSE)
        armRight.setDirection(Servo.Direction.FORWARD)
        pitch.setDirection(Servo.Direction.REVERSE)
        fingerLeft.setDirection(Servo.Direction.FORWARD)
        fingerRight.setDirection(Servo.Direction.REVERSE)

        val gamepad = GamepadEx(gamepad1)
        var config = 1

        waitForStart()
        while (opModeIsActive() && !isStopRequested) {
            when {
                gamepad1.a -> config = 0
                gamepad1.x -> config = 1
                gamepad1.b -> config = 2
            }
            when (config) {
                0 -> {
                    armLeft.setPosition(-120.0)  { theta: Double -> theta/180.0 + 1.0 }
                    armRight.setPosition(-120.0) { theta: Double -> theta/180.0 + 1.0 }
                    pitch.setPosition(60.0)  { theta: Double -> theta/270.0 }
                    fingerLeft.setPosition(Globals.FINGER_CLOSE)
                    fingerRight.setPosition(Globals.FINGER_CLOSE)
                    telemetry.addLine("Lock Position")
                }
                1 -> {
                    armLeft.setPosition(0.0)  { theta: Double -> theta/180.0 + 1.0 }
                    armRight.setPosition(0.0) { theta: Double -> theta/180.0 + 1.0 }
                    pitch.setPosition(0.0)  { theta: Double -> theta/270.0 }
                    fingerLeft.setPosition(Globals.FINGER_CLOSE)
                    fingerRight.setPosition(Globals.FINGER_CLOSE)
                    telemetry.addLine("Parallel to Ground")
                }
                2 -> {
                    armLeft.setPosition(-30.0)  { theta: Double -> theta/180.0 + 1.0 }
                    armRight.setPosition(-30.0) { theta: Double -> theta/180.0 + 1.0 }
                    pitch.setPosition(90.0)  { theta: Double -> theta/270.0 }
                    fingerLeft.setPosition(Globals.FINGER_OPEN)
                    fingerRight.setPosition(Globals.FINGER_OPEN)
                    telemetry.addLine("Perpendicular to Slides")
                }
            }
            telemetry.update()
        }
    }
}