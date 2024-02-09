package org.firstinspires.ftc.teamcode.seventh.opmode.test

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.ColorRangeSensor
import com.qualcomm.robotcore.hardware.Servo
import com.scrapmetal.quackerama.hardware.QuackServo
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.seventh.robot.hardware.Globals

@TeleOp(name = "ILoveFingers")
class ILoveFingers : LinearOpMode() {
    override fun runOpMode() {
        val fingerLeft = QuackServo(hardwareMap, "fingerLeft", QuackServo.ModelPWM.AXON_MICRO)
        val fingerRight = QuackServo(hardwareMap, "fingerRight", QuackServo.ModelPWM.AXON_MICRO)
        val colorLeft = hardwareMap.get(ColorRangeSensor::class.java, "colorLeft")
        val colorRight = hardwareMap.get(ColorRangeSensor::class.java, "colorRight")
        fingerLeft.setDirection(Servo.Direction.REVERSE)
        fingerRight.setDirection(Servo.Direction.FORWARD)

        waitForStart()
        while (opModeIsActive()) {
            if (gamepad1.a) {
                fingerLeft.setPosition(Globals.FINGER_CLOSE)
                fingerRight.setPosition(Globals.FINGER_CLOSE)
            } else if (gamepad1.b) {
                fingerLeft.setPosition(Globals.FINGER_OPEN)
                fingerRight.setPosition(Globals.FINGER_OPEN)
            }
            telemetry.addData("left finger", fingerLeft.getCommandedPosition())
            telemetry.addData("right finger", fingerRight.getCommandedPosition())
            telemetry.addData("left dist", colorLeft.getDistance(DistanceUnit.MM))
            telemetry.addData("right dist", colorRight.getDistance(DistanceUnit.MM))
            telemetry.update()
        }
    }
}