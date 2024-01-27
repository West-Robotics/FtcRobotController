package org.firstinspires.ftc.teamcode.seventh.opmode.test

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Servo
import com.scrapmetal.quackerama.hardware.QuackServo
import org.firstinspires.ftc.teamcode.seventh.robot.hardware.Globals

@TeleOp(name = "ILoveFingers")
class ILoveFingers : LinearOpMode() {
    override fun runOpMode() {
        val fingerLeft = QuackServo(hardwareMap, "fingerLeft", QuackServo.ModelPWM.GOBILDA_SPEED)
        val fingerRight = QuackServo(hardwareMap, "fingerRight", QuackServo.ModelPWM.GOBILDA_SPEED)

        fingerLeft.setDirection(Servo.Direction.FORWARD)
        fingerRight.setDirection(Servo.Direction.REVERSE)

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
            telemetry.update()
        }
    }
}