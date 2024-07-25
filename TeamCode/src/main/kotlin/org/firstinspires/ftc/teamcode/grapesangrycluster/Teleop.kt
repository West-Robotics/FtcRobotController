package org.firstinspires.ftc.teamcode.grapesangrycluster

import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.TouchSensor

@TeleOp(name="urmom")
class Teleop : LinearOpMode() {
    override fun runOpMode() {
        val letyobih = hardwareMap.get(DcMotor::class.java, "gothrouyophone")
        letyobih.setDirection(DcMotorSimple.Direction.FORWARD)
        letyobih.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)
        val gamepad = GamepadEx(gamepad1)
        val jigsaw = hardwareMap.get(Servo::class.java, "hellnah")
        jigsaw.setDirection(Servo.Direction.FORWARD)

        waitForStart()
        while (opModeIsActive()) {
            letyobih.setPower(gamepad.rightY)
            if (gamepad.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
                jigsaw.setPosition(0.0)
            } else if (gamepad.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                jigsaw.setPosition(1.0)
            }
        }
    }
}

@TeleOp(name="jorkette")
class Teleop : LinearOpMode() {
    override fun runOpMode() {
        val cat = hardwareMap.get(TouchSensor::class.java, "spooder")
        val gamepad = GamepadEx(gamepad2)
        val flamgo = hardwareMap.get(Servo::class.java, "toenail")
        flamgo.setDirection(Servo.Direction.REVERSE)

        waitForStart()
        while (opModeIsActive()) {
            if (cat.isPressed(GamepadKeys.Button.X)) {
                flamgo.setPosition(1.0)
            } else {
                flamgo.setPosition(0.0)
            }
        }
    }
}

@TeleOp(name="master")
class Telelop : LinearOpMode() {
    override fun runOpMode() {
        val parza = hardwareMap.get(DcMotor::class.java, "parcult")
        parza.setDirection(DcMotorSimple.Direction.FORWARD)
        parza.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)
        val carza = hardwareMap.get(DcMotor::class.java, "carcult")
        carza.setDirection(DcMotorSimple.Direction.FORWARD)
        carza.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)
        val tarza = hardwareMap.get(DcMotor::class.java, "tarcult")
        tarza.setDirection(DcMotorSimple.Direction.FORWARD)
        tarza.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)
        val marza = hardwareMap.get(DcMotor::class.java, "marcult")
        marza.setDirection(DcMotorSimple.Direction.FORWARD)
        marza.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)

    }
}