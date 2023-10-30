package org.firstinspires.ftc.teamcode.seventh.opmode.test

import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple

import org.firstinspires.ftc.teamcode.seventh.robot.hardware.Hardware
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.OutputSubsystem

@TeleOp(name = "OuttakeTest")
class OuttakeTest : LinearOpMode() {
    @Override
    override fun runOpMode() {
        val hardware = Hardware(hardwareMap)
        val out = OutputSubsystem(hardware)

        val gamepad = GamepadEx(gamepad1)

//        hardware.pivot.setDirection(Servo.Direction.FORWARD)
//        hardware.pivot.setPwmRange(new PwmControl.PwmRange(500, 2500))
//        hardware.fingerLeft.setDirection(Servo.Direction.FORWARD)
//        hardware.fingerLeft.setPwmRange(new PwmControl.PwmRange(500, 2500))
//        hardware.fingerRight.setDirection(Servo.Direction.REVERSE)
//        hardware.fingerRight.setPwmRange(new PwmControl.PwmRange(500, 2500))
//        double position = 0.5

        waitForStart()
        hardware.intake.setDirection(DcMotorSimple.Direction.FORWARD)
        hardware.intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT)
        hardware.intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)

        while (opModeIsActive() && !isStopRequested()) {
//            if (gamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) < 0.5) {
//                position = 0.5
//            } else {
//                position = gamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)
//            }
//            hardware.pivot.setPosition(gamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER))
//            hardware.fingerLeft.setPosition(gamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)+0.4)
//            hardware.fingerRight.setPosition(gamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)+0.34)
            hardware.read(out)
            if (gamepad.getButton(GamepadKeys.Button.A)) {
                out.update(OutputSubsystem.OutputState.INTAKE)
            } else if (gamepad.getButton(GamepadKeys.Button.B)) {
                out.update(OutputSubsystem.OutputState.LOCK)
            } else if (gamepad.getButton(GamepadKeys.Button.X)) {
                out.update(OutputSubsystem.OutputState.INTERMEDIARY)
            } else if (gamepad.getButton(GamepadKeys.Button.Y)) {
                out.update(OutputSubsystem.OutputState.READY)
            } else if (gamepad.getButton(GamepadKeys.Button.DPAD_DOWN)) {
                out.update(OutputSubsystem.OutputState.DROP)
            }
            if (gamepad.getButton(GamepadKeys.Button.LEFT_BUMPER)) {
                hardware.intake.setPower(-1.0)
            } else if (gamepad.getButton(GamepadKeys.Button.RIGHT_BUMPER)) {
                hardware.intake.setPower(0.0)
            } else if (gamepad.getButton(GamepadKeys.Button.DPAD_LEFT)) {
                hardware.intake.setPower(0.2)
            }
            hardware.write(out)
            telemetry.addData("pivot pos", hardware.pivot.getPosition())
            telemetry.addData("left pos", hardware.fingerLeft.getPosition())
            telemetry.addData("right pos", hardware.fingerRight.getPosition())
            telemetry.update()
        }
    }
}
