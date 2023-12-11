package org.firstinspires.ftc.teamcode.seventh.opmode.test

import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.seventh.robot.hardware.Robot
import org.firstinspires.ftc.teamcode.seventh.robot.hardware.Robot.hardwareMap
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.OutputSubsystem
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.RobotState

@TeleOp(name="PitchTest")
class PivotTest : LinearOpMode() {
    @Override
    override fun runOpMode() {
        val pivot = hardwareMap.get(Servo::class.java, "pitch")

        waitForStart()

        while (opModeIsActive() && !isStopRequested) {
            pivot.position = gamepad1.left_trigger.toDouble()
        }
    }
}