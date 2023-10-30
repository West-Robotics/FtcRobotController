package org.firstinspires.ftc.teamcode.seventh.opmode.test;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;
import com.sun.source.doctree.StartElementTree;
import org.firstinspires.ftc.teamcode.seventh.robot.hardware.Globals

import org.firstinspires.ftc.teamcode.seventh.robot.hardware.Hardware;
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.LiftSubsystem;

@TeleOp(name = "LiftTest")
class LiftTest : LinearOpMode() {
    override fun runOpMode() {
        val hardware = Hardware(hardwareMap)
        val gamepad = GamepadEx(gamepad1)

        hardware.liftLeft.direction = DcMotorSimple.Direction.FORWARD
        hardware.liftLeft.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        hardware.liftLeft.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        hardware.liftRight.direction = DcMotorSimple.Direction.REVERSE
        hardware.liftRight.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        hardware.liftRight.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        hardware.liftLeftEnc.setDirection(Motor.Direction.FORWARD)
        hardware.liftLeftEnc.setDistancePerPulse(Globals.LIFT_DISTANCE_PER_PULSE)
        hardware.liftLeftEnc.reset()
        hardware.liftRightEnc.setDirection(Motor.Direction.FORWARD)
        hardware.liftRightEnc.setDistancePerPulse(Globals.LIFT_DISTANCE_PER_PULSE)
        hardware.liftRightEnc.reset()
        waitForStart();

        while (opModeIsActive() && !isStopRequested) {
            val power = gamepad.leftY/2.0
            hardware.liftLeft.power = power;
            hardware.liftRight.power = power;
            telemetry.addData("power", power);
            telemetry.addData("dist", hardware.liftLeftEnc.distance);
            telemetry.update();
        }
    }
}
