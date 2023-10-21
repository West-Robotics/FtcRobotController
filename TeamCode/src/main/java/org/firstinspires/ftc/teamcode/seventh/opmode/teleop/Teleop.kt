package org.firstinspires.ftc.teamcode.seventh.opmode.teleop

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.seventh.robot.command.CycleCommand
import org.firstinspires.ftc.teamcode.seventh.robot.command.CycleCommand.CycleState
import org.firstinspires.ftc.teamcode.seventh.robot.command.TeleMachines
import org.firstinspires.ftc.teamcode.seventh.robot.hardware.Hardware
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.IntakeSubsystem
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.LiftSubsystem
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.OutputSubsystem
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.OutputSubsystem.OutputState
import kotlin.math.abs
import kotlin.math.pow
import kotlin.math.sign
import kotlin.time.DurationUnit
import kotlin.time.TimeSource

@TeleOp(name = "SussyOp")
public class Teleop : LinearOpMode() {

    // run blocking?
    override fun runOpMode() {
        // IDEAS:
        // automatically flip driving direction
        // lock robot to face exactly backdrop
        // mecanum feedforward

        val hardware = Hardware(hardwareMap)
        val drive = SampleMecanumDrive(hardware, hardwareMap)
        val intake = IntakeSubsystem(hardware)
        val lift = LiftSubsystem(hardware)
        val out = OutputSubsystem(hardware)

        val primary = GamepadEx(gamepad1)
        val secondary = GamepadEx(gamepad2)
        var x = 0.0
        var y = 0.0
        var turn = 0.0
        // change per millisecond
        val SLEW_RATE = 6.0*1e-3
        val timeSource = TimeSource.Monotonic
        var loopTime: TimeSource.Monotonic.ValueTimeMark = timeSource.markNow()

        // is this bad? maybe switch to a singleton
        val cycle = CycleCommand(intake, lift, out)
        val cycleMachine = TeleMachines(primary, secondary).cycle
        val outMachine = TeleMachines(primary, secondary).out

        cycleMachine.start()
        outMachine.start()
        hardware.read(intake, lift, out)
        cycle.update(cycleMachine.state as CycleState, outMachine.state as OutputState)
        hardware.write(intake, lift, out)
        waitForStart()

        while (opModeIsActive() && !isStopRequested) {
            val loop = timeSource.markNow()
            val dt = (loop - loopTime).toDouble(DurationUnit.MILLISECONDS)
            loopTime = loop
            // remember to convert to seconds (multiply by 10^-3) for slew rate lmao

            primary.readButtons();
            secondary.readButtons();

            // update state machines
            cycleMachine.update();
            outMachine.update();

            // update all subsystems
            hardware.read(intake, lift, out);
            cycle.update(cycleMachine.state as CycleState, outMachine.state as OutputState)
            hardware.write(intake, lift, out);

            // only change dt powers by at max the slew rate
            x += (primary.leftY - x).let { if (abs(it) < SLEW_RATE*dt) it else sign(it)*SLEW_RATE*dt }
            y += (-primary.leftX - y).let { if (abs(it) < SLEW_RATE*dt) it else sign(it)*SLEW_RATE*dt }
            turn += (-primary.rightX - turn).let { if (abs(it) < SLEW_RATE*dt) it else sign(it)*SLEW_RATE*dt }

            val multiplier = when {
                cycleMachine.state == CycleState.READY              -> 0.5
                intake.state == IntakeSubsystem.IntakeState.INTAKE  -> 0.5
                primary.isDown(GamepadKeys.Button.LEFT_BUMPER)      -> 0.5
                else                                                -> 1.0
            }
            drive.setWeightedDrivePower(Pose2d(x.pow(3)*multiplier, y.pow(3)*multiplier, (turn/1.5).pow(3)*multiplier))

            telemetry.addData("pivot pos", hardware.pivot.position);
            telemetry.addData("left pos", hardware.fingerLeft.position);
            telemetry.addData("right pos", hardware.fingerRight.position)
            telemetry.addData("lift dist", lift.distance)
            telemetry.addData("list state", lift.state)
            telemetry.addData("hz ", 1000 / dt)
            telemetry.update()
        }
    }
}
