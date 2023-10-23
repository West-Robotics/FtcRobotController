package org.firstinspires.ftc.teamcode.seventh.opmode.auto

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.seventh.robot.command.CycleCommand
import org.firstinspires.ftc.teamcode.seventh.robot.command.TeleMachines
import org.firstinspires.ftc.teamcode.seventh.robot.hardware.Hardware
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.HangSubsystem
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.IntakeSubsystem
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.LiftSubsystem
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.OutputSubsystem
import kotlin.math.abs
import kotlin.math.pow
import kotlin.math.sign
import kotlin.time.DurationUnit
import kotlin.time.TimeSource

@Autonomous(name =
    "🟦🟥🟥🟥🟦🟩🟩🟩🟦🟫🟫🟫🟦🟧🟧🟧🟦🟨🟨🟨🟦\n" +
    "🟥🟥🟦🟦🟩🟩🟦🟦🟫🟫🟦🟦🟧🟧🟦🟦🟨🟨🟦🟦🟦\n" +
    "🟥🟥🟥🟥🟩🟩🟩🟩🟫🟫🟫🟫🟧🟧🟧🟧🟨🟨🟨🟨🟦\n" +
    "🟦🟥🟦🟥🟦🟩🟦🟩🟦🟫🟦🟫🟦🟧🟦🟧🟦🟨🟦🟨🟦\n" +
    "🟦🟨🟨🟨🟦🟥🟥🟥🟦🟩🟩🟩🟦🟫🟫🟫🟦🟧🟧🟧🟦\n" +
    "🟨🟨🟦🟦🟥🟥🟦🟦🟩🟩🟦🟦🟫🟫🟦🟦🟧🟧🟦🟦🟦\n" +
    "🟨🟨🟨🟨🟥🟥🟥🟥🟩🟩🟩🟩🟫🟫🟫🟫🟧🟧🟧🟧🟦\n" +
    "🟦🟨🟦🟨🟦🟥🟦🟥🟦🟩🟦🟩🟦🟫🟦🟫🟦🟧🟦🟧🟦\n" +
    "🟦🟧🟧🟧🟦🟨🟨🟨🟦🟥🟥🟥🟦🟩🟩🟩🟦🟫🟫🟫🟦\n" +
    "🟧🟧🟦🟦🟨🟨🟦🟦🟥🟥🟦🟦🟩🟩🟦🟦🟫🟫🟦🟦🟦\n" +
    "🟧🟧🟧🟧🟨🟨🟨🟨🟥🟥🟥🟥🟩🟩🟩🟩🟫🟫🟫🟫🟦\n" +
    "🟦🟧🟦🟧🟦🟨🟦🟨🟦🟥🟦🟥🟦🟩🟦🟩🟦🟫🟦🟫🟦\n" +
    "🟦🟫🟫🟫🟦🟧🟧🟧🟦🟨🟨🟨🟦🟥🟥🟥🟦🟩🟩🟩🟦\n" +
    "🟫🟫🟦🟦🟧🟧🟦🟦🟨🟨🟦🟦🟥🟥🟦🟦🟩🟩🟦🟦🟦\n" +
    "🟫🟫🟫🟫🟧🟧🟧🟧🟨🟨🟨🟨🟥🟥🟥🟥🟩🟩🟩🟩🟦\n" +
    "🟦🟫🟦🟫🟦🟧🟦🟧🟦🟨🟦🟨🟦🟥🟦🟥🟦🟩🟦🟩🟦\n" +
    "🟦🟩🟩🟩🟦🟫🟫🟫🟦🟧🟧🟧🟦🟨🟨🟨🟦🟥🟥🟥🟦\n" +
    "🟩🟩🟦🟦🟫🟫🟦🟦🟧🟧🟦🟦🟨🟨🟦🟦🟥🟥🟦🟦🟦\n" +
    "🟩🟩🟩🟩🟫🟫🟫🟫🟧🟧🟧🟧🟨🟨🟨🟨🟥🟥🟥🟥🟦\n" +
    "🟦🟩🟦🟩🟦🟫🟦🟫🟦🟧🟦🟧🟦🟨🟦🟨🟦🟥🟦🟥🟦\n" +
    "🟦🟦🟦🟦🟦🟦🟦🟦🟦🟦🟦🟦🟦🟦🟦🟦🟦🟦🟦🟦🟦")
class SussyAuto : LinearOpMode() {
    override fun runOpMode() {
        val allHubs = hardwareMap.getAll(LynxModule::class.java)
        for (hub in allHubs) {
            hub.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL
        }
        val hardware = Hardware.getInstance(hardwareMap)
        val drive = SampleMecanumDrive(hardware, hardwareMap)
        val intake = IntakeSubsystem(hardware)
        val lift = LiftSubsystem(hardware)
        val out = OutputSubsystem(hardware)

        val primary = GamepadEx(gamepad1)
        val secondary = GamepadEx(gamepad2)
        val timeSource = TimeSource.Monotonic
        var loopTime: TimeSource.Monotonic.ValueTimeMark = timeSource.markNow()

        // is this bad? maybe switch to a singleton
        val cycle = CycleCommand(intake, lift, out)
        val cycleMachine = TeleMachines(primary, secondary).cycle
        val outMachine = TeleMachines(primary, secondary).out
        val hangMachine = TeleMachines(primary, secondary).hang

        cycleMachine.start()
        outMachine.start()
        hardware.read(intake, lift, out)
        cycle.update(cycleMachine.state as CycleCommand.CycleState, outMachine.state as OutputSubsystem.OutputState)
        hardware.write(intake, lift, out)
        telemetry.addLine("waiting for start")
        telemetry.update()
        waitForStart()

        while (opModeIsActive() && !isStopRequested) {
            for (hub in allHubs) {
                hub.clearBulkCache()
            }
            val loop = timeSource.markNow()
            val dt = (loop - loopTime).toDouble(DurationUnit.MILLISECONDS)
            loopTime = loop

            // update state machines
            cycleMachine.update();
            outMachine.update();
            hangMachine.update();

            // update all subsystems
            hardware.read(intake, lift, out);
            cycle.update(cycleMachine.state as CycleCommand.CycleState, outMachine.state as OutputSubsystem.OutputState)
            hardware.write(intake, lift, out);

            telemetry.addData("pivot pos", hardware.pivot.position);
            telemetry.addData("left pos", hardware.fingerLeft.position);
            telemetry.addData("right pos", hardware.fingerRight.position)
            telemetry.addData("lift dist", lift.distance)
            telemetry.addData("list state", lift.state)
            telemetry.addData("cycle state", cycleMachine.state as CycleCommand.CycleState)
            telemetry.addData("out state", outMachine.state as OutputSubsystem.OutputState)
            telemetry.addData("hz ", 1000 / dt)
            telemetry.update()
        }
        hardware.stop()
    }
}