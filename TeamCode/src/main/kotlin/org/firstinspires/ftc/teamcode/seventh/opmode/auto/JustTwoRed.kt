package org.firstinspires.ftc.teamcode.seventh.opmode.auto

import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.configuration.LynxConstants
import com.scrapmetal.quackerama.control.Pose2d
import com.scrapmetal.quackerama.control.gvf.GG
import com.sfdev.assembly.state.StateMachineBuilder
import org.firstinspires.ftc.teamcode.seventh.robot.command.CycleCommand
import org.firstinspires.ftc.teamcode.seventh.robot.hardware.Globals
import org.firstinspires.ftc.teamcode.seventh.robot.hardware.Robot
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.DriveSubsystem
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.IntakeSubsystem
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.LiftSubsystem
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.OutputSubsystem
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.RobotState
import org.firstinspires.ftc.teamcode.seventh.robot.vision.GetPropPositionPipeline
import org.firstinspires.ftc.teamcode.seventh.robot.vision.Vision
import java.lang.Math.toDegrees
import kotlin.time.DurationUnit
import kotlin.time.TimeSource

@Autonomous(name =
"""
RED JUST TWO
"""
)
class JustTwoRed : LinearOpMode() {
    enum class AutoStates {
        TO_PURPLE,
        DROP_PURPLE,
        TO_YELLOW,
        BACKDROP,
        EXTEND,
        SCORE,
        RETRACT,
        LOCK,
        PARK,
    }
    override fun runOpMode() {
        Globals.AUTO = true
        Globals.side = Globals.Side.RED
        Robot.hardwareMap = hardwareMap
        val allHubs = hardwareMap.getAll(LynxModule::class.java)
        var CONTROL_HUB: LynxModule = allHubs[0]
        for (hub in allHubs) {
            hub.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL
            if (hub.isParent() && LynxConstants.isEmbeddedSerialNumber(hub.serialNumber)) {
                CONTROL_HUB = hub
            }
        }
        val drive = DriveSubsystem(hardwareMap)
        val intake = IntakeSubsystem(hardwareMap)
        val lift = LiftSubsystem(hardwareMap)
        val output = OutputSubsystem(hardwareMap)
        val vision = Vision(hardwareMap)
        val gamepad = GamepadEx(gamepad1)

        val timeSource = TimeSource.Monotonic
        var loopTime: TimeSource.Monotonic.ValueTimeMark = timeSource.markNow()

        val cycle = CycleCommand(intake, lift, output)

        var state = RobotState.LOCK
        var height = 0
        intake.setHeight(1)

        Robot.read(intake, output)
        cycle.update(state, height, 0.0)
        Robot.write(intake, output)

        vision.enableProp()
        // waitForStart()
        var cycleCount = -1

        drive.startIMUThread(this)
        while (opModeInInit()) {
            CONTROL_HUB.clearBulkCache()
            Robot.read(drive)
            telemetry.addData("x", drive.getPoseEstimate().position.u)
            telemetry.addData("y", drive.getPoseEstimate().position.v)
            telemetry.addData("heading", drive.getPoseEstimate().heading.polarAngle)
            telemetry.addData("prop", vision.getPropPosition())
            telemetry.update()
        }

        val paths = AutoPaths(
                Globals.Side.RED,
                Globals.Start.CLOSE,
                Globals.Lane.LANE_2,
                Globals.YellowSide.LEFT,
                Globals.Stack.CLOSE,
                Globals.Park.INNER,
                vision.getPropPosition(),
        )
        vision.disableProp()
        drive.setPoseEstimate(paths.initPose)
        val gg = GG(
                kN = 0.4,
                kA = 0.0001, // was 0.0004
                paths.purple,
                paths.yellow,
                paths.parkPath,
        )
        val autoMachine = StateMachineBuilder()
                .state(AutoStates.TO_PURPLE)
                    .transitionTimed(2.5)
                .state(AutoStates.DROP_PURPLE)
                    .onEnter { intake.setHeight(5) }
                    .transitionTimed(0.5)
                    .onExit { gg.currentIndex++ }
                .state(AutoStates.TO_YELLOW)
                    .transitionTimed(2.0)
                .state(AutoStates.BACKDROP)
                    .onEnter { height = 1; state = RobotState.BACKDROP }
                    .transitionTimed(1.5)
                .state(AutoStates.EXTEND)
                    .onEnter { state = RobotState.EXTEND }
                    .transitionTimed(1.0)
                .state(AutoStates.SCORE)
                    .onEnter { state = RobotState.SCORE }
                    .transitionTimed(1.0)
                .state(AutoStates.RETRACT)
                    .onEnter { state = RobotState.BACKDROP }
                    .transitionTimed(1.0)
                .state(AutoStates.LOCK)
                    .onEnter { height = 0; state = RobotState.LOCK }
                    .transitionTimed(1.0)
                .state(AutoStates.PARK)
                    .onEnter { gg.currentIndex++ }
                .build()

        autoMachine.start()
        while (opModeIsActive()) {
            CONTROL_HUB.clearBulkCache()
            val loop = timeSource.markNow()
            val dt = (loop - loopTime).toDouble(DurationUnit.MILLISECONDS)
            loopTime = loop

            // update all subsystems
            Robot.read(drive, intake, lift, output);
            autoMachine.update()
            cycle.update(state, height, drive.wallDist)
            val input = gg.update(drive.getPoseEstimate().position, drive.getVelocity().position)
            drive.update(
                    input = input,
                    // input = gg.update(drive.getPoseEstimate().position, drive.getVelocity().position),
                    correcting = false,
                    fieldOriented = true,
                    dt = Robot.dt,
                    pid = true,
            )
            Robot.write(drive, intake, lift, output);

            // telemetry.addData("prop pos", traj.prop)
            // telemetry.addData("lift dist", lift.distance)
            telemetry.addData("hz", 1000 / dt)
            telemetry.addData("current index", gg.currentIndex)
            // telemetry.addData("error", gg.error(drive.getPoseEstimate().position))
            // telemetry.addData("onTarget", gg.onTarget(drive.getPoseEstimate().position))
            telemetry.addData("state", autoMachine.state as AutoStates)
            telemetry.update()
        }
    }
}
