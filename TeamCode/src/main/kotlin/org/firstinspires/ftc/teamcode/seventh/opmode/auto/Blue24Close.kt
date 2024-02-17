package org.firstinspires.ftc.teamcode.seventh.opmode.auto

import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.configuration.LynxConstants
import com.qualcomm.robotcore.util.ElapsedTime
import com.scrapmetal.quackerama.control.Pose2d
import com.scrapmetal.quackerama.control.Vector2d
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
import org.firstinspires.ftc.teamcode.seventh.robot.vision.Vision
import java.lang.Math.toDegrees

@Autonomous(name =
"""
BLUE 2+4 CLOSE
"""
)
class Blue22Close : LinearOpMode() {
    enum class AutoStates {
        TO_PURPLE,
        DROP_PURPLE,
        TO_YELLOW,
        RESET_POSE,
        DROP_YELLOW,
        RETRACT,
        LOCK,
        TO_STACKS,
        INTAKE_STACKS,
        TO_BACKDROP,
        RESET_POSE_AGAIN,
        DROP_WHITE,
        PARK,
    }
    override fun runOpMode() {
        Globals.AUTO = true
        Globals.side = Globals.Side.BLUE
        Globals.start = Globals.Start.CLOSE
        Robot.hardwareMap = hardwareMap
        val allHubs = hardwareMap.getAll(LynxModule::class.java)
        var CONTROL_HUB: LynxModule = allHubs[0]
        for (hub in allHubs) {
            hub.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL
            if (hub.isParent && LynxConstants.isEmbeddedSerialNumber(hub.serialNumber)) {
                CONTROL_HUB = hub
            }
        }
        val drive = DriveSubsystem(hardwareMap)
        val intake = IntakeSubsystem(hardwareMap)
        val lift = LiftSubsystem(hardwareMap)
        val output = OutputSubsystem(hardwareMap)
        val vision = Vision(hardwareMap)
        val cycle = CycleCommand(intake, lift, output)

        var state = RobotState.LOCK
        var height = 0
        var stackCount = 5
        val timer = ElapsedTime()

        lateinit var gg: GG
        val autoMachine = StateMachineBuilder()
                .state(AutoStates.TO_PURPLE)
                .transitionTimed(1.5)
                .state(AutoStates.DROP_PURPLE)
                .onEnter { intake.setHeight(5) }
                .transitionTimed(0.2)
                .onExit { gg.currentIndex++ }
                .state(AutoStates.TO_YELLOW)
                .transitionTimed(1.1)
                .state(AutoStates.RESET_POSE)
                .onEnter { drive.getPoseEstimate().let {
                    drive.setPoseEstimate(Pose2d(
                            Vector2d(72.0-9.5-drive.wallDist-9.0, it.position.v),
                            it.heading
                    ))
                } }
                .transitionTimed(0.3)
                .onExit { drive.update(Pose2d(), correcting = false, fieldOriented = true, dt = Robot.dt, pid = false)}
                .state(AutoStates.DROP_YELLOW)
                .onEnter { timer.reset(); height = 1; state = RobotState.BACKDROP }
                .transitionTimed(1.8)
                .state(AutoStates.RETRACT)
                .onEnter { state = RobotState.BACKDROP; gg.maxVel = 0.50; gg.kN = 0.3; gg.currentIndex++ }
                .transitionTimed(0.5)
                .state(AutoStates.LOCK)
                .onEnter { height = 0; state = RobotState.LOCK }
                .transition({ stackCount in listOf(5, 3) }, AutoStates.TO_STACKS) { intake.setHeight(stackCount) }
                .state(AutoStates.TO_STACKS)
                .transitionTimed(4.5)
                .state(AutoStates.INTAKE_STACKS)
                .onEnter { timer.reset() }
                .transitionTimed(1.0)
                .state(AutoStates.TO_BACKDROP)
                .onEnter { gg.currentIndex++; timer.reset() }
                .transitionTimed(4.0)
                .state(AutoStates.RESET_POSE_AGAIN)
                .onEnter { drive.getPoseEstimate().let {
                    drive.setPoseEstimate(Pose2d(
                            Vector2d(72.0-9.5-drive.wallDist-9.0, it.position.v),
                            it.heading
                    ))
                }
                stackCount -= 2 }
                .transitionTimed(0.3)
                .onExit { drive.update(Pose2d(), correcting = false, fieldOriented = false, dt = Robot.dt, pid = false); timer.reset() }
                .state(AutoStates.DROP_WHITE)
                .onEnter { height = 2; state = RobotState.BACKDROP }
                .transitionTimed(1.3, AutoStates.RETRACT)
                .build()
        intake.setHeight(1)
        Robot.read(intake, output)
        cycle.update(state, height, 0.0)
        Robot.write(intake, output)

        vision.enableProp()
        drive.startIMUThread(this)
        while (opModeInInit()) {
            telemetry.addData("prop", vision.getPropPosition())
            telemetry.update()
        }
        val paths = AutoPaths(
                Globals.Side.BLUE,
                Globals.Start.CLOSE,
                Globals.Lane.LANE_3,
                Globals.YellowSide.LEFT,
                Globals.Stack.FAR,
                Globals.Park.NONE,
                vision.getPropPosition(),
        )
        vision.disableProp()
        drive.setPoseEstimate(paths.initPose)
        gg = GG(
                kN = 0.5,
                kD = 0.004,
                maxVel = 0.8,
                paths.purple,
                paths.yellow,
                paths.intake,
                paths.score,
                paths.intake,
                paths.score,
                paths.parkPath,
        )
        autoMachine.start()

        while (opModeIsActive()) {
            CONTROL_HUB.clearBulkCache()
            Robot.dtUpdate()
            autoMachine.update()

            // update all subsystems
            Robot.read(intake, output, lift, drive)
            val input = gg.update(drive.getPoseEstimate().position, drive.getVelocity().position)
            drive.update(
                    input = input,
                    correcting = false,
                    fieldOriented = true,
                    dt = Robot.dt,
                    pid = true,
            )

            if (autoMachine.state as AutoStates == AutoStates.DROP_YELLOW) {
                when {
                    timer.seconds() > 0.7 && state == RobotState.BACKDROP
                    -> { state = RobotState.EXTEND }
                    timer.seconds() > 0.9 && timer.seconds() < 1.2 && state == RobotState.EXTEND
                    -> { height = 6 }
                    timer.seconds() > 1.2 && state == RobotState.EXTEND
                    -> { state = RobotState.SCORE }
                    timer.seconds() > 1.6 && state == RobotState.SCORE
                    -> { height = 1 }
                }
            }
            if (autoMachine.state as AutoStates == AutoStates.DROP_WHITE) {
                when {
                    timer.seconds() > 0.7 && state == RobotState.BACKDROP
                    -> { state = RobotState.EXTEND }
                    timer.seconds() > 1.0 && state == RobotState.EXTEND
                    -> { state = RobotState.SCORE }
                }
            }
            when {
                autoMachine.state as AutoStates == AutoStates.TO_STACKS &&
                        drive.getPoseEstimate().position.u < -48.0
                -> { state = RobotState.INTAKE }
                autoMachine.state as AutoStates == AutoStates.INTAKE_STACKS && timer.seconds() > 0.5
                -> { intake.setHeight(stackCount-1) }
                autoMachine.state as AutoStates == AutoStates.TO_BACKDROP && timer.seconds() > 2.0 && state == RobotState.INTAKE
                -> { state = RobotState.PRELOCK }
                autoMachine.state as AutoStates == AutoStates.TO_BACKDROP && timer.seconds() > 2.5 && state == RobotState.PRELOCK
                -> { state = RobotState.LOCK }
            }
            cycle.update(state, height, drive.wallDist)
            if (
                    (autoMachine.state as AutoStates == AutoStates.DROP_YELLOW || autoMachine.state as AutoStates == AutoStates.DROP_WHITE) &&
                    timer.seconds() > 0.2
            ) {
                Robot.write(lift, output)
            } else {
                Robot.write(drive, lift, output, intake)
            }


            telemetry.addData("hz", 1000 / Robot.dt)
            telemetry.addData("error", gg.error(drive.getPoseEstimate().position))
            telemetry.addData("onTarget", gg.onTarget(drive.getPoseEstimate().position))
            telemetry.addData("path index", gg.currentIndex)
            telemetry.addData("stack count", stackCount)
            telemetry.addData("state", autoMachine.state as AutoStates)
            telemetry.addData("x", drive.getPoseEstimate().position.u)
            telemetry.addData("y", drive.getPoseEstimate().position.v)
            telemetry.addData("heading error", toDegrees(input.heading.polarAngle - drive.getPoseEstimate().heading.polarAngle))
            telemetry.update()
        }
        Globals.pose = drive.getPoseEstimate()
    }
}
