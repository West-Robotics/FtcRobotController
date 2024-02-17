package org.firstinspires.ftc.teamcode.seventh.opmode.teleop

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.configuration.LynxConstants
import com.qualcomm.robotcore.util.ElapsedTime
import com.scrapmetal.quackerama.control.Pose2d
import com.scrapmetal.quackerama.control.Rotation2d
import com.scrapmetal.quackerama.control.Vector2d
import com.sfdev.assembly.state.StateMachineBuilder

import org.firstinspires.ftc.teamcode.seventh.robot.command.CycleCommand
import org.firstinspires.ftc.teamcode.seventh.robot.hardware.Globals
import org.firstinspires.ftc.teamcode.seventh.robot.hardware.Robot
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.HangSubsystem
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.IntakeSubsystem
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.LiftSubsystem
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.OutputSubsystem
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.RobotState

@TeleOp(name = "PRESENTATION")
class Presentation : LinearOpMode() {
    enum class ShowcaseStates {
        INTAKING,
        PRELOCK,
        LOCK,
        WIGGLE_UP,
        WIGGLE_DOWN,
        WIGGLE_UP_AGAIN,
        RAISE,
        EXTEND,
        LOWER,
        SCORE,
        MAX,
        RETRACT,
        FINAL_DOWN,
    } // when hnag?
    enum class IndividualStates {

    }
    override fun runOpMode() {
        Globals.AUTO = false
        Robot.hardwareMap = hardwareMap
        val allHubs = hardwareMap.getAll(LynxModule::class.java)
        var CONTROL_HUB: LynxModule = allHubs[0]
        for (hub in allHubs) {
            hub.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL
            if (hub.isParent && LynxConstants.isEmbeddedSerialNumber(hub.serialNumber)) {
                CONTROL_HUB = hub
            }
        }
        val intake = IntakeSubsystem(hardwareMap)
        val lift = LiftSubsystem(hardwareMap)
        val output = OutputSubsystem(hardwareMap)
        val hang = HangSubsystem(hardwareMap)

        var state = RobotState.LOCK

        var height = 0
        val cycle = CycleCommand(intake, lift, output)
        val showcaseMachine = StateMachineBuilder()
                .state(ShowcaseStates.INTAKING)
                    .onEnter { state = RobotState.INTAKE }
                    .transition { output.leftFilled && output.rightFilled }
                    .transitionTimed(1.0)
                .state(ShowcaseStates.PRELOCK)
                    .onEnter { state = RobotState.PRELOCK }
                    .transitionTimed(0.2)
                .state(ShowcaseStates.LOCK)
                    .onEnter { state = RobotState.LOCK }
                    .transitionTimed(0.2)
                .state(ShowcaseStates.WIGGLE_UP)
                    .onEnter { intake.setHeight(5) }
                    .transitionTimed(0.2)
                .state(ShowcaseStates.WIGGLE_DOWN)
                    .onEnter { intake.setHeight(1) }
                    .transitionTimed(0.2)
                .state(ShowcaseStates.WIGGLE_UP_AGAIN)
                    .onEnter { intake.setHeight(5) }
                    .transitionTimed(0.2)
                .state(ShowcaseStates.RAISE)
                    .onEnter { state = RobotState.BACKDROP; height = 1 }
                    .transitionTimed(0.7)
                .state(ShowcaseStates.EXTEND)
                    .onEnter { state = RobotState.EXTEND }
                    .transitionTimed(0.3)
                .state(ShowcaseStates.LOWER)
                    .onEnter { height = 6 }
                    .transitionTimed(0.2)
                .state(ShowcaseStates.SCORE)
                    .onEnter { state = RobotState.SCORE }
                    .transitionTimed(0.3)
                .state(ShowcaseStates.MAX)
                    .onEnter { height = 4 }
                    .transitionTimed(0.8)
                .state(ShowcaseStates.RETRACT)
                    .onEnter { state = RobotState.BACKDROP }
                    .transitionTimed(0.2)
                .state(ShowcaseStates.FINAL_DOWN)
                    .onEnter { state = RobotState.LOCK; height = 0 }
                .build()

        showcaseMachine.start()
        Robot.read(intake, lift, output, hang)
        cycle.update(RobotState.LOCK, height, 0.0)
        Robot.write(intake, lift, output, hang)
        waitForStart()
        val timer = ElapsedTime()

        while (opModeIsActive()) {
            // === STANDARD UPDATES ===
            CONTROL_HUB.clearBulkCache()
            Robot.dtUpdate()

            Robot.read(hang, intake, output, lift)
            showcaseMachine.update()
            cycle.update(
                    state,
                    height,
                    6.0
            )
            if (state == RobotState.EXTEND) {
                hang.update(HangSubsystem.HangState.RAISE)
                timer.reset()
            }
            if (showcaseMachine.state as ShowcaseStates in
                    listOf(ShowcaseStates.EXTEND, ShowcaseStates.LOWER, ShowcaseStates.SCORE, ShowcaseStates.MAX, ShowcaseStates.RETRACT, ShowcaseStates.FINAL_DOWN)) {
                if (timer.seconds() < 1.5) {
                    hang.update(HangSubsystem.HangState.RAISE)
                } else if (timer.seconds() < 3.0) {
                    hang.update(HangSubsystem.HangState.LOWER)
                } else {
                    hang.update(HangSubsystem.HangState.STOP)
                }
            }
            Robot.write(lift, output, intake, hang)
            telemetry.update()
        }
    }
}