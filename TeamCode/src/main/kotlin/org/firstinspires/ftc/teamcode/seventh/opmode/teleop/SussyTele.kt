package org.firstinspires.ftc.teamcode.seventh.opmode.teleop

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.PerpetualCommand
import com.arcrobotics.ftclib.command.RunCommand
import com.arcrobotics.ftclib.command.button.Trigger
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.outoftheboxrobotics.photoncore.Photon
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.scrapmetal.quackerama.control.Pose2d
import com.sfdev.assembly.state.StateMachineBuilder

import org.firstinspires.ftc.teamcode.seventh.robot.command.FallCommand
import org.firstinspires.ftc.teamcode.seventh.robot.command.IntakeCommand
import org.firstinspires.ftc.teamcode.seventh.robot.command.RiseCommand
import org.firstinspires.ftc.teamcode.seventh.robot.hardware.Globals
import org.firstinspires.ftc.teamcode.seventh.robot.hardware.Robot
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.DriveSubsystem
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.DroneSubsystem
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.HangSubsystem
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.IntakeSubsystem
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.LiftSubsystem
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.OutputSubsystem
import org.firstinspires.ftc.teamcode.seventh.robot.command.RobotState
import org.firstinspires.ftc.teamcode.seventh.robot.command.SpitCommand
import java.lang.Math.toDegrees
import kotlin.math.pow
import kotlin.math.sign
import kotlin.time.DurationUnit
import kotlin.time.TimeSource

@Photon
@TeleOp(name =
    "🟦🟦🟦🟦🟦🟦🟦🟦🟦⬛⬛⬛⬛⬛🟦🟦🟦🟦🟦🟦🟦\n" +
    "🟦🟦🟦🟦🟦🟦🟦🟦⬛⬛🟥🟥🟥⬛⬛⬛🟦🟦🟦🟦🟦\n" +
    "🟦🟦🟦🟦🟦🟦🟦⬛⬛🟥🟥🟥🟥🟥🟥🟥⬛🟦🟦🟦🟦\n" +
    "🟦🟦🟦🟦🟦🟦🟦⬛🟥🟥🟥🟥🟥🟥🟥🟥⬛🟦🟦🟦🟦\n" +
    "🟦🟦🟦🟦🟦🟦🟦⬛🟥🟥🟥🟥🟥🟥🟥🟥🟥⬛🟦🟦🟦\n" +
    "🟦🟦🟦🟦🟦🟦🟦⬛🟥🟥🟥⬜⬜⬜⬜⬜⬜⬛⬛🟦🟦\n" +
    "🟦🟦🟦🟦⬛⬛⬛🟥🟥🟥⬜⬜⬜⬜⬜⬜⬜⬜⬛🟦🟦\n" +
    "🟦🟦🟦⬛🟥⬛⬛🟥🟥🟥⬜⬜⬜⬜⬜⬜⬜⬜⬛🟦🟦\n" +
    "🟦🟦🟦⬛🟥⬛⬛🟥🟥🟥⬛⬜⬜⬜⬜⬜⬜⬜⬛🟦🟦\n" +
    "🟦🟦⬛⬛🟥⬛⬛🟥🟥🟥🟥⬛⬛⬛⬛⬛⬛⬛🟦🟦🟦\n" +
    "🟦🟦⬛⬛🟥⬛⬛🟥🟥🟥🟥🟥🟥🟥🟥🟥🟥⬛🟦🟦🟦\n" +
    "🟦🟦⬛⬛🟥⬛⬛🟥🟥🟥🟥🟥🟥🟥🟥🟥🟥⬛🟦🟦🟦\n" +
    "🟦🟦⬛⬛🟥⬛⬛🟥🟥🟥🟥🟥🟥🟥🟥🟥🟥⬛🟦🟦🟦\n" +
    "🟦🟦⬛⬛🟥⬛⬛🟥🟥🟥🟥🟥🟥🟥🟥🟥🟥⬛🟦🟦🟦\n" +
    "🟦🟦⬛⬛🟥⬛⬛🟥🟥🟥🟥🟥🟥🟥🟥🟥🟥⬛🟦🟦🟦\n" +
    "🟦🟦⬛⬛🟥⬛⬛🟥🟥🟥🟥🟥🟥🟥🟥🟥🟥⬛🟦🟦🟦\n" +
    "🟦🟦⬛⬛🟥⬛⬛🟥🟥🟥🟥🟥🟥🟥🟥🟥🟥⬛🟦🟦🟦\n" +
    "🟦🟦🟦⬛⬛⬛⬛🟥🟥🟥🟥⬛⬛🟥🟥🟥🟥⬛🟦🟦🟦\n" +
    "🟦🟦🟦🟦🟦⬛⬛🟥🟥🟥🟥⬛⬛🟥🟥🟥🟥⬛🟦🟦🟦\n" +
    "🟦🟦🟦🟦🟦⬛⬛⬛🟥🟥⬛⬛⬛🟥🟥🟥⬛🟦🟦🟦🟦\n" +
    "🟦🟦🟦🟦🟦🟦🟦⬛⬛⬛⬛🟦⬛⬛⬛⬛⬛🟦🟦🟦🟦")
// TODO: add heading pidf
class SussyTele : LinearOpMode() {
    // run blocking?
    override fun runOpMode() {
        Globals.AUTO = false
        // this has the potential to slow down looptimes
        // telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
        val driver = Gigapad(gamepad1)
        val operator = Gigapad(gamepad2)
        Robot.init(hardwareMap, telemetry, driver, operator)
        val drive = DriveSubsystem(hardwareMap)
        val intake = IntakeSubsystem(hardwareMap)
        val lift = LiftSubsystem(hardwareMap)
        val output = OutputSubsystem(hardwareMap)
        var queuedHeight = 0
        var queuedRoll = OutputSubsystem.Roll.HORIZ

        // WHAT HAPPENS WHEN YOU TRY TO CHANGE ROLL WHILE IT IS TRANSITIONING?
        // DO I NEED TO MANAGE THOSE CONFLICTS
        // without subsystem requirement, multiple commands could attempt at once
        // with subsystem requirement, you could end a command by just trying to change the roll
        // do you need to declare requirements on an instant command?
        // do both need to declare to find a conflict or just one?
        // maybe this actually works if we only declare requirements on the big commands and leave
        // the little settings requirement free
        val fsm = StateMachineBuilder()
                .state(RobotState.LOCK)
                    .transition({ operator.wasJustReleased(GamepadKeys.Button.RIGHT_BUMPER) }, RobotState.BACKDROP)
                // .state(RobotState.TRANSITION)
                // .transition({})
                .state(RobotState.BACKDROP)
                    .transition({ operator.wasJustReleased(GamepadKeys.Button.RIGHT_BUMPER) }, RobotState.LOCK)
                .build()
        val locking = Trigger { fsm.state == RobotState.LOCK }
        val backdropping = Trigger { fsm.state == RobotState.BACKDROP }

        driver.guide.whenActive(InstantCommand({ drive.resetHeading() }))
        PerpetualCommand(RunCommand({ drive.updateOpen(
            Pose2d(
                driver.leftY,
                -driver.leftX,
                -driver.rightX.let { it.sign*it.pow(2) },
            ),
            fieldOriented = false,
            headingPID = false,
        )})).schedule()
        locking.and(operator.getGamepadButton(GamepadKeys.Button.A))
            .whileActiveOnce(IntakeCommand(intake, output, lift))
        locking.and(operator.getGamepadButton(GamepadKeys.Button.B))
            .whileActiveOnce(SpitCommand(intake, output))
        locking.and(operator.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER))
            .whenActive(RiseCommand({ queuedHeight }, { queuedRoll }, lift, output))
        locking.and(operator.north).whenActive(InstantCommand({ queuedRoll = OutputSubsystem.Roll.VERT_I }))
        locking.and(operator.nw).whenActive(InstantCommand({ queuedRoll = OutputSubsystem.Roll.RM }))
        locking.and(operator.west).whenActive(InstantCommand({ queuedRoll = OutputSubsystem.Roll.HORIZ }))
        locking.and(operator.sw).whenActive(InstantCommand({ queuedRoll = OutputSubsystem.Roll.LM }))
        locking.and(operator.south).whenActive(InstantCommand({ queuedRoll = OutputSubsystem.Roll.VERT }))
        locking.and(operator.dec).whenActive(InstantCommand({ queuedHeight-- }))
        locking.and(operator.inc).whenActive(InstantCommand({ queuedHeight++ }))
        backdropping.and(operator.north).whenActive(InstantCommand({ output.set(OutputSubsystem.Roll.VERT_I); lift.set(OutputSubsystem.Roll.VERT_I) }))
        backdropping.and(operator.nw).whenActive(InstantCommand({ output.set(OutputSubsystem.Roll.RM); lift.set(OutputSubsystem.Roll.RM) }))
        backdropping.and(operator.west).whenActive(InstantCommand({ output.set(OutputSubsystem.Roll.HORIZ); lift.set(OutputSubsystem.Roll.HORIZ) }))
        backdropping.and(operator.sw).whenActive(InstantCommand({ output.set(OutputSubsystem.Roll.LM); lift.set(OutputSubsystem.Roll.LM) }))
        backdropping.and(operator.south).whenActive(InstantCommand({ output.set(OutputSubsystem.Roll.VERT); lift.set(OutputSubsystem.Roll.VERT) }))
        backdropping.and(operator.dec).whenActive(InstantCommand({ lift.lower() }))
        backdropping.and(operator.inc).whenActive(InstantCommand({ lift.raise() }))
        backdropping.and(operator.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER))
            .whenActive(FallCommand(lift, output))
        backdropping.and(driver.lTrig).whenActive(InstantCommand({ output.set(OutputSubsystem.Claw.BOTH) }))
        backdropping.and(driver.rTrig).whenActive(InstantCommand({ output.set(OutputSubsystem.Claw.NONE) }))
        backdropping.and(driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER))
            .whenActive(InstantCommand({ output.set(OutputSubsystem.Claw.RIGHT) }))
        backdropping.and(driver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER))
            .whenActive(InstantCommand({ output.set(OutputSubsystem.Claw.LEFT) }))
        backdropping.whileActiveContinuous(RunCommand({ output.set(-toDegrees(drive.getPoseEstimate().heading.theta).let {
            if (it <= 180) it else -(360-it)
        }) }))
        operator.getGamepadButton(GamepadKeys.Button.X).whenPressed(InstantCommand({ intake.lower() }))
        operator.getGamepadButton(GamepadKeys.Button.Y).whenPressed(InstantCommand({ intake.raise() }))
        waitForStart()

        fsm.start()
        drive.startIMUThread(this)
        val timeSource = TimeSource.Monotonic
        while (opModeIsActive()) {
            run {
                val t1 = timeSource.markNow()
                Robot.read(intake, lift, output, drive)
                fsm.update()
                val t2 = timeSource.markNow()
                telemetry.addData("section 1 lt", (t2-t1).toDouble(DurationUnit.MILLISECONDS))
            }
            // if (fsm.state == RobotState.BACKDROP)
            // when {
            //     secondary.wasJustPressed(GamepadKeys.Button.BACK) && gamepad2.guide -> output.pivotOffset -= 11.25
            //     secondary.wasJustPressed(GamepadKeys.Button.START) && gamepad2.guide -> output.pivotOffset += 11.25
            //     secondary.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER) && gamepad2.guide -> drone.set(DroneSubsystem.State.DIPER)
            // }

            run {
                val t1 = timeSource.markNow()
                telemetry.addData("state", fsm.state as RobotState)
                telemetry.addData("hz ", 1 / Robot.getDt())
                telemetry.addData("board level", lift.boardLevel)
                telemetry.addData("lift extension", lift.extension)
                telemetry.addData("grounded", lift.grounded)
                telemetry.addData("queued height", queuedHeight)
                telemetry.addData("queued roll", queuedRoll)
                telemetry.addData("heading", toDegrees(drive.getPoseEstimate().heading.theta))
                Robot.write(drive, output, lift, intake)
                val t2 = timeSource.markNow()
                telemetry.addData("section 2 lt", (t2-t1).toDouble(DurationUnit.MILLISECONDS))
            }
        }
    }
}