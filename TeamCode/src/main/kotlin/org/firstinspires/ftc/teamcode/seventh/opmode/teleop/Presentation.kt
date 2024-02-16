// package org.firstinspires.ftc.teamcode.seventh.opmode.teleop
//
// import com.acmerobotics.dashboard.FtcDashboard
// import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
// import com.arcrobotics.ftclib.gamepad.GamepadEx
// import com.arcrobotics.ftclib.gamepad.GamepadKeys
// import com.qualcomm.hardware.lynx.LynxModule
// import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
// import com.qualcomm.robotcore.eventloop.opmode.TeleOp
// import com.qualcomm.robotcore.hardware.configuration.LynxConstants
// import com.qualcomm.robotcore.util.ElapsedTime
// import com.scrapmetal.quackerama.control.Pose2d
// import com.scrapmetal.quackerama.control.Rotation2d
// import com.scrapmetal.quackerama.control.Vector2d
// import com.sfdev.assembly.state.StateMachineBuilder
//
// import org.firstinspires.ftc.teamcode.seventh.robot.command.CycleCommand
// import org.firstinspires.ftc.teamcode.seventh.robot.hardware.Globals
// import org.firstinspires.ftc.teamcode.seventh.robot.hardware.Robot
// import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.HangSubsystem
// import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.IntakeSubsystem
// import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.LiftSubsystem
// import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.OutputSubsystem
// import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.RobotState
//
// @TeleOp(name = "PRESENTATION")
// class Presentation : LinearOpMode() {
//     enum class ShowcaseStates {
//         INTAKING,
//         PRELOCK,
//         LOCK,
//         REVERSE,
//         WIGGLE_INTAKE,
//         // begin hang
//         QUICK_ON_THE_DRAW, // fast push out and score
//         EXTENDOINGTON, // extend lift to max, lower, lower hang, sine, lock
//     }
//     enum class IndividualStates {
//
//     }
//     override fun runOpMode() {
//         Globals.AUTO = false
//         Robot.hardwareMap = hardwareMap
//         val allHubs = hardwareMap.getAll(LynxModule::class.java)
//         var CONTROL_HUB: LynxModule = allHubs[0]
//         for (hub in allHubs) {
//             hub.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL
//             if (hub.isParent && LynxConstants.isEmbeddedSerialNumber(hub.serialNumber)) {
//                 CONTROL_HUB = hub
//             }
//         }
//         val intake = IntakeSubsystem(hardwareMap)
//         val lift = LiftSubsystem(hardwareMap)
//         val output = OutputSubsystem(hardwareMap)
//         val hang = HangSubsystem(hardwareMap)
//
//         val primary = GamepadEx(gamepad1)
//         var fillEdge = false
//         var fillTimer = ElapsedTime()
//
//         var height = 0
//         val cycle = CycleCommand(intake, lift, output)
//         var lastState = RobotState.LOCK
//         val showcaseMachine = StateMachineBuilder()
//                 .state(RobotState.LOCK)
//                 .transition({ secondary.wasJustPressed(GamepadKeys.Button.A) }, RobotState.INTAKE)
//                 .transition({ primary.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER) }, RobotState.ALIGN)
//                 .transition({ secondary.wasJustPressed(GamepadKeys.Button.BACK) }, RobotState.SPIT)
//                 .onExit { lastState = RobotState.LOCK }
//                 .state(RobotState.PRELOCK)
//                 .transition({ secondary.wasJustPressed(GamepadKeys.Button.B) }, RobotState.LOCK)
//                 .transition({ fillEdge && fillTimer.seconds() > 0.50 }, RobotState.LOCK)
//                 .transition({ secondary.wasJustPressed(GamepadKeys.Button.A) }, RobotState.INTAKE)
//                 .transition({ secondary.wasJustPressed(GamepadKeys.Button.BACK) }, RobotState.SPIT)
//                 .onExit { lastState = RobotState.PRELOCK }
//                 .state(RobotState.INTAKE)
//                 .transition({ secondary.wasJustPressed(GamepadKeys.Button.B) }, RobotState.PRELOCK)
//                 .transition({ fillEdge && fillTimer.seconds() > 0.25 }, RobotState.PRELOCK, { fillTimer = ElapsedTime() })
//                 .transition({ secondary.wasJustPressed(GamepadKeys.Button.BACK) && !gamepad2.guide}, RobotState.SPIT)
//                 .onExit { lastState = RobotState.INTAKE }
//                 .state(RobotState.SPIT)
//                 .transition({ secondary.wasJustPressed(GamepadKeys.Button.B) }, RobotState.LOCK)
//                 .transition({ secondary.wasJustPressed(GamepadKeys.Button.A) }, RobotState.INTAKE)
//                 .onExit { lastState = RobotState.SPIT }
//                 .state(RobotState.ALIGN)
//                 .transition({ secondary.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER) }, RobotState.LOCK)
//                 .transition({ primary.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER) }, RobotState.BACKDROP)
//                 .onExit { lastState = RobotState.ALIGN }
//                 .state(RobotState.BACKDROP)
//                 .transition({ secondary.wasJustPressed(GamepadKeys.Button.DPAD_DOWN) }, RobotState.LOCK)
//                 .transition({ lift.onTarget() && lastState == RobotState.ALIGN }, RobotState.EXTEND)
//                 .transition({ primary.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER) }, RobotState.EXTEND)
//                 .onExit { lastState = RobotState.BACKDROP }
//                 .state(RobotState.EXTEND)
//                 .transition({ primary.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER) }, RobotState.SCORE)
//                 .transition({ primary.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.9 }, RobotState.SCORE_L)
//                 .transition({ primary.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.9 }, RobotState.SCORE_R)
//                 // .transition({ cycle.onTarget() && intendedOutputState == RobotState.SCORE }, RobotState.SCORE)
//                 // .transition({ cycle.onTarget() && intendedOutputState == RobotState.SCORE_L }, RobotState.SCORE_L)
//                 // .transition({ cycle.onTarget() && intendedOutputState == RobotState.SCORE_R }, RobotState.SCORE_R)
//                 .transition({ secondary.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER) }, RobotState.BACKDROP)
//                 .onExit { lastState = RobotState.EXTEND }
//                 .state(RobotState.SCORE_L)
//                 .transition({ secondary.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER) }, RobotState.BACKDROP)
//                 .transition({ primary.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER) }, RobotState.SCORE)
//                 .transition({ primary.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.9 }, RobotState.SCORE_R)
//                 .onExit { lastState = RobotState.SCORE_L }
//                 .state(RobotState.SCORE)
//                 .transition({ secondary.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER) }, RobotState.BACKDROP)
//                 .transition({ primary.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.9 }, RobotState.SCORE_L)
//                 .transition({ primary.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.9 }, RobotState.SCORE_R)
//                 .onExit { lastState = RobotState.SCORE }
//                 .state(RobotState.SCORE_R)
//                 .transition({ secondary.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER) }, RobotState.BACKDROP)
//                 .transition({ primary.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.9 }, RobotState.SCORE_L)
//                 .transition({ primary.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER) }, RobotState.SCORE)
//                 .onExit { lastState = RobotState.SCORE_R }
//                 .build()
//
//         cycleMachine.start()
//         Robot.read(intake, lift, output, hang)
//         cycle.update(cycleMachine.state as RobotState, height, 0.0)
//         Robot.write(intake, lift, output, hang)
//         drive.setPoseEstimate(Globals.pose)
//         waitForStart()
//
//         while (opModeIsActive()) {
//             // === STANDARD UPDATES ===
//             CONTROL_HUB.clearBulkCache()
//             primary.readButtons()
//             cycleMachine.update()
//             Robot.dtUpdate()
//
//             if (output.leftFilled && output.rightFilled && !fillEdge) {
//                 fillEdge = true
//                 fillTimer = ElapsedTime()
//                 primary.gamepad.rumbleBlips(1)
//             } else if (!output.leftFilled && !output.rightFilled && fillEdge) {
//                 fillEdge = false
//             }
//             Robot.read(hang, intake, output, lift)
//             cycle.update(
//                     cycleMachine.state as RobotState,
//                     height,
//                     drive.wallDist
//             )
//             Robot.write(lift, output, intake, hang)
//             telemetry.addData("cycle state", cycleMachine.state as RobotState)
//             telemetry.addData("lift height", height)
//             telemetry.addData("lift commanded", lift.state.commandedExtension)
//             telemetry.addData("lift dist", lift.state.extension)
//             telemetry.addData("grounded", lift.state.grounded)
//             telemetry.addData("left dets", output.leftFilled)
//             telemetry.addData("right dets", output.rightFilled)
//             telemetry.update()
//         }
//     }
// }
//