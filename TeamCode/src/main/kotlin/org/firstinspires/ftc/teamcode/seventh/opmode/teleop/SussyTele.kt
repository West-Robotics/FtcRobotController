// package org.firstinspires.ftc.teamcode.seventh.opmode.teleop
//
// import com.acmerobotics.roadrunner.geometry.Pose2d
// import com.arcrobotics.ftclib.gamepad.GamepadEx
// import com.arcrobotics.ftclib.gamepad.GamepadKeys
// import com.qualcomm.hardware.lynx.LynxModule
// import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
// import com.qualcomm.robotcore.eventloop.opmode.TeleOp
// import com.qualcomm.robotcore.util.ElapsedTime
//
// import org.firstinspires.ftc.teamcode.seventh.drive.SampleMecanumDrive
// import org.firstinspires.ftc.teamcode.seventh.robot.command.CycleCommand
// import org.firstinspires.ftc.teamcode.seventh.robot.command.CycleCommand.CycleState
// import org.firstinspires.ftc.teamcode.seventh.robot.command.TeleMachines
// import org.firstinspires.ftc.teamcode.seventh.robot.hardware.Globals
// import org.firstinspires.ftc.teamcode.seventh.robot.hardware.Hardware
// import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.HangSubsystem
// import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.IntakeSubsystem
// import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.LiftSubsystem
// import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.OutputSubsystem
// import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.OutputSubsystem.OutputState
// import kotlin.math.abs
// import kotlin.math.pow
// import kotlin.math.sign
// import kotlin.time.DurationUnit
// import kotlin.time.TimeSource
//
// @TeleOp(name =
//     "🟦🟦🟦🟦🟦🟦🟦🟦🟦⬛⬛⬛⬛⬛🟦🟦🟦🟦🟦🟦🟦\n" +
//     "🟦🟦🟦🟦🟦🟦🟦🟦⬛⬛🟥🟥🟥⬛⬛⬛🟦🟦🟦🟦🟦\n" +
//     "🟦🟦🟦🟦🟦🟦🟦⬛⬛🟥🟥🟥🟥🟥🟥🟥⬛🟦🟦🟦🟦\n" +
//     "🟦🟦🟦🟦🟦🟦🟦⬛🟥🟥🟥🟥🟥🟥🟥🟥⬛🟦🟦🟦🟦\n" +
//     "🟦🟦🟦🟦🟦🟦🟦⬛🟥🟥🟥🟥🟥🟥🟥🟥🟥⬛🟦🟦🟦\n" +
//     "🟦🟦🟦🟦🟦🟦🟦⬛🟥🟥🟥⬜⬜⬜⬜⬜⬜⬛⬛🟦🟦\n" +
//     "🟦🟦🟦🟦⬛⬛⬛🟥🟥🟥⬜⬜⬜⬜⬜⬜⬜⬜⬛🟦🟦\n" +
//     "🟦🟦🟦⬛🟥⬛⬛🟥🟥🟥⬜⬜⬜⬜⬜⬜⬜⬜⬛🟦🟦\n" +
//     "🟦🟦🟦⬛🟥⬛⬛🟥🟥🟥⬛⬜⬜⬜⬜⬜⬜⬜⬛🟦🟦\n" +
//     "🟦🟦⬛⬛🟥⬛⬛🟥🟥🟥🟥⬛⬛⬛⬛⬛⬛⬛🟦🟦🟦\n" +
//     "🟦🟦⬛⬛🟥⬛⬛🟥🟥🟥🟥🟥🟥🟥🟥🟥🟥⬛🟦🟦🟦\n" +
//     "🟦🟦⬛⬛🟥⬛⬛🟥🟥🟥🟥🟥🟥🟥🟥🟥🟥⬛🟦🟦🟦\n" +
//     "🟦🟦⬛⬛🟥⬛⬛🟥🟥🟥🟥🟥🟥🟥🟥🟥🟥⬛🟦🟦🟦\n" +
//     "🟦🟦⬛⬛🟥⬛⬛🟥🟥🟥🟥🟥🟥🟥🟥🟥🟥⬛🟦🟦🟦\n" +
//     "🟦🟦⬛⬛🟥⬛⬛🟥🟥🟥🟥🟥🟥🟥🟥🟥🟥⬛🟦🟦🟦\n" +
//     "🟦🟦⬛⬛🟥⬛⬛🟥🟥🟥🟥🟥🟥🟥🟥🟥🟥⬛🟦🟦🟦\n" +
//     "🟦🟦⬛⬛🟥⬛⬛🟥🟥🟥🟥🟥🟥🟥🟥🟥🟥⬛🟦🟦🟦\n" +
//     "🟦🟦🟦⬛⬛⬛⬛🟥🟥🟥🟥⬛⬛🟥🟥🟥🟥⬛🟦🟦🟦\n" +
//     "🟦🟦🟦🟦🟦⬛⬛🟥🟥🟥🟥⬛⬛🟥🟥🟥🟥⬛🟦🟦🟦\n" +
//     "🟦🟦🟦🟦🟦⬛⬛⬛🟥🟥⬛⬛⬛🟥🟥🟥⬛🟦🟦🟦🟦\n" +
//     "🟦🟦🟦🟦🟦🟦🟦⬛⬛⬛⬛🟦⬛⬛⬛⬛⬛🟦🟦🟦🟦")
// // TODO: add rumble on intake
// class SussyTele : LinearOpMode() {
//     // run blocking?
//     override fun runOpMode() {
//         // IDEAS:
//         // automatically flip driving direction
//         // lock robot to face exactly backdrop
//         // mecanum feedforward
//         // delete clutter opmodes
//         val allHubs = hardwareMap.getAll(LynxModule::class.java)
//         for (hub in allHubs) {
//             hub.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL
//         }
//         val hardware = Hardware.getInstance(hardwareMap)
//         val drive =
//             SampleMecanumDrive(
//                 hardware,
//                 hardwareMap
//             )
//         val intake = IntakeSubsystem(hardware)
//         val lift = LiftSubsystem(hardware)
//         val out = OutputSubsystem(hardware)
//         val hang = HangSubsystem(hardware)
//
//         val primary = GamepadEx(gamepad1)
//         val secondary = GamepadEx(gamepad2)
//         var x = 0.0
//         var y = 0.0
//         var turn = 0.0
//         // change per millisecond
//         val SLEW_RATE = 2.0*1e-3
//         var dir = -1
//         val timeSource = TimeSource.Monotonic
//         var loopTime: TimeSource.Monotonic.ValueTimeMark = timeSource.markNow()
//
//         // is this bad? maybe switch to a singleton
//         val cycle = CycleCommand(intake, lift, out)
//         val cycleMachine = TeleMachines.getCycleMachine(primary, secondary)
//         val outMachine = TeleMachines.getOutMachine(primary, secondary)
//         val hangMachine = TeleMachines.getHangMachine(primary, secondary)
//         var height = -1
//
//         cycleMachine.start()
//         outMachine.start()
//         hangMachine.start()
//         hardware.read(intake, lift, out, hang)
//         cycle.update(cycleMachine.state as CycleState, outMachine.state as OutputState, -1)
//         hardware.write(intake, lift, out, hang)
//         telemetry.addLine("waiting for start")
//         telemetry.update()
//         waitForStart()
//
//         var multiplier = 1.0
//         var lastLiftState = LiftSubsystem.LiftState.DOWN
//         var lastOutState = OutputState.LOCK
//         val elapsedTime = ElapsedTime()
//         while (opModeIsActive() && !isStopRequested) {
//             for (hub in allHubs) {
//                 hub.clearBulkCache()
//             }
//             val loop = timeSource.markNow()
//             val dt = (loop - loopTime).toDouble(DurationUnit.MILLISECONDS)
//             loopTime = loop
//             // remember to convert to seconds (multiply by 10^-3) for slew rate lmao
//
//             primary.readButtons()
//             secondary.readButtons()
//
//             // update state machines
//             cycleMachine.update()
//             outMachine.update()
//             hangMachine.update()
//             if (secondary.wasJustPressed(GamepadKeys.Button.DPAD_UP) && height < 4) {
//                 height++
//             } else if (secondary.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
//                 // if (height > 0) {
//                 //     height = 0
//                 // } else if (height == 0) {
//                 //     height = -1
//                 // }
//                 height = -1
//             }
//             if (lift.state == LiftSubsystem.LiftState.DOWN && lastLiftState == LiftSubsystem.LiftState.UP) {
//                 dir *= -1
//             }
//             if (outMachine.state == OutputState.LOCK && lastOutState == OutputState.INTAKE) {
//                 dir *= -1
//             }
//             if (primary.wasJustPressed(GamepadKeys.Button.A)) {
//                 dir = 1
//             }
//             // if (primary.wasJustPressed(GamepadKeys.Button.X)) {
//             //     dir *= -1
//             // }
//             lastLiftState = lift.state
//             lastOutState = outMachine.state as OutputState
//
//             if (secondary.wasJustPressed(GamepadKeys.Button.START)) {
//                 Globals.PIVOT_OUTTAKE -= 0.01
//             } else if (secondary.wasJustPressed(GamepadKeys.Button.BACK)) {
//                 Globals.PIVOT_OUTTAKE += 0.01
//             }
//
//             // update all subsystems
//             hardware.read(intake, lift, out, hang)
//             cycle.update(cycleMachine.state as CycleState, outMachine.state as OutputState, height)
//             hang.update(hangMachine.state as HangSubsystem.HangState)
//             hardware.write(intake, lift, out, hang)
//
//             // only change dt powers by at max the slew rate
//             x += (primary.leftY - x).let { if (abs(it) < SLEW_RATE*dt) it else sign(it)*SLEW_RATE*dt }
//             y += (-primary.leftX - y).let { if (abs(it) < SLEW_RATE*dt) it else sign(it)*SLEW_RATE*dt }
//             // turn += (-primary.rightX - turn).let { if (abs(it) < SLEW_RATE*(4.0/3.0)*dt) it else sign(it)*SLEW_RATE*(4.0/3.0)*dt }
//             turn += (-primary.rightX - turn).let {
//                 if (sign(turn) == 1.0) {
//                     if (sign(it) == 1.0) {
//                         if (abs(it) < SLEW_RATE * (4.0 / 3.0) * dt) it else sign(it) * SLEW_RATE * (4.0 / 3.0) * dt
//                     } else {
//                         it
//                     }
//                 } else if (sign(turn) == 0.0) {
//                     if (abs(it) < SLEW_RATE * (4.0 / 3.0) * dt) it else sign(it) * SLEW_RATE * (4.0 / 3.0) * dt
//                 } else {
//                     if (sign(it) == -1.0) {
//                         if (abs(it) < SLEW_RATE * (4.0 / 3.0) * dt) it else sign(it) * SLEW_RATE * (4.0 / 3.0) * dt
//                     } else {
//                         it
//                     }
//                 }
//             }
//
//             multiplier = when {
//                 primary.isDown(GamepadKeys.Button.B)                -> 0.15
//                 primary.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER) && multiplier == 1.0 -> 0.3
//                 primary.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER) && multiplier == 0.3 || secondary.wasJustPressed(GamepadKeys.Button.DPAD_DOWN) -> 1.0
//                 // cycleMachine.state == CycleState.READY              -> 0.5
//                 // intake.state == IntakeSubsystem.IntakeState.INTAKE  -> 0.5
//                 else                                                -> multiplier
//             }
//             telemetry.addData("multi", multiplier)
//             val turn_multi = when {
//                 multiplier == 0.3 -> 0.6
//                 else -> 1.0
//             }
//             drive.setWeightedDrivePower(Pose2d(dir*x.pow(3)*multiplier*13.0/hardware.voltage,
//                                                y.pow(3)*multiplier*13.0/hardware.voltage,
//                                         (turn/1.6)*turn_multi.pow(2)*13.0/hardware.voltage))
//
//             if (90.0 < elapsedTime.seconds() && elapsedTime.seconds() < 91.0) {
//                 primary.gamepad.rumble(1000)
//                 secondary.gamepad.rumble(1000)
//             }
//             if (105.0 < elapsedTime.seconds() && elapsedTime.seconds() < 106.0) {
//                 primary.gamepad.rumbleBlips(6)
//                 secondary.gamepad.rumbleBlips(6)
//             }
//             telemetry.addData("pivot pos", hardware.pivot.position);
//             telemetry.addData("left pos", hardware.fingerLeft.position);
//             telemetry.addData("right pos", hardware.fingerRight.position)
//             telemetry.addData("lift dist", lift.distance)
//             telemetry.addData("list state", lift.state)
//             telemetry.addData("cycle state", cycleMachine.state as CycleState)
//             telemetry.addData("out state", outMachine.state as OutputState)
//             telemetry.addData("hz ", 1000 / dt)
//             telemetry.update()
//         }
//         // hardware.stop()
//     }
// }