//package org.firstinspires.ftc.teamcode.seventh.opmode.test;
//
//import com.arcrobotics.ftclib.gamepad.GamepadEx;
//import com.arcrobotics.ftclib.gamepad.GamepadKeys;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.sfdev.assembly.state.StateMachine;
//import com.sfdev.assembly.state.StateMachineBuilder;
//
//import org.firstinspires.ftc.teamcode.seventh.robot.command.ScoreCommand;
//import org.firstinspires.ftc.teamcode.seventh.robot.hardware.Hardware;
//
//@TeleOp(name = "ScoreTest")
//public class ScoreTest extends LinearOpMode {
//
//    Hardware hardware = Hardware.getInstance();
//    ScoreCommand score;
//    GamepadEx gamepad;
//    ScoreCommand.ScoreState state = ScoreCommand.ScoreState.INTAKE;
//    StateMachine machine = new StateMachineBuilder()
//            .state(ScoreCommand.ScoreState.INTAKE)
//            .onEnter(() -> state = ScoreCommand.ScoreState.INTAKE)
//            .transition(() -> gamepad.getButton(GamepadKeys.Button.LEFT_BUMPER), ScoreCommand.ScoreState.LOCK)
//            .state(ScoreCommand.ScoreState.LOCK)
//            .onEnter(() -> state = ScoreCommand.ScoreState.LOCK)
//            .transition(() -> gamepad.getButton(GamepadKeys.Button.RIGHT_BUMPER), ScoreCommand.ScoreState.READY)
//            .state(ScoreCommand.ScoreState.READY)
//            .onEnter(() -> state = ScoreCommand.ScoreState.READY)
//            .transition(() -> gamepad.getButton(GamepadKeys.Button.DPAD_DOWN), ScoreCommand.ScoreState.DROP)
//            .transition(() -> gamepad.getButton(GamepadKeys.Button.DPAD_LEFT), ScoreCommand.ScoreState.DROP_L)
//            .transition(() -> gamepad.getButton(GamepadKeys.Button.DPAD_RIGHT), ScoreCommand.ScoreState.DROP_R)
//            .state(ScoreCommand.ScoreState.DROP)
//            .onEnter(() -> state = ScoreCommand.ScoreState.DROP)
//            .transition(() -> gamepad.getButton(GamepadKeys.Button.DPAD_UP), ScoreCommand.ScoreState.INTAKE)
//            .state(ScoreCommand.ScoreState.DROP_L)
//            .onEnter(() -> state = ScoreCommand.ScoreState.DROP_L)
//            .transition(() -> gamepad.getButton(GamepadKeys.Button.DPAD_UP), ScoreCommand.ScoreState.INTAKE)
//            .state(ScoreCommand.ScoreState.DROP_R)
//            .onEnter(() -> state = ScoreCommand.ScoreState.DROP_R)
//            .transition(() -> gamepad.getButton(GamepadKeys.Button.DPAD_UP), ScoreCommand.ScoreState.INTAKE)
//            .build();
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        hardware.init(hardwareMap);
//        score = new ScoreCommand(hardware);
//        gamepad = new GamepadEx(gamepad1);
//
//        waitForStart();
//
//        machine.start();
//        while (opModeIsActive() && !isStopRequested()) {
//            machine.update();
//            score.update(state, gamepad.getLeftY());
//            telemetry.addData("power", gamepad.getLeftY());
////            telemetry.addData("dist", score.getDist());
//            telemetry.addData("state", state.toString());
//            telemetry.addData("position of pivot", hardware.pivot.getPosition());
//            telemetry.update();
//        }
//    }
//}
