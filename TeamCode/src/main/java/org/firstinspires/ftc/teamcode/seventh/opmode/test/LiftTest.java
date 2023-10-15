package org.firstinspires.ftc.teamcode.seventh.opmode.test;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;
import com.sun.source.doctree.StartElementTree;

import org.firstinspires.ftc.teamcode.seventh.robot.hardware.Hardware;
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.LiftSubsystem;

@TeleOp(name = "LiftTest")
public class LiftTest extends LinearOpMode {

    Hardware hardware = Hardware.getInstance();
    LiftSubsystem lift;
    GamepadEx gamepad;

    @Override
    public void runOpMode() throws InterruptedException {
        hardware.init(hardwareMap);
        gamepad = new GamepadEx(gamepad1);
        lift = new LiftSubsystem(hardware);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
//            lift.update(gamepad.getLeftY()/2.0);
            telemetry.addData("power", gamepad.getLeftY()/2.0);
//            telemetry.addData("dist", lift.getDistance());
            telemetry.update();
        }
    }
}
