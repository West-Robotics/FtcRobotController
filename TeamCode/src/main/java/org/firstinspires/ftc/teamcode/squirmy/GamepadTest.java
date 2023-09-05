package org.firstinspires.ftc.teamcode.squirmy;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.List;

@TeleOp(name = "Squirmy Gamepad")
public class GamepadTest extends OpMode {

    double x = 0.0;
    double loopTime = 0.0;

    @Override
    public void init() {

    }

    @Override
    public void loop() {
        // try caching values
        x = 0.0;
        x = gamepad1.left_stick_y;
        x = gamepad1.left_stick_x;
        x = gamepad1.right_stick_x;
        x = gamepad1.right_stick_y;
        x = gamepad1.touchpad_finger_1_x;
        x = gamepad1.touchpad_finger_2_x;
        x = gamepad1.left_trigger;
        x = gamepad1.right_trigger;
        double loop = System.nanoTime();
        telemetry.addData("hz", 1000000000 / (loop - loopTime));
        telemetry.addData("loop time", (loop - loopTime)/1000000);
        loopTime = loop;
        telemetry.update();
    }
}
