package org.firstinspires.ftc.teamcode.workshop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Controller;

@TeleOp(name = "")
public class Teleop extends OpMode {
    private Controller controller;
    private Servo servo;
    private DcMotor motor;

    @Override
    public void init() {
        controller = new Controller(gamepad1);
        servo = hardwareMap.get(Servo.class,  "servo");
        motor = hardwareMap.get(DcMotor.class,  "motor");
    }

    @Override
    public void loop() {
        controller.update();
        if (controller.A()) {
            servo.setPosition(0);
        }
    }
}