package org.firstinspires.ftc.teamcode.seventh.robot.subsystem;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.seventh.robot.hardware.Hardware;

public class HangSubsystem {
    private Hardware hardware;

    private double power = 0.0;

    public HangSubsystem(Hardware hardware) {
        this.hardware = hardware;
        hardware.hang.setDirection(DcMotorSimple.Direction.FORWARD);
        hardware.hang.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hardware.hang.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        stop();
    }

    public void raise() {
        write(1.0);
    }

    public void stop() {
        write(0.0);
    }

    public void lower() {
        write(-1.0);
    }

    private void write(double power) {
        if (this.power != power) {
            this.power = power;
            hardware.hang.setPower(power);
        }
    }
}
