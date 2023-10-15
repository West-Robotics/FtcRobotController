package org.firstinspires.ftc.teamcode.seventh.robot.subsystem;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.seventh.robot.hardware.Hardware;

public class HangSubsystem extends Subsystem {
    public enum HangState {
        STOP,
        RAISE,
        LOWER
    } HangState state = HangState.STOP;
    private double power = 0.0;
    private double lastPower = 0.0;

    private Hardware hardware;

    public HangSubsystem(Hardware hardware) {
        this.hardware = hardware;
        hardware.hang.setDirection(DcMotorSimple.Direction.FORWARD);
        // does this actually save any power lol, i don't think it does cause just back emf
        hardware.hang.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        hardware.hang.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        update(state);
    }

    @Override
    public void read() {}

    public void update(HangState s) {
        state = s;
        switch (state) {
            case STOP:
                power = 0.0;
                break;
            case RAISE:
                power = 1.0;
                break;
            case LOWER:
                power = -1.0;
                break;
        }
    }

    @Override
    public void write() {
        if (lastPower != power) {
            lastPower = power;
            hardware.hang.setPower(power);
        }
    }
}
