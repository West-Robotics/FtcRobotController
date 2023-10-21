package org.firstinspires.ftc.teamcode.seventh.robot.subsystem;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.PIDController;
import org.firstinspires.ftc.teamcode.seventh.robot.hardware.Globals;
import org.firstinspires.ftc.teamcode.seventh.robot.hardware.Hardware;

public class LiftSubsystem extends Subsystem {
    public enum LiftState {
        DOWN,
        UP
    } public LiftState liftState = LiftState.DOWN;


    private double position = 0.0;
    private double power = 0.0;
    private double lastPower = 0.0;
    private boolean pressed = true;
//    private double power = 0.0;
    PIDController liftPid = new PIDController(Globals.LIFT_P, Globals.LIFT_I, Globals.LIFT_D);
    private Hardware hardware;

    public LiftSubsystem(Hardware hardware) {
        this.hardware = hardware;
        hardware.liftLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        hardware.liftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hardware.liftLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hardware.liftRight.setDirection(DcMotorSimple.Direction.REVERSE);
        hardware.liftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hardware.liftRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hardware.liftLeftEnc.setDirection(Motor.Direction.FORWARD);
        hardware.liftLeftEnc.setDistancePerPulse(Globals.LIFT_DISTANCE_PER_PULSE);
        hardware.liftLeftEnc.reset();
        hardware.liftRightEnc.setDirection(Motor.Direction.FORWARD);
        hardware.liftRightEnc.setDistancePerPulse(Globals.LIFT_DISTANCE_PER_PULSE);
        hardware.liftRightEnc.reset();
        liftPid.setOutputRange(0, 0.5);
        liftPid.reset();
        liftPid.enable();
        update(liftState);
    }

    public void read() {
        position = hardware.liftLeftEnc.getDistance();
        pressed = hardware.limit.isPressed();
    }

    public void update(LiftState s) {
        switch (s) {
            case UP:
                liftPid.setSetpoint(Globals.LIFT_MAX);
                liftState = LiftState.UP;
                break;
            case DOWN:
                liftPid.setSetpoint(Globals.LIFT_MIN);
                liftState = LiftState.DOWN;
                break;
        }
        power = liftPid.performPID(position);
        write();
    }

    public void write() {
        if (pressed) {
            if (position != 0.0) {
                hardware.liftLeftEnc.reset();
            }
            if (power < 0) {
                power = 0;
            }
        } else if (power < 0) {
            power = power/1.5;
        }
        if (lastPower != power) {
            lastPower = power;
            hardware.liftLeft.setPower(power);
            hardware.liftRight.setPower(power);
        }
    }

    public double getDistance() { return position; }
}
