package org.firstinspires.ftc.teamcode.seventh.robot.subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.seventh.robot.hardware.Globals;
import org.firstinspires.ftc.teamcode.seventh.robot.hardware.Hardware;

public class IntakeSubsystem extends Subsystem {
    public enum IntakeState {
        INTAKE,
        STOP,
        SPIT;

        private int rollerHeight = 1;
        public void raise() {
            if (rollerHeight != 5) {
                rollerHeight++;
            }
        }
        public void lower() {
            if (rollerHeight != 1) {
                rollerHeight--;
            }
        }
    } IntakeState intakeState = IntakeState.STOP;

    private double power = 0.0;
    private double lastPower = 0.0;
//    private double lastAngle = 0;
//    private double angle = 0;
    private Hardware hardware;

    public IntakeSubsystem(Hardware hardware) {
        this.hardware = hardware;
        hardware.intake.setDirection(DcMotorSimple.Direction.REVERSE);
        hardware.intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        hardware.intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        hardware.outerPivotLeft.setDirection(Servo.Direction.FORWARD);
//        hardware.outerPivotLeft.setPwmRange(new PwmControl.PwmRange(500, 2500));
//        hardware.outerPivotRight.setDirection(Servo.Direction.REVERSE);
//        hardware.outerPivotRight.setPwmRange(new PwmControl.PwmRange(500, 2500));
        update(intakeState);
    }

    // maybe add jam detection?
    @Override
    public void read() {}

    public void update(IntakeState s) {
        switch (s) {
            case INTAKE:
                power = 1;
                break;
            case STOP:
                power = 0.0;
                break;
            case SPIT:
                power = -0.4;
                break;
        }
        write();
    }

    @Override
    public void write() {
        if (lastPower != power) {
            lastPower = power;
            hardware.intake.setPower(power);
        }
//        if (lastAngle != a) {
//            lastAngle = a;
//            hardware.outerPivotLeft.setPosition(a);
//            hardware.outerPivotRight.setPosition(a);
//        }
    }

    public IntakeState getState() {
        return intakeState;
    }
}
