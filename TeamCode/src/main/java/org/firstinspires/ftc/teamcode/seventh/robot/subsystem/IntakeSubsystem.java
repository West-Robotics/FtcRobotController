package org.firstinspires.ftc.teamcode.seventh.robot.subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.seventh.robot.hardware.Globals;
import org.firstinspires.ftc.teamcode.seventh.robot.hardware.Hardware;

public class IntakeSubsystem {
    private Hardware hardware;

    public enum IntakeState {
        INTAKE,
        STOP,
        SPIT
    } IntakeState intakeState = IntakeState.STOP;
    public enum OuterState {
        STACK_5,
        STACK_4,
        STACK_3,
        STACK_2,
        STACK_1
    } OuterState outerState = OuterState.STACK_1;

    private double lastPower = 0.0;
    private double power = 0.0;
    private int lastAngle = 0;
    private int angle = 0;

    public IntakeSubsystem(Hardware hardware) {
        this.hardware = hardware;
        hardware.intake.setDirection(DcMotorSimple.Direction.FORWARD);
        hardware.intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        hardware.intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hardware.intake.setDirection(DcMotorSimple.Direction.REVERSE);
        hardware.outerPivot.setInverted(true);
        update(IntakeState.STOP, OuterState.STACK_1);
    }

    public void update(IntakeState is, OuterState os) {
        switch (is) {
            case INTAKE:
                power = 0.8;
                break;
            case STOP:
                power = 0.0;
                break;
            case SPIT:
                power = -0.4;
                break;
        }
        switch (os) {
            case STACK_1:
                angle = Globals.STACK_1;
                break;
            case STACK_2:
                angle = Globals.STACK_2;
                break;
            case STACK_3:
                angle = Globals.STACK_3;
                break;
            case STACK_4:
                angle = Globals.STACK_4;
                break;
            case STACK_5:
                angle = Globals.STACK_5;
                break;
        }
        write(power, angle);
    }

    public OuterState higher(OuterState s) {
        switch (s) {
            case STACK_1:
                return OuterState.STACK_2;
            case STACK_2:
                return OuterState.STACK_3;
            case STACK_3:
                return OuterState.STACK_4;
            case STACK_4:
                return OuterState.STACK_5;
            case STACK_5:
                return OuterState.STACK_5;
        }
        return OuterState.STACK_1;
    }

    public OuterState lower(OuterState s) {
        switch (s) {
            case STACK_1:
                return OuterState.STACK_1;
            case STACK_2:
                return OuterState.STACK_1;
            case STACK_3:
                return OuterState.STACK_2;
            case STACK_4:
                return OuterState.STACK_3;
            case STACK_5:
                return OuterState.STACK_4;
        }
        return OuterState.STACK_1;
    }

    private void write(double p, int a) {
        if (lastPower != p) {
            lastPower = p;
            hardware.intake.setPower(p);
        }
        if (lastAngle != a) {
            lastAngle = a;
            hardware.outerPivot.turnToAngle(a);
        }
    }
}
