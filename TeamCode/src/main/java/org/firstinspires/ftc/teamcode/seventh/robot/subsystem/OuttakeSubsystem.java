package org.firstinspires.ftc.teamcode.seventh.robot.subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.seventh.robot.hardware.Globals;
import org.firstinspires.ftc.teamcode.seventh.robot.hardware.Hardware;

public class OuttakeSubsystem {
    private Hardware hardware;

    public enum OuttakeState {
        TRANSFER,
        LOCK,
        INTERMEDIARY,
        OUTTAKE_READY,
        OUTTAKE_DROP,
        OUTTAKE_DROP_L,
        OUTTAKE_DROP_R,
    } OuttakeState outtakeState = OuttakeState.TRANSFER;

    private double lastPivotAngle = 0;
    private double lastLeftAngle = 0;
    private double lastRightAngle = 0;

    public OuttakeSubsystem(Hardware hardware) {
        this.hardware = hardware;
        // this doesn't work cause blue can't be programmed lol
        hardware.pivot.setDirection(Servo.Direction.FORWARD);
        hardware.pivot.setPwmRange(new PwmControl.PwmRange(500, 2500));
        hardware.fingerLeft.setDirection(Servo.Direction.FORWARD);
        hardware.fingerLeft.setPwmRange(new PwmControl.PwmRange(500, 2500));
        hardware.fingerRight.setDirection(Servo.Direction.FORWARD);
        hardware.fingerRight.setPwmRange(new PwmControl.PwmRange(500, 2500));
        update(outtakeState);
    }

    public void update(OuttakeState os) {
        switch (os) {
            case TRANSFER:
                write(Globals.PIVOT_TRANSFER, Globals.FINGER_L_OPEN, Globals.FINGER_R_OPEN);
                break;
            case LOCK:
                write(Globals.PIVOT_TRANSFER, Globals.FINGER_L_CLOSE, Globals.FINGER_R_CLOSE);
                break;
            case INTERMEDIARY:
                write(Globals.PIVOT_INTERMEDIARY, Globals.FINGER_L_CLOSE, Globals.FINGER_R_CLOSE);
                break;
            case OUTTAKE_READY:
                write(Globals.PIVOT_OUTTAKE, Globals.FINGER_L_CLOSE, Globals.FINGER_R_CLOSE);
                break;
            case OUTTAKE_DROP:
                write(Globals.PIVOT_OUTTAKE, Globals.FINGER_L_OPEN, Globals.FINGER_R_OPEN);
                break;
            case OUTTAKE_DROP_L:
                write(Globals.PIVOT_OUTTAKE, Globals.FINGER_L_OPEN, Globals.FINGER_R_CLOSE);
                break;
            case OUTTAKE_DROP_R:
                write(Globals.PIVOT_OUTTAKE, Globals.FINGER_L_CLOSE, Globals.FINGER_R_OPEN);
                break;
        }
    }

    private void write(double p, double l, double r) {
        // TODO: add small tolerances cause imperfections
        if (lastPivotAngle != p) {
            lastPivotAngle = p;
            hardware.pivot.setPosition(p);
        }
        if (lastLeftAngle != l) {
            lastLeftAngle = l;
            hardware.fingerLeft.setPosition(l);
        }
        if (lastRightAngle != r) {
            lastRightAngle = r;
            hardware.fingerRight.setPosition(r);
        }
    }
}
