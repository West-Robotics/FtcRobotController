package org.firstinspires.ftc.teamcode.seventh.robot.subsystem;

import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.seventh.robot.hardware.Globals;
import org.firstinspires.ftc.teamcode.seventh.robot.hardware.Hardware;

public class OutputSubsystem extends Subsystem {
    public enum OutputState {
        INTAKE,
        LOCK,
        INTERMEDIARY,
        INTERMEDIARY_1,
        READY,
        DROP,
        DROP_L,
        DROP_R,
        PLOP_READY,
        PLOP_L,
        PLOP_R,
        POOP,
    } OutputState outputState = OutputState.LOCK;

    private double pivAngle = 0;
    private double leftAng = 0;
    private double rightAng = 0;
    private double lastPivotAngle = 0;
    private double lastLeftAngle = 0;
    private double lastRightAngle = 0;

    private Hardware hardware;
    public OutputSubsystem(Hardware hardware) {
        this.hardware = hardware;
        // this doesn't work cause blue can't be programmed lol
        hardware.pivot.setDirection(Servo.Direction.FORWARD);
        hardware.pivot.setPwmRange(new PwmControl.PwmRange(500, 2500));
        hardware.fingerLeft.setDirection(Servo.Direction.FORWARD);
        hardware.fingerLeft.setPwmRange(new PwmControl.PwmRange(500, 2500));
        hardware.fingerRight.setDirection(Servo.Direction.FORWARD);
        hardware.fingerRight.setPwmRange(new PwmControl.PwmRange(500, 2500));
        update(outputState);
    }

    public void read() {}

    public void update(OutputState os) {
        switch (os) {
            case INTAKE:
                pivAngle = Globals.PIVOT_INTAKE;
                leftAng = Globals.FINGER_L_OPEN;
                rightAng = Globals.FINGER_R_OPEN;
                break;
            case LOCK:
                pivAngle = Globals.PIVOT_INTAKE;
                leftAng = Globals.FINGER_L_CLOSE;
                rightAng = Globals.FINGER_R_CLOSE;
                break;
            case INTERMEDIARY:
                pivAngle = Globals.PIVOT_INTERMEDIARY;
                leftAng = Globals.FINGER_L_CLOSE;
                rightAng = Globals.FINGER_R_CLOSE;
                break;
            case READY:
                pivAngle = Globals.PIVOT_OUTTAKE;
                leftAng = Globals.FINGER_L_CLOSE;
                rightAng = Globals.FINGER_R_CLOSE;
                break;
            case DROP:
                pivAngle = Globals.PIVOT_OUTTAKE;
                leftAng = Globals.FINGER_L_OPEN;
                rightAng = Globals.FINGER_R_OPEN;
                break;
            case DROP_L:
                pivAngle = Globals.PIVOT_OUTTAKE;
                leftAng = Globals.FINGER_L_OPEN;
                rightAng = Globals.FINGER_R_CLOSE;
                break;
            case DROP_R:
                pivAngle = Globals.PIVOT_OUTTAKE;
                leftAng = Globals.FINGER_L_CLOSE;
                rightAng = Globals.FINGER_R_OPEN;
                break;
            case PLOP_READY:
                pivAngle = Globals.PIVOT_PLOP;
                leftAng = Globals.FINGER_L_CLOSE;
                rightAng = Globals.FINGER_R_CLOSE;
                break;
            case PLOP_L:
                pivAngle = Globals.PIVOT_PLOP;
                leftAng = Globals.FINGER_L_OPEN;
                rightAng = Globals.FINGER_R_CLOSE;
                break;
            case PLOP_R:
                pivAngle = Globals.PIVOT_PLOP;
                leftAng = Globals.FINGER_L_CLOSE;
                rightAng = Globals.FINGER_R_OPEN;
                break;
            case POOP:
                pivAngle = Globals.PIVOT_POOP;
                leftAng = Globals.FINGER_L_CLOSE;
                rightAng = Globals.FINGER_R_CLOSE;
        }
    }

    public void write() {
        // TODO: add small tolerances because imperfections
        if (pivAngle != lastPivotAngle) {
            lastPivotAngle = pivAngle;
            hardware.pivot.setPosition(pivAngle);
        }
        if (leftAng != lastLeftAngle) {
            lastLeftAngle = leftAng;
            hardware.fingerLeft.setPosition(leftAng);
        }
        if (rightAng != lastRightAngle) {
            lastRightAngle = rightAng;
            hardware.fingerRight.setPosition(rightAng);
        }
    }
}
