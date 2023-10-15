package org.firstinspires.ftc.teamcode.seventh.robot.command;

import org.firstinspires.ftc.teamcode.seventh.robot.hardware.Globals;
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.LiftSubsystem;
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.OutputSubsystem;

public class CycleCommand {
    IntakeSubsystem intake;
    LiftSubsystem lift;
    OutputSubsystem out;
    OutputSubsystem.OutputState outState = OutputSubsystem.OutputState.LOCK;

    public enum CycleState {
        INTAKE,
        LOCK,
        READY,
        GROUND,
        SPIT
    } CycleState cycleState = CycleState.LOCK;

    public CycleCommand(IntakeSubsystem intake, LiftSubsystem lift, OutputSubsystem out) {
        this.intake = intake;
        this.lift = lift;
        this.out = out;
    }

    // okay so to be clear all this does is update the states (and also other computations, hm maybe
    // i should change that) of each subsystem not actually cause any hardware change
    public void update(CycleState cs, OutputSubsystem.OutputState os) {
        switch (cs) {
            // TODO: worry about only starting intake when lift reaches bottom
            case INTAKE:
                intake.update(IntakeSubsystem.IntakeState.INTAKE);
                lift.update(LiftSubsystem.LiftState.DOWN);
                break;
            case LOCK:
                intake.update(IntakeSubsystem.IntakeState.STOP);
                lift.update(LiftSubsystem.LiftState.DOWN);
                break;
            case READY:
                intake.update(IntakeSubsystem.IntakeState.STOP);
                lift.update(LiftSubsystem.LiftState.UP);
                break;
            case SPIT:
                intake.update(IntakeSubsystem.IntakeState.SPIT);
                lift.update(LiftSubsystem.LiftState.DOWN);
                break;
            case GROUND:
                break;
        }

        if (lift.getDistance() < Globals.INTERMEDIARY_ZONE_1) {
            if (os == OutputSubsystem.OutputState.INTAKE) {
                outState = OutputSubsystem.OutputState.INTAKE;
            } else {
                // always assume we want to lock unless specifically requested to open
                outState = OutputSubsystem.OutputState.LOCK;
            }
        } else if (Globals.INTERMEDIARY_ZONE_1 <= lift.getDistance() && lift.getDistance() <= Globals.INTERMEDIARY_ZONE_2) {
            outState = OutputSubsystem.OutputState.INTERMEDIARY;
        } else if (Globals.INTERMEDIARY_ZONE_2 < lift.getDistance()) {
            if (!(os == OutputSubsystem.OutputState.LOCK || os == OutputSubsystem.OutputState.INTAKE)) {
                outState = os;
            } else {
                outState = OutputSubsystem.OutputState.INTERMEDIARY;
            }
        }
        out.update(outState);
    }

    public double getDistance() { return lift.getDistance(); }
}
