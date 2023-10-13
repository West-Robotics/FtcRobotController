package org.firstinspires.ftc.teamcode.seventh.robot.command;

import org.firstinspires.ftc.teamcode.seventh.robot.hardware.Globals;
import org.firstinspires.ftc.teamcode.seventh.robot.hardware.Hardware;
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.LiftSubsystem;
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.OutputSubsystem;

public class ScoreCommand {
    LiftSubsystem lift;
    OutputSubsystem out;
    OutputSubsystem.OutputState outState = OutputSubsystem.OutputState.LOCK;

    public enum ScoreState {
        INTAKE,
        LOCK,
        READY,
        GROUND
    } ScoreState scoreState = ScoreState.LOCK;

    public ScoreCommand(Hardware hardware) {
        lift = new LiftSubsystem(hardware);
        out = new OutputSubsystem(hardware);
    }

    public void update(ScoreState ss, OutputSubsystem.OutputState os, double p) {
        lift.update(p);
        // this function has to capability to request
        // the lift encoder pos a million times: does this matter with bulk reading?
        if (lift.getLeftDistance() < Globals.INTERMEDIARY_ZONE_1) {
            if (os == OutputSubsystem.OutputState.INTAKE) {
                outState = OutputSubsystem.OutputState.INTAKE;
            } else {
                // always assume we want to lock unless specifically requested to open
                outState = OutputSubsystem.OutputState.LOCK;
            }
        } else if (Globals.INTERMEDIARY_ZONE_1 <= lift.getLeftDistance() && lift.getLeftDistance() <= Globals.INTERMEDIARY_ZONE_2) {
            outState = OutputSubsystem.OutputState.INTERMEDIARY;
        } else if (Globals.INTERMEDIARY_ZONE_2 < lift.getLeftDistance()) {
            if (!(os == OutputSubsystem.OutputState.LOCK || os == OutputSubsystem.OutputState.INTAKE)) {
                outState = os;
            } else {
                outState = OutputSubsystem.OutputState.INTERMEDIARY;
            }
        }
        out.update(outState);
    }

    public double getLeftDist() {
        return lift.getLeftDistance();
    }

    public double getRightDist() {
        return lift.getRightDistance();
    }
}
