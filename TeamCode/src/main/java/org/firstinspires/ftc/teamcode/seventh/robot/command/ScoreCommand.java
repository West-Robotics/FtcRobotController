package org.firstinspires.ftc.teamcode.seventh.robot.command;

import org.firstinspires.ftc.teamcode.seventh.robot.hardware.Globals;
import org.firstinspires.ftc.teamcode.seventh.robot.hardware.Hardware;
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.LiftSubsystem;
import org.firstinspires.ftc.teamcode.seventh.robot.subsystem.OuttakeSubsystem;

public class ScoreCommand {
    LiftSubsystem lift;
    OuttakeSubsystem out;
    OuttakeSubsystem.OuttakeState os;

    public enum ScoreState {
        TRANSFER,
        LOCK,
        OUTTAKE_READY,
        OUTTAKE_DROP,
        OUTTAKE_DROP_L,
        OUTTAKE_DROP_R
    } ScoreState scoreState = ScoreState.TRANSFER;

    public ScoreCommand(Hardware hardware) {
        lift = new LiftSubsystem(hardware);
        out = new OuttakeSubsystem(hardware);
    }

    public void update(ScoreState ss, double p) {
        lift.update(p);
        // this function has to capability to request
        // the lift encoder pos a million times: does this matter with bulk reading?
        if (lift.getRightDistance() < Globals.INTERMEDIARY_ZONE_1) {
            if (ss == ScoreState.TRANSFER) {
                os = OuttakeSubsystem.OuttakeState.TRANSFER;
            } else {
                // always assume we want to lock unless specifically requested to open
                os = OuttakeSubsystem.OuttakeState.LOCK;
            }
        } else if (Globals.INTERMEDIARY_ZONE_1 <= lift.getRightDistance() && lift.getRightDistance() <= Globals.INTERMEDIARY_ZONE_2) {
            os = OuttakeSubsystem.OuttakeState.INTERMEDIARY;
        } else if (Globals.INTERMEDIARY_ZONE_2 < lift.getRightDistance()) {
            if (ss == ScoreState.OUTTAKE_READY) {
                os = OuttakeSubsystem.OuttakeState.OUTTAKE_READY;
            } else if (ss == ScoreState.OUTTAKE_DROP) {
                os = OuttakeSubsystem.OuttakeState.OUTTAKE_DROP;
            } else if (ss == ScoreState.OUTTAKE_DROP_L) {
                os = OuttakeSubsystem.OuttakeState.OUTTAKE_DROP_L;
            } else if (ss == ScoreState.OUTTAKE_DROP_R) {
                os = OuttakeSubsystem.OuttakeState.OUTTAKE_DROP_R;
            } else if (ss == ScoreState.LOCK){
                os = OuttakeSubsystem.OuttakeState.INTERMEDIARY;
            } else if (ss == ScoreState.TRANSFER) {
                os = OuttakeSubsystem.OuttakeState.INTERMEDIARY;
            }
        }
        out.update(os);
    }

    public double getLeftDist() {
        return lift.getLeftDistance();
    }

    public double getRightDist() {
        return lift.getRightDistance();
    }
}
