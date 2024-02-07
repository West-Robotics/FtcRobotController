package org.firstinspires.ftc.teamcode.Technofeathers.Teleop;


import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH;

import com.qualcomm.robotcore.util.ElapsedTime;

public class Automations extends EggnogTeleopTest{
    ElapsedTime scoringPositionTimer = new ElapsedTime();
    ElapsedTime pixelDropAndResetTimer = new ElapsedTime();
    ElapsedTime intakeRunTimer = new ElapsedTime();
    ElapsedTime intakeStopTimer = new ElapsedTime();


    public void ScoringPosition() {
        scoringPositionTimer.reset();
        if (0 < controller1.left_stick_y) {
            drive.drive(controller1.left_stick_x/2, controller1.left_stick_y/2, controller1.right_stick_x/2);
        }
        while (scoringPositionTimer.seconds() < 0.5) {
            grabber.setPosition(0.67);
        }

        grabbedPixels = 1;
        while (0.5 < scoringPositionTimer.seconds() && scoringPositionTimer.seconds() < 1.4/* && liftTooHigh == 0*/) {
            lift1.setPower(1);
            lift2.setPower(1);
        }
        while (1.4 < scoringPositionTimer.seconds() && scoringPositionTimer.seconds() < 2) {
            pivot1.setPosition(0);
        }
        pivotReadyToDrop = 0;

    }

    public void PixelDropAndReset() {
        pixelDropAndResetTimer.reset();
        while (pixelDropAndResetTimer.seconds() < 0.7) {
            grabber.setPosition(1);
        }

        while (0.7 < pixelDropAndResetTimer.seconds() && pixelDropAndResetTimer.seconds() < 0.8) {
            lift1.setPower(-1);
            lift2.setPower(-1);
        }

        while (0.87 < pixelDropAndResetTimer.seconds() && pixelDropAndResetTimer.seconds() < 1.5) {
            pivot1.setPosition(1);
        }
    }

    public void IntakeRun() {
        intakeRunTimer.reset();
        while (intakeRunTimer.seconds() < 0.5) {
            stopper.setPosition(.9);
        }
        while (1 < intakeRunTimer.seconds() && intakeRunTimer.seconds() < 1.4) {
            intake.setPower(1);
        }
    }

    public void IntakeStop() {
        intakeStopTimer.reset();
        stopper.setPosition(.37);
        intake.setPower(0);
        if (intakeStopTimer.seconds() >= 4) {
            dylanRan = 0;
            //Must NOT recognize truss (should be fine if we drive straight) or oncoming robots (oh noooo); implement color sensor perhaps?
            telemetry.addLine("Dylan will be run as soon as robot detects something within range");
        }
    }
}