package org.firstinspires.ftc.teamcode.Technofeathers.Teleop;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
//import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Technofeathers.TechnofeathersTestDrive;
import org.firstinspires.ftc.teamcode.Technofeathers.Teleop.Subsystems.*;

public class Functions {
    public TechnofeathersTestDrive drive;
    public Telemetry telemetry;
    public AirplaneLauncher airplaneLauncher = new AirplaneLauncher();
    public DistSense1 distSense1 = new DistSense1();
    public Grabber grabber = new Grabber();
    public Intake intake = new Intake();
    public Lift lift = new Lift();
    public Pivot pivot1 = new Pivot();
    public Stopper stopper = new Stopper();

    ElapsedTime scoringPositionTimer = new ElapsedTime();
    ElapsedTime pixelDropAndResetTimer = new ElapsedTime();
    ElapsedTime intakeRunTimer = new ElapsedTime();
    ElapsedTime intakeStopTimer = new ElapsedTime();
    public void setUp(HardwareMap hardwareMap, Telemetry telemetry) {
        airplaneLauncher.setUpAirplaneLauncher(hardwareMap);
        distSense1.setUpDistSense1(hardwareMap);
        grabber.setUpGrabber(hardwareMap);
        intake.setUpIntake(hardwareMap);
        lift.setUpLift(hardwareMap);
        pivot1.setUpPivot(hardwareMap, telemetry);
        stopper.setUpStopper(hardwareMap);
    }

    //ADVANCED AUTOMATIONS
    public void scoringPosition() {
        scoringPositionTimer.reset();
        /*
        if (0 < controller1.left_stick_y) {
            drive.drive(controller1.left_stick_x/2, controller1.left_stick_y/2, controller1.right_stick_x/2);
        }
         */
        while (scoringPositionTimer.seconds() < 0.5) {
            grabber.move();
        }
        while (0.5 < scoringPositionTimer.seconds() && scoringPositionTimer.seconds() < 1.4 && !lift.limitReached()) {
            lift.goUp();
        }
        while (1.4 < scoringPositionTimer.seconds() && scoringPositionTimer.seconds() < 2) {
            pivot1.move();
        }
    }

    public void pixelDropAndReset() {
        pixelDropAndResetTimer.reset();
        while (pixelDropAndResetTimer.seconds() < 0.7) {
            grabber.move();
        }

        while (0.7 < pixelDropAndResetTimer.seconds() && pixelDropAndResetTimer.seconds() < 0.8) {
            lift.goDown();
        }

        while (0.87 < pixelDropAndResetTimer.seconds() && pixelDropAndResetTimer.seconds() < 1.5) {
            pivot1.move();
        }
    }

    public void intakeRun() {
        intakeRunTimer.reset();
        while (intakeRunTimer.seconds() < 0.5) {
            stopper.move();
        }
        while (1 < intakeRunTimer.seconds() && intakeRunTimer.seconds() < 1.4) {
            intake.rotateForwards();
        }
    }

    public void intakePushOut() {
        intake.rotateBackwards();
    }

    public void intakeStop() {
        intakeStopTimer.reset();
        stopper.move();
        intake.off();
        /*
        if (intakeStopTimer.seconds() >= 5) {
            dylanRan = 0;
            //Must NOT recognize truss (should be fine if we drive straight) or oncoming robots (oh noooo); implement color sensor perhaps?
            telemetry.addLine("Dylan will be run as soon as robot detects something within range");
        }
         */
    }

    //Manual Controls
    public void launchAirplane() {
        airplaneLauncher.releaseAirplane();
    }
    public void liftGoUp() {
        lift.goUp();
    }
    public void liftGoDown() {
        lift.goDown();
    }
    public void liftStop() {
        lift.stop();
    }
    public void grabberMove() {
        grabber.move();
    }
    public void pivotMove() {
        pivot1.move();
    }
    public void stopperMove() {
        stopper.move();
    }
    public void stopperUp() {
        stopper.move();
    }
}