package org.firstinspires.ftc.teamcode.Technofeathers.Teleop;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
//import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import static java.lang.Thread.sleep;

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
        this.telemetry = telemetry;
    }

    //ADVANCED AUTOMATIONS
    public void scoringPosition() throws InterruptedException {
        //scoringPositionTimer.reset();
        //telemetry.addData("Time Ran ScoringPosition: ", scoringPositionTimer.seconds());
        /*
        if (0 < controller1.left_stick_y) {
            drive.drive(controller1.left_stick_x/2, controller1.left_stick_y/2, controller1.right_stick_x/2);
        }
         */
        grabber.move();
        telemetry.addLine("Grabber moved");
        sleep(500);
        lift.goUp();
        telemetry.addLine("Lift going up");
        sleep(4000);
        lift.stop();
        telemetry.addLine("Lift stopped, pivot will move");
        pivot1.move();
        telemetry.addLine("Scoring position achieved");
    }

    public void pixelDropAndReset() throws InterruptedException {
        //pixelDropAndResetTimer.reset();
        //telemetry.addData("Time Ran PixelDrop + Reset: ", pixelDropAndResetTimer.seconds());
        grabber.move();
        //pivot1.move();
        sleep(1000);
        lift.goDown();
        sleep(1000);
        lift.stop();
    }

    public void intakeRun() throws InterruptedException {
        lift.goUp();
        sleep(4000);
        telemetry.addLine("lift Going Up");
        if (!pivot1.pivotReadyToDrop) {
            pivot1.move();
            telemetry.addLine("Pivot Out");
        }
        lift.stop();
        //will change to lift.go down later
        telemetry.addLine("lift Going Down");
        stopper.move();
        sleep(1000);
        intake.rotateForwards();
        telemetry.addLine("Intake Started");
    }

    public void intakePushOut() {
        intake.rotateBackwards();
    }

    public void intakeStop() throws InterruptedException {
        //intakeStopTimer.reset();
        //telemetry.addData("Time Ran intakeStop: ", intakeStopTimer.seconds());
        if (stopper.stopperDown) {
            stopper.move();
        }
        intake.off();
        while (!lift.maxLimitReached()) {
            lift.goUp();
        }
        sleep(4000);
        if (pivot1.pivotReadyToDrop) {
            pivot1.move();
        }
        if (grabber.grabbedPixels) {
            grabber.move();
        }
        /*
        if (intakeStopTimer.seconds() >= 5) {
            dylanRan = 0;
            //Must NOT recognize truss (should be fine if we drive straight) or oncoming robots (oh noooo); implement color sensor perhaps?
            telemetry.addLine("Dylan will be run as soon as robot detects something within range");
        }
         */
    }

    public boolean liftMaxLimitReached() {
        return lift.maxLimitReached();
    }

    public boolean liftMinLimitReached() {
        return lift.minLimitReached();
    }

    public double liftRotation() {
        return lift.getLiftCurrentRotation();
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