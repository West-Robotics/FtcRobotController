package org.firstinspires.ftc.teamcode.Technofeathers.Bibot.Teleop;

//import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Technofeathers.Bibot.Teleop.Subsystems.AirplaneLauncher;
import org.firstinspires.ftc.teamcode.Technofeathers.Bibot.Teleop.Subsystems.DistSense1;
import org.firstinspires.ftc.teamcode.Technofeathers.Bibot.Teleop.Subsystems.DistSense2;
import org.firstinspires.ftc.teamcode.Technofeathers.Bibot.Teleop.Subsystems.DistSense3;
import org.firstinspires.ftc.teamcode.Technofeathers.Bibot.Teleop.Subsystems.Grabber;
import org.firstinspires.ftc.teamcode.Technofeathers.Bibot.Teleop.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Technofeathers.Bibot.Teleop.Subsystems.Lift;
import org.firstinspires.ftc.teamcode.Technofeathers.Bibot.Teleop.Subsystems.Pivot;
import org.firstinspires.ftc.teamcode.Technofeathers.Bibot.Teleop.Subsystems.Stopper;
import org.firstinspires.ftc.teamcode.Technofeathers.Bibot.Teleop.Subsystems.TouchSense1;
import org.firstinspires.ftc.teamcode.Technofeathers.TechnofeathersDrive;
import org.firstinspires.ftc.teamcode.Technofeathers.Bibot.Teleop.Subsystems.*;

public class Functions {
    public TechnofeathersDrive drive;
    public Telemetry telemetry;
    public AirplaneLauncher airplaneLauncher = new AirplaneLauncher();
    public DistSense1 distSense1 = new DistSense1();
    public DistSense2 distSense2 = new DistSense2();
    public DistSense3 distSense3 = new DistSense3();
    public TouchSense1 touchSense1 = new TouchSense1();
    public Grabber grabber = new Grabber();
    public Intake intake = new Intake();
    public Lift lift = new Lift();
    public Pivot pivot1 = new Pivot();
    public Stopper stopper = new Stopper();
    public boolean dpadRightRunning = false;
    public boolean dpadLeftRunning = false;
    public boolean intakeOnRunning = false;
    public boolean intakeOffRunning = false;


    //public boolean higherLevel = false;


    ElapsedTime scoringPositionTimer = new ElapsedTime();
    ElapsedTime pixelDropAndResetTimer = new ElapsedTime();
    ElapsedTime intakeRunTimer = new ElapsedTime();
    ElapsedTime intakeStopTimer = new ElapsedTime();
    public void setUp(HardwareMap hardwareMap, Telemetry telemetry) {
        airplaneLauncher.setUpAirplaneLauncher(hardwareMap);
        distSense1.setUpDistSense1(hardwareMap);
        distSense2.setUpDistSense2(hardwareMap);
        distSense3.setUpDistSense3(hardwareMap);
        touchSense1.setUpTouchSense1(hardwareMap, telemetry);
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
        if (!grabber.grabbedPixels) {
            grabber.move();
            telemetry.addLine("Grabber moved");
        }
        sleep(750);
        lift.goUp();
        telemetry.addLine("Lift going up");
        sleep(500);
        lift.stop();
        telemetry.addLine("Lift stopped, pivot will move");
        if (!pivot1.pivotReadyToDrop) {
            pivot1.move();
        }
        telemetry.addLine("Scoring position achieved");
    }

    public void pixelDropAndReset() throws InterruptedException {
        dpadRightRunning = true;
        //pixelDropAndResetTimer.reset();
        //telemetry.addData("Time Ran PixelDrop + Reset: ", pixelDropAndResetTimer.seconds());
        if (grabber.grabbedPixels) {
            grabber.move();
        }
        sleep(500);
        if (pivot1.pivotReadyToDrop) {
            pivot1.move();
        }
        sleep(750);
        lift.goDown();
        dpadRightRunning = false;
        //sleep(500);
        //TODO: test if this works when touch sensor is pressed
        //can go down for longer if touch sensor works
        //lift.stop();
    }

    public void intakeRun() throws InterruptedException {
        intakeOnRunning = true;
        //TODO: run by encoder for lift
        lift.goUp();
        sleep(500);
        telemetry.addLine("lift Going Up");
        lift.stop();
        if (!stopper.stopperDown) {
            stopper.move();
        }
        if (!stopper.stopperDown) {
            stopper.move();
        }
        sleep(750);
        intake.rotateForwards();
        telemetry.addLine("Intake Started");
        intakeOnRunning = false;
    }

    public void intakeStop() throws InterruptedException {
        intakeOffRunning = true;
        //intakeStopTimer.reset();
        //telemetry.addData("Time Ran intakeStop: ", intakeStopTimer.seconds());
        intake.off();
        if (stopper.stopperDown) {
            stopper.move();
        }

        if (stopper.stopperDown) {
            stopper.move();
        }
        if (grabber.grabbedPixels) {
            grabber.move();
        }
        sleep(750);
        lift.goDown();

        intakeOffRunning = false;
        /*
        if (intakeStopTimer.seconds() >= 5) {
            dylanRan = 0;
            //Must NOT recognize truss (should be fine if we drive straight) or oncoming robots (oh noooo); implement color sensor perhaps?
            telemetry.addLine("Dylan will be run as soon as robot detects something within range");
        }
         */
    }
    public void liftSetPower(Double power) {
        lift.setPower(power);
    }

    public double backdropParallelAngle() {
        distSense2.getDistance();
        distSense3.getDistance();
        return Math.asin((distSense3.getDistance() - distSense2.getDistance())/12.25);
    }

    public void intakePushOut() {
        intake.rotateBackwards();
    }

    public boolean touchSense1Pressed() {
        return touchSense1.pressedDown();
    }

    public void liftResetEncoders() {
        lift.resetEncoders();
    }
    public byte getLiftStatus() {
        return lift.getStatus();
    }

    public boolean getDpadRightStatus() {
        return dpadRightRunning;
    }

    public boolean getDpadLeftStatus() {
        return dpadLeftRunning;
    }

    public boolean getIntakeOnStatus() {
        return intakeOnRunning;
    }

    public boolean getIntakeOffStatus() {
        return intakeOffRunning;
    }

    public void faceBackdrop() {

    }
    /*
    public double theTurnToAlignToBackdrop() {
        distSense2.getDistance();
        distSense3.getDistance();
        //clockwise imposition of distSense1 distSense2 and distSense3
        if (distSense3.getDistance() < distSense2.getDistance()) {
            return (1 - (distSense3.getDistance()/distSense2.getDistance()));
        }
        else if (distSense2.getDistance() < distSense3.getDistance()) {
            return ((distSense3.getDistance()/distSense2.getDistance()) - 1);
        }
    }

     */

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
}