// TODO: Add PID tuning with controller (methods for changing variables are done already)
// TODO: Update preset values, add presets for stack
// TODO: Add slow descent + encoder reset on limit switch
// TODO: Set up all FSMs
// TODO: Tune drive PID
package org.firstinspires.ftc.teamcode.WizardBot;

import static java.lang.Math.abs;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.teamcode.Controller;
import org.firstinspires.ftc.teamcode.WizardBot.Subsystems.WizardLift;
import org.firstinspires.ftc.teamcode.WizardBot.Subsystems.IntakeFSM;
import org.firstinspires.ftc.teamcode.WizardBot.Subsystems.WristReset;
import org.firstinspires.ftc.teamcode.WizardBot.Subsystems.LiftZeroFSM;
import org.firstinspires.ftc.teamcode.WizardBot.Subsystems.OuttakeFSM;
import org.firstinspires.ftc.teamcode.WizardBot.Subsystems.StackSetupFSM;
import org.firstinspires.ftc.teamcode.WizardBot.Subsystems.StackPickupFSM;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import com.arcrobotics.ftclib.hardware.SimpleServo;

@TeleOp(name = "Wizard Bot Teleop Child Safe")
public class WizardTeleopChildSafe extends OpMode {

    private WizardBotDrive drive;
    private Controller controller;
    private Controller controller2;
    private WizardLift lift;
    private IntakeFSM intakeFSM;
    private OuttakeFSM outtakeFSM;
    private LiftZeroFSM liftZero;
    private StackSetupFSM stackSetupFSM;
    private StackPickupFSM stackPickupFSM;
    private WristReset teleopInit;
    private TouchSensor liftLimit;
    private DistanceSensor intakeSensor;
    private ServoEx clawServo;
    private ServoEx wristServo;
    private ServoEx pivotServo1;
    private ServoEx pivotServo2;
    private DcMotor lift1;
    private DcMotor lift2;
    private DcMotor intake1;
    private DcMotor intake2;
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    double leftTrigger = 0;
    double rightTrigger = 0;
    double leftTrigger2 = 0;
    double rightTrigger2 = 0;
    int slowMode = 0;
    int intaking = 0;
    int stackLevel = 0;
    int driveControl = 0;

    public WizardTeleopChildSafe() {
    }

    @Override
    public void init() {
        drive = new WizardBotDrive(this, hardwareMap);
        controller = new Controller(gamepad1);
        controller2 = new Controller(gamepad2);
        lift = new WizardLift();
        intakeFSM = new IntakeFSM();
        outtakeFSM = new OuttakeFSM();
        stackSetupFSM = new StackSetupFSM();
        stackPickupFSM = new StackPickupFSM();
        teleopInit = new WristReset();
        liftZero = new LiftZeroFSM();

        clawServo = new SimpleServo(hardwareMap, "clawServo", 0, 360);
        wristServo = new SimpleServo(hardwareMap, "wristServo", 0, 360);
        pivotServo1 = new SimpleServo(hardwareMap, "pivotServo1", 0, 360);
        pivotServo2 = new SimpleServo(hardwareMap, "pivotServo2", 0, 360);
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        liftLimit = hardwareMap.get(TouchSensor.class, "liftLimit");
        intakeSensor = hardwareMap.get(DistanceSensor.class, "intakeSensor");
        lift1 = hardwareMap.get(DcMotor.class, "lift1");
        lift2 = hardwareMap.get(DcMotor.class, "lift2");
        intake1 = hardwareMap.get(DcMotor.class, "intake1");
        intake2 = hardwareMap.get(DcMotor.class, "intake2");

        lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lift.liftInit(hardwareMap);
        intakeFSM.intakeInit(hardwareMap);
        outtakeFSM.outtakeInit(hardwareMap);
        stackSetupFSM.init(hardwareMap);
        stackPickupFSM.init(hardwareMap);
        teleopInit.teleopInit(hardwareMap);
        liftZero.liftZeroInit(hardwareMap, lift);
    }

    @Override
    public void loop() {
        controller.update();
        controller2.update();

//        telemetry.addData("LiftLimit", liftLimit.isPressed());
//        telemetry.addData("LiftEncdr", lift1.getCurrentPosition());
//        telemetry.addData("LiftEncdr2", lift2.getCurrentPosition());
//        telemetry.addData("SlowMode?", slowMode);
//        telemetry.addData("HorizontalLift P", lift.getP());
//        telemetry.addData("HorizontalLift I", lift.getI());
//        telemetry.addData("HorizontalLift D", lift.getD());
//        telemetry.addData("FF", lift.getFF());
//        telemetry.addData("Lift1Power", lift1.getPower());
//        telemetry.addData("Lift2Power", lift2.getPower());
//        telemetry.addData("PID Setpoint", lift.getSetpoint());
//        telemetry.addData("PID Power", lift.getPIDPower());
//        telemetry.addData("Encoder Average", lift.getEncoderAverage());
//        telemetry.addData("Distance Sensor", intakeSensor.getDistance(DistanceUnit.INCH));
//        telemetry.addData("leftTriggerOld", leftTrigger);
//        telemetry.addData("rightTriggerOld", rightTrigger);
//        telemetry.addData("Drive P", drive.getDriveP());
//        telemetry.addData("Drive I", drive.getDriveI());
//        telemetry.addData("Drive D", drive.getDriveD());
//        telemetry.addData("HorizontalLift Resting", liftZero.getLiftResting());
//        telemetry.addData("Arrived", lift.arrived());
        telemetry.addData("Whos Driving", driveControl);
        telemetry.update();

        //Drive Controls
        if (controller.rightBumperOnce()) {
            if (slowMode == 0) {
                slowMode = 1;
            } else {
                slowMode = 0;
            }
        }
        if (controller.leftBumperOnce()) {
            if (driveControl == 0) {
                driveControl = 1;
            } else {
                driveControl = 0;
            }
        }
        if (driveControl == 0) {
            if (slowMode == 0) {
                drive.drive(controller2.left_stick_x, controller2.left_stick_y, controller2.right_stick_x);
            } else {
                drive.drive(controller2.left_stick_x*2/3, controller2.left_stick_y / 2, controller2.right_stick_x / 2);
            }
        } else {
            if (slowMode == 0) {
                drive.drive(controller.left_stick_x, controller.left_stick_y, controller.right_stick_x);
            } else {
                drive.drive(controller.left_stick_x*2/3, controller.left_stick_y/2, controller.right_stick_x /2);
        }
        }

        //Scoring
        if ((controller2.right_trigger > 0.5 && rightTrigger == 0 && controller2.left_trigger == 0) || (controller.right_trigger > 0.5 && rightTrigger2 == 0 && controller.left_trigger == 0)) {
            liftZero.startLiftZeroFSM();
            outtakeFSM.startOuttakeFSM();
            liftZero.setLiftResting(0);
        }
        liftZero.liftZero();
        outtakeFSM.outtake();

        //Intake
        if ((controller2.left_trigger > 0.5 && leftTrigger == 0 && controller2.right_trigger == 0) || (controller.left_trigger > 0.5 && leftTrigger2 == 0 && controller.right_trigger == 0)) {
            intake1.setPower(0.3);
            intake2.setPower(-0.3);
            intaking = 1;
        }

        // Intake Automation
        if (intakeSensor.getDistance(DistanceUnit.INCH) < 5 && intaking == 1) {
            intake1.setPower(0);
            intake2.setPower(0);
            intaking = 0;
            intakeFSM.resetTimer();
            intakeFSM.startIntakeFSM();
        }
        intakeFSM.intake();
        if (intaking == 1) {
            clawServo.turnToAngle(103);
        }

        //Reset Automation
        if (controller.left_trigger == 1 && controller.right_trigger == 1) {
            teleopInit.resetTimer();
            teleopInit.startTeleopInit();
        }
        teleopInit.init();

        //Raising HorizontalLift
        if (controller.XOnce()) {
            lift.setLiftPos(850);
            liftZero.setLiftResting(0);
            slowMode = 1;
        } else if (controller.YOnce()) {
            lift.setLiftPos(1750);
            liftZero.setLiftResting(0);
            slowMode = 1;
        } else if (controller.BOnce()) {
            lift.setLiftPos(2500);
            liftZero.setLiftResting(0);
            slowMode = 1;
        }
        if (lift.getSetpoint() == 850 || lift.getSetpoint() == 1750 || lift.getSetpoint() == 2500) {
            pivotServo1.turnToAngle(310);
            pivotServo2.turnToAngle(50);
        }

        //Manual HorizontalLift Control
        if (controller.dpadUpOnce()) {
            lift.setLiftPos(lift.getSetpoint() + 100);
        }

        //Intake Reverse
        if (controller.startOnce() && intake1.getPower() == 0) {
            intake1.setPower(-0.2);
            intake2.setPower(0.2);
        } else if (controller.backOnce() && intake1.getPower() == 0) {
            intake1.setPower(-0.7);
            intake2.setPower(0.7);
        } else if ((controller.startOnce() || controller.backOnce()) && intake1.getPower() < 1) {
            intake1.setPower(0);
            intake2.setPower(0);
            teleopInit.resetTimer();
            teleopInit.startTeleopInit();
        }

        //HorizontalLift PID Toggle and Trigger Toggle
        if (controller2.left_trigger < 0.1) {
            leftTrigger = 0;
        }
        if (controller2.right_trigger < 0.1) {
            rightTrigger = 0;
        }
        if (controller.left_trigger < 0.1) {
            leftTrigger2 = 0;
        }
        if (controller.right_trigger < 0.1) {
            rightTrigger2 = 0;
        }
        if (liftZero.getLiftResting() == 0) {
            lift.moveLift();
        }

        //Stack Access HorizontalLift Controls
        if (controller.dpadRightOnce()) {
            stackLevel = stackLevel+1;
            lift.setLiftPos(100 + 100*stackLevel);
            liftZero.setLiftResting(0);
            stackSetupFSM.resetTimer();
            stackSetupFSM.startStackSetupFSM();
            slowMode = 1;
        }
        stackSetupFSM.setup();

        //Remove From Stack
        if (controller.AOnce()) {
            stackPickupFSM.resetTimer();
            stackPickupFSM.startStackPickupFSM();
            stackLevel = 0;
            slowMode = 0;
        }
        stackPickupFSM.pickup();
    }
}

//        //PID Tuning
//        // P controlled with bumpers
//        if (controller2.dpadLeftOnce()) {
//            lift.changeLiftP(-0.001);
//        //    drive.changeDriveP(-0.001);
//        } else if (controller2.dpadRightOnce()) {
//            lift.changeLiftP(0.001);
//        //    drive.changeDriveP(0.001);
//        }
//        // D controlled with A and Y
//        if (controller2.AOnce()) {
//            lift.changeLiftD(-0.001);
//        //    drive.changeDriveD(-0.001);
//        } else if (controller2.YOnce()) {
//            lift.changeLiftD(0.001);
//        //    drive.changeDriveD(0.001);
//        }
//        // FF controlled with dpad
//        if (controller2.dpadDownOnce()) {
//            lift.changeLiftFF(-0.01);
//        } else if (controller2.dpadUpOnce()) {
//            lift.changeLiftFF(0.01);
//        }
//        // I controlled with X and B
//        if (controller2.XOnce()) {
//            lift.changeLiftI(-0.001);
//        //    drive.changeDriveI(-0.001);
//        } else if (controller2.BOnce()) {
//            lift.changeLiftI(0.001);
//        //    drive.changeDriveI(0.001);
//        }
//
//
//        //Manual HorizontalLift Control for FF Tuning
//        if (controller.left_trigger == 1) {
//            lift1.setPower(-.5);
//            lift2.setPower(.5);
//            liftResting = 1;
//        } else if (controller.right_trigger == 1) {
//            lift1.setPower(1);
//            lift2.setPower(-1);
//            liftResting = 1;
//        } else {
//            liftResting = 0;
//        }

