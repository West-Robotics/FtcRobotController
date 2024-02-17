package org.firstinspires.ftc.teamcode.Technofeathers.Teleop;

import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Controller;
import org.firstinspires.ftc.teamcode.Technofeathers.TechnofeathersPDTest;
import org.firstinspires.ftc.teamcode.Technofeathers.TechnofeathersTestDrive;

@TeleOp(name = "EggnogTeleopTest")
public class EggnogTeleopTest extends OpMode {
    private TechnofeathersPDTest test = new TechnofeathersPDTest(0.1);
    //smaller kp = slowing down earlier
    //bigger kp = slowing down later
    public TechnofeathersTestDrive drive;
    public Controller controller1;
    public Controller controller2;
    /*
    public Servo pivot1;
    public Servo grabber;
    public Servo airplaneLauncher;
    public DcMotor lift1;
    public DcMotor lift2;
    public DcMotor intake;
    public Servo stopper;

     */
    public DistanceSensor distSense1;
    //public Telemetry telemetry;
    //private int i = 0;
    //private int j = 0;
    //double lift1CurrentRotation = lift1.getCurrentPosition()/537.7;
    public int intakeOn = 0;
    public int planeLaunched = 0;
    //public byte dylanRan = 0;
    //public byte daveRan = 0;
    public int grabbedPixels = 0;
    public int pivotReadyToDrop = 1;

    public int placeholderE = 1;
    public int placeholderF = 1;

    public int placeholderG = 1;
    public int placeholderH = 1;
    public int placeholderI = 1;
    public boolean dpadRightRunning = false;
    public boolean dpadLeftRunning = false;
    public boolean intakeOnRunning = false;
    public boolean intakeOffRunning = false;



    public Functions functions = new Functions();


    public ElapsedTime timer = new ElapsedTime();

    @Override
    public void init() {
        drive = new TechnofeathersTestDrive();
        controller1 = new Controller(gamepad1);
        controller2 = new Controller(gamepad2);
        /*
        pivot1 = hardwareMap.get(Servo.class,  "pivot1");
        grabber = hardwareMap.get(Servo.class, "grabber");
        lift1 = hardwareMap.get(DcMotor.class,  "lift1");
        lift2 = hardwareMap.get(DcMotor.class, "lift2");
        lift1.setDirection(DcMotorSimple.Direction.FORWARD);
        lift2.setDirection(DcMotorSimple.Direction.REVERSE);
        lift1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake = hardwareMap.get(DcMotor.class, "intake");
        stopper = hardwareMap.get(Servo.class, "stopper");
        airplaneLauncher = hardwareMap.get(Servo.class, "airplaneLauncher");
        distSense1 = hardwareMap.get(DistanceSensor.class, "distSense1");

        lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset the motor encoder
        lift1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // Turn the motor back on when we are done
        lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset the motor encoder
        lift2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // Turn the motor back on when we are done
        //pivot1.setPosition(1);

         */
        drive.setupMotors(hardwareMap);
        functions.setUp(this.hardwareMap,telemetry);

    }

    @Override
    public void loop() {
        controller1.update();
        controller2.update();
        drive.drive(controller1.left_stick_x, -controller1.left_stick_y, controller1.right_stick_x);
        functions.liftSetPower(controller2.left_stick_y);
        telemetry.addData("Lift Current Rotation: ", functions.liftRotation());
        telemetry.addData("DpadRightRunning ", functions.dpadRightRunning);
        /*
        if (functions.liftMinLimitReached()) {
            functions.liftStop();
            telemetry.addLine("Lift Min Limit Reached");
        }

         */
        //lift
        if (functions.touchSense1Pressed() && functions.getLiftStatus() == -1) {
            functions.liftStop();
        }

        if (controller2.leftBumper()) {
            functions.liftGoUp();
            telemetry.addLine("Lift go up");
        }
        else if (controller2.rightBumper() && !functions.touchSense1Pressed()) {
            functions.liftGoDown();
        }
        else {
            functions.liftStop();
        }


        telemetry.addData("Lift Status: ", functions.getLiftStatus());
       // if (distSense1.getDistance(INCH) <= 10 && 0 < controller2.left_stick_y) {
          //  drive.drive(controller2.left_stick_x/2, controller2.left_stick_y/2, controller2.right_stick_x/2);
            //functions.scoringPosition();
            //dylanRan = 1;
        //}
        /*
        if (lift1CurrentRotation >=4) {
            liftTooHigh = 1;
        }nm,
        else {
            liftTooHigh = 0;
        }

         */

        if (controller2.dpadLeftOnce()) {
            try {
                functions.scoringPosition();
                telemetry.addLine("Scoring Position achieved");
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }

        if (controller2.dpadRightOnce()) {
            try {
                functions.pixelDropAndReset();
            }
            catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }

        if (controller2.AOnce() && intakeOn == 0) {
            try {
                functions.intakeRun();
            }
            catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            intakeOn = 1;
        }
        else if (controller2.AOnce() && intakeOn == 1){
            try {
                functions.intakeStop();
            }
            catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            intakeOn = 0;
            //dylanRan = 0;
        }

        if (controller2.BOnce()) {
            functions.intakePushOut();
        }
        else if (controller2.BOnce()){
            try {
                functions.intakeStop();
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }

        if (controller2.dpadUpOnce()) {
            // grabbing pixels
            functions.grabberMove();
        }

        if (controller1.dpadDownOnce()) {
            // releasing pixels
            functions.grabberMove();
        }

        if (controller2.YOnce()) {
            functions.pivotMove();
            //will run which one is the one available
        }

        if (controller2.XOnce()) {
            functions.stopperUp();
        }

        if (controller1.right_trigger > 0.9 && planeLaunched == 0) {
            functions.launchAirplane();
        }
        if (controller2.right_trigger > 0.9 && planeLaunched == 1) {
            //airplaneLauncher.setPosition(0.9);
            //planeLaunched = 0;
        }

        if (controller2.left_trigger > 0.9) {
            drive.drive(0,0,-functions.backdropParallelAngle() * 2 / Math.PI);
            try {
                sleep(1000);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            drive.drive(0,0,0);
        }
        telemetry.update();
    }
    /*
    private void ScoringPosition() {
        ElapsedTime teleopTimer1 = new ElapsedTime();
        teleopTimer1.reset();
        while (teleopTimer1.seconds() < 0.5) {
            grabber.setPosition(0.67);
        }
        while (0.5 < teleopTimer1.seconds() && teleopTimer1.seconds() < 1.4) {
            lift1.setPower(1);
            lift2.setPower(1);
        }
        while (1.4 < teleopTimer1.seconds() && teleopTimer1.seconds() < 2) {
            pivot1.setPosition(0);
        }
    }

     */
    /*
    private void PixelDropAndReset() {
        ElapsedTime teleopTimer2 = new ElapsedTime();
        teleopTimer2.reset();
        while(teleopTimer2.seconds() < 0.7) {
            grabber.setPosition(1);
            grabbedPixels = 0;
        }

        while(0.7 < teleopTimer2.seconds() && teleopTimer2.seconds() < 0.8) {
            lift1.setPower(-1);
            lift2.setPower(-1);
        }

        while(0.87 < teleopTimer2.seconds() && teleopTimer2.seconds() < 1.5) {
            pivot1.setPosition(1);
            pivotReadyToDrop = 0;
        }
    }
    public void IntakeRun() {
        ElapsedTime teleopTimer3 = new ElapsedTime();
        teleopTimer3.reset();
        while(teleopTimer3.seconds() < 0.5) {
            stopper.setPosition(.9);
        }
        while(1 < teleopTimer3.seconds() && teleopTimer3.seconds() < 1.4) {
            intake.setPower(1);
        }
        intakeOn = 1;
    }
    */

}