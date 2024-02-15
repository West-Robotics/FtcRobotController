package org.firstinspires.ftc.teamcode.Technofeathers.Teleop;

import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH;

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

    public Functions functions = new Functions();


    public ElapsedTime timer = new ElapsedTime();

    @Override
    public void init() {
        drive = new TechnofeathersTestDrive();
        controller1 = new Controller(gamepad1);
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
        drive.drive(controller1.left_stick_x, -controller1.left_stick_y, controller1.right_stick_x);
        if (functions.liftMaxLimitReached()) {
            functions.liftStop();
        }
        if (functions.liftMinLimitReached()) {
            functions.liftStop();
        }
       // if (distSense1.getDistance(INCH) <= 10 && 0 < controller1.left_stick_y) {
          //  drive.drive(controller1.left_stick_x/2, controller1.left_stick_y/2, controller1.right_stick_x/2);
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

        if(controller1.dpadLeftOnce()) {
            try {
                functions.scoringPosition();
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }

        if(controller1.dpadRightOnce()) {
            try {
                functions.pixelDropAndReset();
            }
            catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }

        if (controller1.AOnce() && intakeOn == 0) {
            try {
                functions.intakeRun();
            }
            catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            intakeOn = 1;
        }
        else if (controller1.AOnce() && intakeOn == 1){
            try {
                functions.intakeStop();
            }
            catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            intakeOn = 0;
            //dylanRan = 0;
        }

        if (controller1.BOnce()) {
            functions.intakePushOut();
        }
        else if (controller1.BOnce()){
            try {
                functions.intakeStop();
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }

        if (controller1.dpadUpOnce()) {
            // grabbing pixels
            functions.grabberMove();
        }

        if (controller1.dpadDownOnce()) {
            // releasing pixels
            functions.grabberMove();
        }

        if (controller1.YOnce()) {
            functions.pivotMove();
            //will run which one is the one available
        }

        if (controller1.XOnce()) {
            functions.stopperUp();
        }

        //lift
        if (controller1.leftBumper()/* && liftTooHigh == 0*/) {
            functions.liftGoUp();
        }
        else if (controller1.rightBumper()) {
            functions.liftGoDown();
        }
        else {
            functions.liftStop();
        }

        if (controller1.right_trigger > 0.9 && planeLaunched == 0) {
            functions.launchAirplane();
        }
        if(controller1.right_trigger > 0.9 && planeLaunched == 1) {
            //airplaneLauncher.setPosition(0.9);
            //planeLaunched = 0;
        }
        if(controller1.backOnce()){
            //pivot1.setPosition(0.25);
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