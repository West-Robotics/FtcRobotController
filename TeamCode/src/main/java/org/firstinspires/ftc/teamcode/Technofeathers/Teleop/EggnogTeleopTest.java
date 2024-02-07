package org.firstinspires.ftc.teamcode.Technofeathers.Teleop;

import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Controller;
import org.firstinspires.ftc.teamcode.Technofeathers.TechnofeathersDrive;
import org.firstinspires.ftc.teamcode.Technofeathers.TechnofeathersPDTest;

@TeleOp(name = "EggnogTeleopTest")
public class EggnogTeleopTest extends OpMode {
    private TechnofeathersPDTest test = new TechnofeathersPDTest(0.1);
    //smaller kp = slowing down earlier
    //bigger kp = slowing down later
    public TechnofeathersDrive drive;
    public Controller controller1;
    public Servo pivot1;
    public Servo grabber;
    public Servo airplaneLauncher;
    public DcMotor lift1;
    public DcMotor lift2;
    public DcMotor intake;
    public Servo stopper;
    public DistanceSensor distSense1;
    //private int i = 0;
    //private int j = 0;
    //double lift1CurrentRotation = lift1.getCurrentPosition()/537.7;
    public int intakeOn = 0;
    public int liftTooHigh = 0;
    public int planeLaunched = 0;
    public byte dylanRan = 0;
    public byte daveRan = 0;
    public int grabbedPixels = 0;
    public int pivotReadyToDrop = 1;

    public int placeholderE = 1;
    public int placeholderF = 1;

    public int placeholderG = 1;
    public int placeholderH = 1;
    public int placeholderI = 1;


    public ElapsedTime timer = new ElapsedTime();
    private Functions functions = new Functions();

    @Override
    public void init() {
        drive = new TechnofeathersDrive(this, hardwareMap);
        controller1 = new Controller(gamepad1);
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
    }

    @Override
    public void loop() {
        controller1.update();
        drive.drive(controller1.left_stick_x, controller1.left_stick_y/1.25, controller1.right_stick_x/1.25);

        //drive.drive(-controller1.left_stick_x, -controller1.left_stick_y/1.25, -controller1.right_stick_x/1.25);
        if (controller1.AOnce() && intakeOn == 0) {
            stopper.setPosition(.9);
            intake.setPower(1);
            intakeOn = 1;
            //import timer later
        } else if (controller1.AOnce() && intakeOn == 1){
            stopper.setPosition(.37);
            intake.setPower(0);
            intakeOn = 0;
            dylanRan = 0;
            //Once intake stops, Dylan can run; needs extreme testing
            //TODO: Difference between backdrop and border needs to be figured out;
        }

        if (controller1.BOnce() && placeholderG == 1) {
            intake.setPower(-1);
            placeholderG = 2;
        } else if (controller1.BOnce() && placeholderG == 2){
            intake.setPower(0);
            placeholderG = 1;
        }

        //lift
        if (controller1.leftBumper() && liftTooHigh == 0) {
            functions.LiftGoUp();
        } else if (controller1.rightBumper()) {
            functions.LiftGoDown();
        } else {
            functions.StopLift();
        }

        if (dylanRan == 0) {
            functions.Dylan();
        }

        if (controller1.dpadRight()) {
            functions.Dave();
        }

        if (controller1.right_trigger > 0.9 && planeLaunched == 0) {
            airplaneLauncher.setPosition(0.5);
            planeLaunched = 1;
        }

        if(controller1.right_trigger > 0.9 && planeLaunched == 1) {
            airplaneLauncher.setPosition(0.9);
            planeLaunched = 0;
        }

        if(controller1.backOnce()){
            pivot1.setPosition(0.25);
        }

        telemetry.addData("Distance from nearest object: ", distSense1.getDistance(INCH));
    }
}