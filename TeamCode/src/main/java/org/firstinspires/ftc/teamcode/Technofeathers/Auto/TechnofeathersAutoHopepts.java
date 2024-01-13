package org.firstinspires.ftc.teamcode.Technofeathers.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Controller;
import org.firstinspires.ftc.teamcode.Technofeathers.TechnofeathersDrive;
import org.firstinspires.ftc.teamcode.Technofeathers.TechnofeathersPDTest;

import java.util.concurrent.TimeUnit;

@Autonomous(name="TechnofeathersAutoHopepts", group="Technofeathers")
public class TechnofeathersAutoHopepts extends LinearOpMode {

    private TechnofeathersPDTest test = new TechnofeathersPDTest(0.1);
    private Servo pivot1;
    private Servo grabber;
    private DcMotor lift1;
    private DcMotor lift2;
    private DcMotor intake;
    private Servo stopper;

    public int intake_state = 0;
    public int placeholderB = 1;
    public int placeholderX = 1;
    public int placeholderY = 1;



    @Override public void runOpMode() throws InterruptedException {

        TechnofeathersDrive drive = new TechnofeathersDrive(this, hardwareMap);
        pivot1 = hardwareMap.get(Servo.class,  "pivot1");
        grabber = hardwareMap.get(Servo.class, "grabber");
        lift1 = hardwareMap.get(DcMotor.class,  "lift1");
        lift2 = hardwareMap.get(DcMotor.class, "lift2");
        lift1.setDirection(DcMotorSimple.Direction.FORWARD);
        lift2.setDirection(DcMotorSimple.Direction.REVERSE);
        intake = hardwareMap.get(DcMotor.class, "intake");
        stopper = hardwareMap.get(Servo.class, "stopper");

        telemetry.addLine("Variables Instantiated");
        telemetry.update();

        waitForStart();
        ElapsedTime e = new ElapsedTime();
        e.reset();
        e.startTime();

       while (opModeIsActive() && !isStopRequested()) {

           lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset the motor encoder
           lift1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // Turn the motor back on when we are done
           //um is this really necessary
           lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset the motor encoder
           lift2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // Turn the motor back on when we are done

           telemetry.addLine("boop");
           telemetry.update();

           double lift1CurrentRotation = lift1.getCurrentPosition()/537.7;
           double lift2CurrentRotation = lift2.getCurrentPosition()/537.7;
           if (lift1CurrentRotation > 3) {
                lift1.setPower(0);
                lift2.setPower(0);
           }
           telemetry.addLine("Beginning to Utilize Encoders");
           telemetry.update();

           if (e.time(TimeUnit.SECONDS) < 1) {
               drive.drive(0,1,0.17);
           }
           drive.drive(0,0,0);
           telemetry.addLine("Drive ran and now it is stopped.");
           telemetry.update();
            /*
           if (e.time(TimeUnit.SECONDS) < 1) {
               intake.setPower(1);
           }

             */
           /*
           if (e.time(TimeUnit.SECONDS) < 2 && e.time(TimeUnit.SECONDS) > 1) {
               pivot1.setPosition(180);
               test.setDesiredPoint(0.4);
               test.update(pivot1.getPosition());
           }
            */

           if (e.time(TimeUnit.SECONDS) < 3 && e.time(TimeUnit.SECONDS) > 2) {
               lift1.setPower(0.5);
               lift2.setPower(0.5);
           }

           lift1.setPower(0);
           lift2.setPower(0);

           pivot1.setPosition(0);

           grabber.setPosition(1);

           e.reset();
        }
    }
}
