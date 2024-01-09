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
        //may be deleted


        drive = new TechnofeathersDrive(this, hardwareMap);
        pivot1 = hardwareMap.get(Servo.class,  "pivot1");
        grabber = hardwareMap.get(Servo.class, "grabber");
        lift1 = hardwareMap.get(DcMotor.class,  "lift1");
        lift2 = hardwareMap.get(DcMotor.class, "lift2");
        lift1.setDirection(DcMotorSimple.Direction.FORWARD);
        lift2.setDirection(DcMotorSimple.Direction.REVERSE);
        intake = hardwareMap.get(DcMotor.class, "intake");
        stopper = hardwareMap.get(Servo.class, "stopper");

        telemetry.addLine("ur dad");
        telemetry.update();

        waitForStart();
        ElapsedTime e = new ElapsedTime();
        e.reset();
        e.startTime();

       telemetry.addLine("ur mom");
       telemetry.update();

        telemetry.addLine("bleep");
        telemetry.update();

       while (opModeIsActive() && !isStopRequested()) {
           telemetry.addLine("boop");
           telemetry.update();
           if (e.time(TimeUnit.SECONDS) < 1) {
               drive.drive(0,1,0.17);
           }
           drive.drive(0,0,0);
           telemetry.addLine("ukfukyg.hlhnkjmgcjdytfuvhjfyr6yfhvnbjhkijojlkh");
           telemetry.update();

           telemetry.addLine("aiorjgioaerjgfiosjfw");
           telemetry.update();
           // these whiles should be ifs otherwise they will keep runnig by themselves amd be stuck in there
           // except that is not your issue
           // a logic thing for later
           if (e.time(TimeUnit.SECONDS) < 1) {
               intake.setPower(1);
           }

           if (e.time(TimeUnit.SECONDS) < 2 && e.time(TimeUnit.SECONDS) > 1) {
               pivot1.setPosition(180);
               /*
               test.setDesiredPoint(0.4);
               test.update(pivot1.getPosition());
               */
            }

           while (e.time(TimeUnit.SECONDS) < 3 && e.time(TimeUnit.SECONDS) > 2) {
               lift1.setPower(-1);
               lift2.setPower(1);
           }
           lift1.setPower(0.5);
           lift2.setPower(0.5);

            /*while(e.time(TimeUnit.SECONDS)<3.6) {
                drive.drive(0,0.3,0);
            }

             */
            while (e.time(TimeUnit.SECONDS) < 5) {
                drive.drive(0, 1, -1);
            }



            /*
            while(e.time(TimeUnit.SECONDS)<1) {
                drive.drive(0,0.3,0);
            }

             */

            e.reset();
            //drive.move(0.4, 60, 0);
        }
    }
}
