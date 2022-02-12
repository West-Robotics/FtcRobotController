package org.firstinspires.ftc.teamcode.vampire.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.vampire.hardware.Arm;
import org.firstinspires.ftc.teamcode.vampire.hardware.Intake;
import org.firstinspires.ftc.teamcode.vampire.hardware.VampireDrive;
import org.firstinspires.ftc.teamcode.vampire.hardware.Webcam;

@Autonomous(name="Vampire: BLWOut", group="Vampire")
public class BLWOut extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        // Dum auto RIP
        VampireDrive drive;
        Arm arm;
        Intake intake;
        Webcam webcam;

        // Elapsed time for timed motion
        ElapsedTime runtime = new ElapsedTime();

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        drive = new VampireDrive(this, hardwareMap);
        arm = new Arm(this, hardwareMap);
        intake = new Intake(this, hardwareMap);
        webcam = new Webcam(this, hardwareMap);
        webcam.debug();

        // Get how many rings are stacked
        int position = 3;
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 2) {

            position = webcam.getCargoPos();
            webcam.update();
            telemetry.update();

        }

        if (position == 3) {

            drive.move(0.6, 27, 0);
            drive.move(0.6, 13, 90);

        } else drive.move(0.6, 30, 32);

        arm.setLift(position);
        drive.turn(1, 45);
        intake.reverse();
        sleep(3000);
        intake.stop();
        drive.turn(1, 45);
        drive.move(0.4, 35, 90);
        drive.move(0.5, 40, 175);
        arm.setLift(0);

        drive.move(0.6, 20, 90);

    }

}
