package org.firstinspires.ftc.teamcode.vampire.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.vampire.hardware.Arm;
import org.firstinspires.ftc.teamcode.vampire.hardware.DuckDuckGo;
import org.firstinspires.ftc.teamcode.vampire.hardware.Intake;
import org.firstinspires.ftc.teamcode.vampire.hardware.VampireDrive;
import org.firstinspires.ftc.teamcode.vampire.hardware.Webcam;

@Autonomous(name="Vampire: BRS", group="Vampire")
public class BRS extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        // Dum auto RIP
        VampireDrive drive;
        Arm arm;
        Intake intake;
        DuckDuckGo spin;
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
        spin = new DuckDuckGo(this, hardwareMap);
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

        if (position == 1) {

            drive.move(0.6, 28, 0);
            drive.move(0.6, 13, -90);

        } else drive.move(0.5, 28, -33);

        arm.setLift(position);
        drive.turn(0.5, -45);
        intake.reverse();
        sleep(3000);
        intake.stop();
        drive.turn(1, 45);
        drive.move(0.5, 40, 111);

        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 4000) { spin.spin(true, false); }
        spin.stop();

        drive.turn(1, 10);
        drive.move(0.6, 22, 20);
        arm.setLift(0);
    }

}
