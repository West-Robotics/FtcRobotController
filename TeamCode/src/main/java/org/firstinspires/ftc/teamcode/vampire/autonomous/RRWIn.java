package org.firstinspires.ftc.teamcode.vampire.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.roadrunner.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.vampire.hardware.*;

@Autonomous(name="Vampire: RRWIn", group="Vampire")
public class RRWIn extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        // Set up subsystems
        MecanumDrive drive = new MecanumDrive(hardwareMap);
        Arm arm = new Arm(this, hardwareMap);
        Intake intake = new Intake(this, hardwareMap);
        Webcam webcam = new Webcam(this, hardwareMap);
        new TapeArm(this, hardwareMap);

        // Set timer
        ElapsedTime runtime = new ElapsedTime();
        double FORWARD_TIME = 1;

        // Set start pose
        Pose2d startPose = new Pose2d(12, -65, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        // Trajectories
        Trajectory toHub1 = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(2, -38, Math.toRadians(135)))
                .build();
        Trajectory toWall1 = drive.trajectoryBuilder(toHub1.end())
                .lineToLinearHeading(new Pose2d(12, -65, 0))
                .build();
        Trajectory toWall2 = drive.trajectoryBuilder(toWall1.end())
                .forward(25)
                .build();
        Trajectory toHub2 = drive.trajectoryBuilder(new Pose2d(12, -65, 0))
                .lineToLinearHeading(new Pose2d(2, -38, Math.toRadians(125)))
                .build();
        Trajectory park1 = drive.trajectoryBuilder(toWall2.end())
                .forward(10)
                .build();
        Trajectory park2 = drive.trajectoryBuilder(park1.end())
                .strafeLeft(20)
                .build();

        // Send telemetry message to signify robot waiting
        telemetry.addData("Status", "Ready to run");
        telemetry.update();

        // Wait for program to start
        waitForStart();
        if (isStopRequested()) return;

        // Get how many rings are stacked
        int position = 3;
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 1) {

            position = webcam.getCargoPos();
            webcam.update();
            telemetry.update();

        }

        // Drop off first freight
        arm.setLift(position);
        drive.followTrajectory(toHub1);
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < DuckDuckGo.AUTO_TIME) intake.reverse();
        intake.stop();
        arm.setLift(0, 0.5);
        drive.followTrajectory(toWall1);
        drive.followTrajectory(toWall2);

        // Back and forth
        for (int i = 0; i < 1; i++) {

            // Get freight
            intake.intake();
            //while (!intake.isFreight()) {

                runtime.reset();
                while (opModeIsActive() && runtime.seconds() < FORWARD_TIME) {

                    drive.setWeightedDrivePower(new Pose2d(0.2, 0, 0));
                    drive.update();

                }
                drive.turn(Math.toRadians(-30));
                drive.turn(Math.toRadians(60));
                drive.turn(Math.toRadians(-30));

            //}
            intake.stop();
            if (intake.isFreight()) {

                // Go to hub
                drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate())
                        .lineTo(new Vector2d(12, -65))
                        .build());
                arm.setLift(3);
                drive.followTrajectory(toHub2);

                // Drop off freight and back to wall
                runtime.reset();
                while (opModeIsActive() && runtime.seconds() < DuckDuckGo.AUTO_TIME) intake.reverse();
                intake.stop();
                arm.setLift(0, 0.5);
                drive.followTrajectory(toWall1);
                drive.followTrajectory(toWall2);
            }

        }

        // Park
        drive.followTrajectory(park1);
        drive.followTrajectory(park2);

    }

}
