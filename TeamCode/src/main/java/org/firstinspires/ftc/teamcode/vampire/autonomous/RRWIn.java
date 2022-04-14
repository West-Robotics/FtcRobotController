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

        // Set start pose
        Pose2d startPose = new Pose2d(12, -65, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        // Trajectories
        Trajectory toHub1 = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(2, -38, Math.toRadians(135)))
                .build();
        Trajectory toWall = drive.trajectoryBuilder(toHub1.end())
                .lineToLinearHeading(new Pose2d(13, -65, 0))
                .build();
        Trajectory toHub2 = drive.trajectoryBuilder(new Pose2d(12, -65, 0))
                .lineToLinearHeading(new Pose2d(2, -38, Math.toRadians(125)))
                .build();
        Trajectory back = drive.trajectoryBuilder(new Pose2d(30, -65, 0))
                .lineToLinearHeading(new Pose2d(12, -65, 0))
                .build();
        Trajectory park = drive.trajectoryBuilder(toWall.end())
                .forward(30)
                .build();
        Trajectory park2 = drive.trajectoryBuilder(park.end())
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
        while (opModeIsActive() && runtime.seconds() < Intake.OUTTAKE_TIME) intake.reverse();
        intake.stop();
        arm.setLift(0, 0.5);
        drive.followTrajectory(toWall);

        // Back and forth
        for (int i = 0; i < 2; i++) {

            // Get freight
            intake.intake();
            boolean isContinue = true;
            while (!intake.isFreight()) {

                if (drive.getPoseEstimate().getX() < 45)
                    drive.setWeightedDrivePower(new Pose2d(0.3, 0, 0));
                else
                    drive.setWeightedDrivePower(new Pose2d(0.3, 0.15, 0));

                if (drive.getPoseEstimate().getX() > 55) {

                    i = 2;
                    isContinue = false;

                }

                drive.update();

            }
            intake.stop();

            if (isContinue) {

                // Go to hub
                drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(30, -65, 0))
                        .build());
                drive.followTrajectory(back);
                arm.setLift(3);
                drive.followTrajectory(toHub2);

                // Drop off freight and back to wall
                runtime.reset();
                while (opModeIsActive() && runtime.seconds() < Intake.OUTTAKE_TIME) intake.reverse();
                intake.stop();
                arm.setLift(0, 0.5);
                drive.followTrajectory(toWall);

            }

        }

        // Park
        drive.followTrajectory(park);
        drive.followTrajectory(park2);

    }

}
