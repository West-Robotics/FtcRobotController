package org.firstinspires.ftc.teamcode.vampire.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.roadrunner.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.vampire.hardware.Arm;
import org.firstinspires.ftc.teamcode.vampire.hardware.DuckDuckGo;
import org.firstinspires.ftc.teamcode.vampire.hardware.Intake;
import org.firstinspires.ftc.teamcode.vampire.hardware.VampireDrive;
import org.firstinspires.ftc.teamcode.vampire.hardware.Webcam;

@Autonomous(name="Vampire: RRWIn", group="Vampire")
public class RRWIn extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        // Set up subsystems
        MecanumDrive drive = new MecanumDrive(hardwareMap);
        Arm arm = new Arm(this, hardwareMap);
        Intake intake = new Intake(this, hardwareMap);
        Webcam webcam = new Webcam(this, hardwareMap);

        // Set timer
        ElapsedTime runtime = new ElapsedTime();

        // Set start pose
        Pose2d startPose = new Pose2d(12, -65, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        // Trajectories
        Trajectory toHub1 = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(2, -38, Math.toRadians(135)))
                .build();
        Trajectory toWall = drive.trajectoryBuilder(new Pose2d(2, -38, Math.toRadians(135)))
                .lineToLinearHeading(new Pose2d(12, -65, 0))
                .build();
        Trajectory toHub2 = drive.trajectoryBuilder(new Pose2d(12, -65, 0))
                .lineToLinearHeading(new Pose2d(2, -38, Math.toRadians(135)))
                .build();
        Trajectory park = drive.trajectoryBuilder(new Pose2d(12, -65, 0))
                .forward(35)
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
        while (opModeIsActive() && runtime.seconds() < 2) {

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
        arm.setLift(0);
        drive.followTrajectory(toWall);

        // Back and forth
        for (int i = 0; i < 3; i++) {

            // Get freight
            intake.intake();
            while (opModeIsActive() && !intake.isFreight())
                drive.setWeightedDrivePower(new Pose2d(0, 0.5, 0));
            intake.stop();

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
            arm.setLift(0);
            drive.followTrajectory(toWall);

        }

        // Park
        drive.followTrajectory(park);

    }

}
