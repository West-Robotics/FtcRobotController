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
import org.firstinspires.ftc.teamcode.vampire.hardware.TapeArm;
import org.firstinspires.ftc.teamcode.vampire.hardware.VampireDrive;
import org.firstinspires.ftc.teamcode.vampire.hardware.Webcam;

@Autonomous(name="Vampire: BRWOut", group="Vampire")
public class BRWOut extends LinearOpMode {

	@Override
	public void runOpMode() throws InterruptedException {

		// Set up subsystems
		MecanumDrive drive = new MecanumDrive(hardwareMap);
		Arm arm = new Arm(this, hardwareMap);
		Intake intake = new Intake(this, hardwareMap);
		DuckDuckGo spin = new DuckDuckGo(this, hardwareMap);
		Webcam webcam = new Webcam(this, hardwareMap);
		new TapeArm(this, hardwareMap);

		// Set timer
		ElapsedTime runtime = new ElapsedTime();

		// Enable debug mode
		webcam.debug();
		arm.debug();

		// Set start pose
		Pose2d startPose = new Pose2d(-36, 65, Math.toRadians(-90));
		drive.setPoseEstimate(startPose);

		// Trajectories
		Trajectory toCarousel = drive.trajectoryBuilder(startPose)
				.strafeTo(new Vector2d(-54, 58))
				.build();
		Trajectory toHub1 = drive.trajectoryBuilder(toCarousel.end())
				.lineToLinearHeading(new Pose2d(-54, 30, 0))
				.build();
		Trajectory toHub2 = drive.trajectoryBuilder(toHub1.end())
				.lineToLinearHeading(new Pose2d(-29, 24, Math.toRadians(0)))
				.build();
		Trajectory backOut = drive.trajectoryBuilder(toHub2.end())
				.lineToLinearHeading(new Pose2d(-44, 30, Math.toRadians(100)))
				.build();
		Trajectory park1 = drive.trajectoryBuilder(toHub2.end())
				.strafeTo(new Vector2d(-10, 72))
				.build();
		Trajectory park2 = drive.trajectoryBuilder(park1.end())
				.forward(60)
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

		// Move to carousel
		drive.followTrajectory(toCarousel);
		runtime.reset();
		while (opModeIsActive() && runtime.seconds() < 1.5) {

			drive.setWeightedDrivePower(new Pose2d(0, -0.1, 0));
			drive.update();

		}
		drive.setWeightedDrivePower(new Pose2d(0, 0, 0));
		runtime.reset();
		while (opModeIsActive() && runtime.seconds() < 2) spin.spin(false, true);
		spin.stop();

		// Drop off first freight
		arm.setLift(position);
		drive.followTrajectory(toHub1);
		drive.followTrajectory(toHub2);
		runtime.reset();
		while (opModeIsActive() && runtime.seconds() < DuckDuckGo.AUTO_TIME) intake.reverse();
		intake.stop();

		// Grab duck
		arm.setLift(0, 0.5);
		drive.followTrajectory(backOut);
		runtime.reset();
		double duckX = 0;
		double duckY = 0;
		webcam.toggleMode(true);
		sleep(1000);
		while (opModeIsActive() && runtime.seconds() < 2) {

			duckX = drive.getPoseEstimate().getX() + (webcam.getDuckDistance() + 1) * Math.sin(Math.PI / 2 - drive.getPoseEstimate().getHeading() + webcam.getDuckPose()[2]);
			duckY = drive.getPoseEstimate().getY() + (webcam.getDuckDistance() + 1) * Math.cos(Math.PI / 2 - drive.getPoseEstimate().getHeading() + webcam.getDuckPose()[2]);

		}
		intake.intake();
		drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate())
				.strafeTo(new Vector2d(duckX, duckY))
				.build());
		sleep(500);
		intake.stop();

		// Drop off duck
		arm.setLift(3, 0.5);
		drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate())
				.lineToLinearHeading(new Pose2d(-28, 28, Math.toRadians(0)))
				.build());
		runtime.reset();
		while (opModeIsActive() && runtime.seconds() < DuckDuckGo.AUTO_TIME) intake.reverse();
		intake.stop();

		// Park
		arm.setLift(0, 1);
		drive.followTrajectory(park1);
		drive.followTrajectory(park2);

	}

}
