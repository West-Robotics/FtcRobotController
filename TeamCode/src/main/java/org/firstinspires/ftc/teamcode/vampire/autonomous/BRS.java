package org.firstinspires.ftc.teamcode.vampire.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.vampire.hardware.Arm;
import org.firstinspires.ftc.teamcode.vampire.hardware.DuckDuckGo;
import org.firstinspires.ftc.teamcode.vampire.hardware.Intake;
import org.firstinspires.ftc.teamcode.vampire.hardware.TapeArm;
import org.firstinspires.ftc.teamcode.vampire.hardware.VampireDrive;
import org.firstinspires.ftc.teamcode.vampire.hardware.Webcam;

@Autonomous(name="Vampire: BRS", group="Vampire")
public class BRS extends LinearOpMode {

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
				.strafeTo(new Vector2d(-54, 59))
				.build();
		Trajectory toHub1 = drive.trajectoryBuilder(toCarousel.end())
				.lineToLinearHeading(new Pose2d(-42, 50, 0))
				.build();
		Trajectory toHub2 = drive.trajectoryBuilder(toHub1.end())
				.lineToLinearHeading(new Pose2d(-28, 25, Math.toRadians(0)))
				.build();
		Trajectory backOut = drive.trajectoryBuilder(toHub2.end())
				.lineToLinearHeading(new Pose2d(-36, 36, Math.toRadians(100)))
				.build();
		Trajectory park1 = drive.trajectoryBuilder(toHub2.end())
				.strafeLeft(30)
				.build();
		Trajectory park2 = drive.trajectoryBuilder(park1.end())
				.lineToLinearHeading(new Pose2d(-62, 38, 0))
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

		// Move to carousel
		drive.followTrajectory(toCarousel);
		runtime.reset();
		while (opModeIsActive() && runtime.seconds() < 1.3) {

			drive.setWeightedDrivePower(new Pose2d(0, -0.1, 0));
			drive.update();

		}
		drive.setWeightedDrivePower(new Pose2d(0, 0, 0));
		runtime.reset();
		while (opModeIsActive() && !spin.isStopPosition()) spin.spin(false, true);
		spin.stop();

		// Drop off first freight
		arm.setLift(position);
		drive.followTrajectory(toHub1);
		drive.followTrajectory(toHub2);
		runtime.reset();
		while (opModeIsActive() && runtime.seconds() < Intake.OUTTAKE_TIME) intake.reverse();
		intake.stop();

		// Calculate duck position
		arm.setLift(0, 0.5);
		drive.followTrajectory(backOut);
		runtime.reset();
		double duckX = 0;
		double duckY = 0;
		webcam.toggleMode(true);
		sleep(1000);
		while (opModeIsActive() && runtime.seconds() < 1.5) {

			duckX = webcam.getDuckDistance() == 0 ? 0 : drive.getPoseEstimate().getX() + (webcam.getDuckDistance() + 4) * Math.sin(Math.PI / 2 - drive.getPoseEstimate().getHeading() + webcam.getDuckPose()[2]);
			duckY = webcam.getDuckDistance() == 0 ? 0 : drive.getPoseEstimate().getY() + (webcam.getDuckDistance() + 4) * Math.cos(Math.PI / 2 - drive.getPoseEstimate().getHeading() + webcam.getDuckPose()[2]);

		}
		double MAX_Y = 63;
		if (Math.abs(duckY) > MAX_Y) {

			// Set maximum Y position and scale it down
			double ratio = Math.abs(duckY) / MAX_Y;
			duckY = Math.copySign(MAX_Y, duckY);
			duckX /= ratio;

		}

		if (!(duckX == 0 && duckY == 0)) {

			// Grab duck only if duck exists
			intake.intakeSlow();
			intake.freightStop(6);
			drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate())
					.strafeTo(new Vector2d(duckX - 2, duckY))
					.build());
			drive.turn(Math.toRadians(-20));
			drive.turn(Math.toRadians(40));
			intake.stop();

			// Drop off duck
			arm.setLift(3, 0.5);
			drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate().plus(new Pose2d(0, 0, Math.toRadians(20))))
					.lineToLinearHeading(
							new Pose2d(-27, 30, Math.toRadians(0)),
							MecanumDrive.getVelocityConstraint(DriveConstants.MID_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
							MecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
					.build());
			runtime.reset();
			while (opModeIsActive() && runtime.seconds() < Intake.OUTTAKE_TIME) intake.reverse();
			intake.stop();

		}

		// Park
		arm.setLift(0, 1.5);
		drive.followTrajectory(park1);
		drive.followTrajectory(park2);

	}

}
