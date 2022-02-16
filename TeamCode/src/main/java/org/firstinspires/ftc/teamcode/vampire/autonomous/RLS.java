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

@Autonomous(name="Vampire: RLS", group="Vampire")
public class RLS extends LinearOpMode {

	@Override
	public void runOpMode() throws InterruptedException {

		// Set up subsystems
		MecanumDrive drive = new MecanumDrive(hardwareMap);
		Arm arm = new Arm(this, hardwareMap);
		Intake intake = new Intake(this, hardwareMap);
		DuckDuckGo spin = new DuckDuckGo(this, hardwareMap);
		Webcam webcam = new Webcam(this, hardwareMap);

		// Set timer
		ElapsedTime runtime = new ElapsedTime();

		// Enable debug mode
		webcam.debug();
		arm.debug();

		// Set start pose
		Pose2d startPose = new Pose2d(-36, -65, Math.toRadians(90));
		drive.setPoseEstimate(startPose);

		// Trajectories
		Trajectory toCarousel = drive.trajectoryBuilder(startPose)
			.splineToLinearHeading(new Pose2d(-55, -59, 0), 0)
			.build();
		Trajectory toHub = drive.trajectoryBuilder(toCarousel.end())
			.lineToLinearHeading(new Pose2d(-26, -38, Math.toRadians(45)))
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

	}

}
