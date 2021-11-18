package org.firstinspires.ftc.teamcode.vampire.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.vampire.hardware.Arm;
import org.firstinspires.ftc.teamcode.vampire.hardware.DuckDuckGo;
import org.firstinspires.ftc.teamcode.vampire.hardware.Intake;
import org.firstinspires.ftc.teamcode.vampire.roadrunner.drive.VampireRRDrive;

@Autonomous(name="Vampire: RLDWhWa", group="Vampire")
public class RLDWhWa extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        // Subsystems
        VampireRRDrive drive = new VampireRRDrive(hardwareMap);
        Arm arm = new Arm(this, hardwareMap);
        Intake intake = new Intake(this, hardwareMap);
        DuckDuckGo spin = new DuckDuckGo(this, hardwareMap);

        // Set starting position
        Pose2d startPose = new Pose2d(-27, -63.75, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        // Create trajectories
        Trajectory deploy = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(-23.75, -40), Math.toRadians(55))
                .build();
        Trajectory wheel = drive.trajectoryBuilder(deploy.end(), true)
                .lineToLinearHeading(new Pose2d(-60, -60, 0))
                .build();
        Trajectory warehouse = drive.trajectoryBuilder(wheel.end())
                .splineToConstantHeading(new Vector2d(40, -40), 0)
                .build();

        waitForStart();
        if(isStopRequested()) return;

        // Deploy cargo
        drive.followTrajectory(deploy);
        arm.setLift(3);
        sleep(1000);
        intake.reverse();
        sleep(2000);
        intake.stop();

        // Spin wheel
        drive.followTrajectory(wheel);
        spin.spinRed();
        sleep(3000);
        spin.stop();

        // Go to warehouse
        drive.followTrajectory(warehouse);

    }

}