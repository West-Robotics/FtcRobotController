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

        MecanumDrive driveRR = new MecanumDrive(hardwareMap);
        VampireDrive driveV = new VampireDrive(this, hardwareMap);

        Pose2d startPose = new Pose2d(-36, -65, Math.toRadians(90));
        driveRR.setPoseEstimate(startPose);

        // Trajectories
        Trajectory toCarousel = driveRR.trajectoryBuilder(startPose)
                .splineToLinearHeading(new Pose2d(-50, -59, 0), 0)
                .build();

        waitForStart();
        if (isStopRequested()) return;

        driveRR.followTrajectory(toCarousel);

        /*
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
        //drive.debug();
        arm.debug();

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

        } else drive.move(0.6, 29, 30);

        arm.setLift(position);
        drive.turn(1, 45);
        intake.reverse();
        sleep(3000);
        intake.stop();
        drive.turn(1, 45);
        drive.move(0.5, 30, 95);
        drive.move(0.4, 34,-178);

        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 4000) { spin.spin(true, false); }
        spin.stop();

        drive.move(0.6, 28, -93);
        arm.setLift(0);
*/
    }

}
