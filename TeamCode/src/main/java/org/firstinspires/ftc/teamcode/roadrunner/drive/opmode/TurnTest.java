package org.firstinspires.ftc.teamcode.roadrunner.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.drive.MecanumDrive;

/*
 * This is a simple routine to test turning capabilities.
 */
@Config
@Autonomous(group = "drive")
public class TurnTest extends LinearOpMode {
    public static double ANGLE = 270; // deg

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        //drive.turn(Math.toRadians(ANGLE));
        drive.turn(Math.toRadians(90));
        sleep(1000);
        drive.turn(Math.toRadians(180));
        sleep(1000);
        drive.turn(Math.toRadians(270));
        sleep(1000);
        drive.turn(Math.toRadians(360));

    }
}
