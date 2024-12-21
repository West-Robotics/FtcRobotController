package org.firstinspires.ftc.teamcode.Technofeathers.Gamble.Auto;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Technofeathers.TechnofeathersDrive;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.TimeUnit;


@Autonomous(name = "autoing")
public class FirstAuto extends LinearOpMode{

    public TechnofeathersDrive drive;

    IMU imu;
    IMU.Parameters imuParameters;
    SparkFunOTOS myOtos;
    YawPitchRollAngles robotOrientation;
    double Yaw = 0;
    @Override
    public void runOpMode() throws InterruptedException{
        drive = new TechnofeathersDrive();
        drive.setupMotors(hardwareMap);
        imu = hardwareMap.get(IMU.class, "imu");
        imuParameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );
        imu.initialize(imuParameters);
        myOtos = hardwareMap.get(SparkFunOTOS.class, "sensor_otos");
        configureOtos();
        telemetry.addData("Current IMU Angle", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        telemetry.addData("Current Otos Angle", myOtos.getPosition().h);
        ElapsedTime e = new ElapsedTime();
        e.reset();
        e.startTime();
        waitForStart();


        while (opModeIsActive()) {
            robotOrientation = imu.getRobotYawPitchRollAngles();
            Yaw = robotOrientation.getYaw(AngleUnit.DEGREES);
            telemetry.addData("Current Angle", Yaw);
            /*while (Yaw < 10) {
                System.out.println("hdgreg");
                //drive.drive(0,0,1);
            }
             */
            // Get the latest position
            while(e.time(TimeUnit.SECONDS) < 3.6) {
                drive.drive(0,0.3,0);

            }
            drive.drive(0,0,0.1);
            /*
            if (Yaw < 360) {
                drive.drive(0,0,1);
            }
            if (Yaw >= 360) {
                drive.drive(0,0,0);
            }

             */
            SparkFunOTOS.Pose2D pos = myOtos.getPosition();

            // Reset the tracking if the user requests it
            // Re-calibrate the IMU if the user requests it

            telemetry.addData("X coordinate", pos.x);
            telemetry.addData("Y coordinate", pos.y);
            telemetry.addData("Heading angle", pos.h);

            telemetry.update();
        }
    }
    private void configureOtos() {
        telemetry.addLine("Configuring OTOS...");
        telemetry.update();

        myOtos.setLinearUnit(DistanceUnit.INCH);
        myOtos.setAngularUnit(AngleUnit.DEGREES);

        //set in inches and degrees
        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0, 0, 0);
        myOtos.setOffset(offset);

        // Here we can set the linear and angular scalars, which can compensate for
        // scaling issues with the sensor measurements. Note that as of firmware
        // version 1.0, these values will be lost after a power cycle, so you will
        // need to set them each time you power up the sensor. They can be any value
        // from 0.872 to 1.127 in increments of 0.001 (0.1%). It is recommended to
        // first set both scalars to 1.0, then calibrate the angular scalar, then
        // the linear scalar. To calibrate the angular scalar, spin the robot by
        // multiple rotations (eg. 10) to get a precise error, then set the scalar
        // to the inverse of the error. Remember that the angle wraps from -180 to
        // 180 degrees, so for example, if after 10 rotations counterclockwise
        // (positive rotation), the sensor reports -15 degrees, the required scalar
        // would be 3600/3585 = 1.004. To calibrate the linear scalar, move the
        // robot a known distance and measure the error; do this multiple times at
        // multiple speeds to get an average, then set the linear scalar to the
        // inverse of the error. For example, if you move the robot 100 inches and
        // the sensor reports 103 inches, set the linear scalar to 100/103 = 0.971
        myOtos.setLinearScalar(1.0);
        myOtos.setAngularScalar(0.992);
        //calibrate before starting, takes up 617 milliseconds
        myOtos.calibrateImu();

        myOtos.resetTracking();

        // After resetting the tracking, the OTOS will report that the robot is at
        // the origin. If your robot does not start at the origin, or you have
        // another source of location information (eg. vision odometry), you can set
        // the OTOS location to match and it will continue to track from there.
        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(0, 0, 0);
        myOtos.setPosition(currentPosition);


        telemetry.update();
    }
}
