package org.firstinspires.ftc.teamcode.Technofeathers.Gamble.Auto;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Controller;
import org.firstinspires.ftc.teamcode.Technofeathers.Gamble.Teleop.Subsystems.HorizontalLift;
import org.firstinspires.ftc.teamcode.Technofeathers.Gamble.Teleop.Subsystems.VerticalLift;
import org.firstinspires.ftc.teamcode.Technofeathers.TechnofeathersDrive;

import java.util.concurrent.TimeUnit;

@Autonomous (name = "PokerAutoTest")
public class PokerAutoTest extends LinearOpMode{
    public DcMotor frontRight;
    public DcMotor frontLeft;
    public DcMotor backRight;
    public DcMotor backLeft;

    public TechnofeathersDrive drive;
    /*public HorizontalLift horizontalLift;
    public VerticalLift verticalLift;

    public Servo diffyRotatorLeft;
    public Servo diffyRotatorRight;
    public Servo linkageServoLeft;
    public Servo linkageServoRight;

    public Servo horizontalLiftServoLeft;
    public Servo horizontalLiftServoRight;
    public Servo pivotSlide;
    public Servo pivotClaw;

    public Servo grabber;

     */

    IMU imu;
    IMU.Parameters imuParameters;
    SparkFunOTOS myOtos;
    YawPitchRollAngles robotOrientation;
    double Yaw = 0;
    double joystickX = 0;
    double joystickY = 0;
    double joystickTurn = 0;
    @Override
    public void runOpMode() throws InterruptedException{
        drive = new TechnofeathersDrive();
        drive.setupMotors(hardwareMap);
        /*
        horizontalLift.setupMotors(hardwareMap);
        verticalLift.setupMotors(hardwareMap);
        diffyRotatorLeft = hardwareMap.get(Servo.class, "diffyRotatorLeft");
        diffyRotatorRight = hardwareMap.get(Servo.class, "diffyRotatorRight");
        linkageServoLeft = hardwareMap.get(Servo.class, "linkageServoLeft");
        linkageServoRight = hardwareMap.get(Servo.class, "linkageServoRight");
        horizontalLiftServoLeft = hardwareMap.get(Servo.class, "horizontalLiftServoLeft");
        horizontalLiftServoRight = hardwareMap.get(Servo.class, "horizontalLiftServoRight");
        pivotSlide = hardwareMap.get(Servo.class, "diffyRotatorLeft");
        pivotClaw = hardwareMap.get(Servo.class, "diffyRotatorLeft");
        grabber = hardwareMap.get(Servo.class, "diffyRotatorLeft");
         */
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
            SparkFunOTOS.Pose2D pos = myOtos.getPosition();
            while (pos.y <= 24) {
                if (pos.y <= 1) {
                    joystickX = 0;
                    joystickY = 0.3;
                    joystickTurn = 0.2;
                }
                drive.drive(joystickX,joystickY,joystickTurn);
                telemetry.addData("X coordinate", pos.x);
                telemetry.addData("Y coordinate", pos.y);
                telemetry.addData("Heading angle", pos.h);
                joystickX = (-pos.x)/10;
                if (pos.y <= 20) {
                    joystickY = 0.5;
                }
                else if (20 < pos.y && pos.y < 24) {
                    joystickY = (24-pos.y)/8;
                }
                joystickTurn = (-pos.h)/360;
                telemetry.update();
            }
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
        myOtos.setLinearScalar(0.992);
        myOtos.setAngularScalar(1.0);
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
