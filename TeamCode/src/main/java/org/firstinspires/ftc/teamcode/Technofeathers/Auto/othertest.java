package org.firstinspires.ftc.teamcode.Technofeathers.Auto;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Autonomous
public class othertest extends LinearOpMode{

    private DistanceSensor distanceSensor;
    private DistanceSensor rightDistanceSensor;
    private DistanceSensor leftDistanceSensor;
    double dist;
    Drivestart drivestart = new Drivestart();

    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;

    DcMotor lift1;
    DcMotor lift2;
    DcMotor intake;

    Servo stopper;
    Servo pivot1;
    Servo grabber;


    int leftPos;
    int rightPos;

    double integralSum = 0;

    double integralSum2 = 0;

    double lastError2 = 0;

    private double lastError = 0;

    ElapsedTime timer = new ElapsedTime();

    double Kp = PIDconstants.Kp;
    double Ki = PIDconstants.Ki;
    double Kd = PIDconstants.Kd;

    double Yaw;
    IMU imu;
    double currentAngle;
    int intakeTicks;

    IMU.Parameters imuParameters;

    double state;
    double targetAngle;
    YawPitchRollAngles robotOrientation;

    public void runOpMode() throws InterruptedException{

        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        distanceSensor = hardwareMap.get(DistanceSensor.class,"distSense1");
        leftDistanceSensor = hardwareMap.get(DistanceSensor.class, "distLeft");
        rightDistanceSensor = hardwareMap.get(DistanceSensor.class, "distRight");
        imu = hardwareMap.get(IMU.class, "imu");
        lift1 = hardwareMap.get(DcMotor.class,"lift1");
        lift2 = hardwareMap.get(DcMotor.class,"lift2");
        intake = hardwareMap.get(DcMotor.class, "intake");
        stopper = hardwareMap.get(Servo.class,"stopper");
        pivot1 = hardwareMap.get(Servo.class, "pivot1");
        grabber = hardwareMap.get(Servo.class, "grabber");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift1.setDirection(DcMotorSimple.Direction.FORWARD);
        lift2.setDirection(DcMotorSimple.Direction.REVERSE);

        drivestart.init(frontLeft,frontRight,backLeft,backRight);
        imuParameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );
        imu.initialize(imuParameters);

        waitForStart();


        //runWithEncoder(1,1,0.5,538);

        do {
            robotOrientation = imu.getRobotYawPitchRollAngles();
            Yaw = robotOrientation.getYaw(AngleUnit.DEGREES);
            state = Math.toRadians(Yaw);
            double distance = distanceSensor.getDistance(DistanceUnit.CM);
            double distanceRight = rightDistanceSensor.getDistance(DistanceUnit.CM);
            double distanceLeft = leftDistanceSensor.getDistance(DistanceUnit.CM);
            telemetry.addData("Current Angle", Yaw);
            telemetry.addData("Distance:", distance);
            telemetry.addData("Distance Right", distanceRight);
            telemetry.addData("Distance Left", distanceLeft);
            telemetry.update();

        }while (opModeIsActive() );

    }

    public void pixelate(){
        grabber.setPosition(0.5);
        pivot1.setPosition(0);
        grabber.setPosition(1);
    }

    public void changeAngle(double targetAng, int OneToGreaterTwoToLess, int negLeftPosRight){

        if (OneToGreaterTwoToLess ==1){
            do {
                state = imu.getRobotOrientation(AxesReference.INTRINSIC,AxesOrder.ZYX,AngleUnit.RADIANS).firstAngle;
                double strength = PIDControl(targetAng, state);
                if (negLeftPosRight == -1){
                    lefting(strength);
                } else if (negLeftPosRight == 1){
                    righting(strength);
                }
            } while (opModeIsActive() && targetAng > state);
        } else if(OneToGreaterTwoToLess ==2){
            do {
                state = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;
                double strength = PIDControl(targetAng, state);
                if (negLeftPosRight == -1){
                    lefting(strength);
                } else if(negLeftPosRight == 1){
                    righting(strength);
                }
            } while (opModeIsActive() && targetAng < state);
        }
    }

    public void goToDistance(double targetDist,int OneToGreaterTwoToLess){
        if (OneToGreaterTwoToLess ==1) {
            do {
                dist = distanceSensor.getDistance(DistanceUnit.CM);
                double strength = PIDControlForStraight(targetDist,dist);
                power(strength);
            } while (opModeIsActive() && dist > targetDist);
        } else if (OneToGreaterTwoToLess ==2) {
            do {
                dist = distanceSensor.getDistance(DistanceUnit.CM);
                double strength = PIDControlForStraight(targetDist,dist);
                power(strength);
            } while(opModeIsActive() && dist < targetDist);
        }
    }

    public void releaseIntake(){
        intakeTicks+=1680;
        intake.setTargetPosition(intakeTicks);
        intake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intake.setPower(0.5);
        while (opModeIsActive() && intake.isBusy()){
            idle();
        }

    }

    public double PIDControl(double reference, double state){
        double error = angleReset(reference-state);
        telemetry.addData("Error",error);
        double angle = Math.toDegrees(error);
        telemetry.addData("Error in Degrees", angle);
        integralSum += error * timer.seconds();
        double derivative = (error - lastError) / (timer.seconds());
        lastError = error;
        timer.reset();
        double returning = (error * Kp) + (derivative * Kd) + (integralSum * Ki);
        return returning;
    }

    public double PIDControlForStraight(double reference, double state){
        double error2 = reference - state;
        integralSum2 += error2 * timer.seconds();
        double derivative = (error2 - lastError2) / (timer.seconds());
        lastError2 = error2;
        timer.reset();
        telemetry.addData("Error2", error2);
        double returning2 = (error2 * Kp) + (derivative * Kd) + (integralSum2 * Ki);
        return  returning2;
    }

    public double angleReset(double radians){
        while (radians > Math.PI){
            radians -= 2 * Math.PI;
        }
        while (radians < -Math.PI){
            radians += 2 * Math.PI;
        }
        return radians;
    }

    public void moveUntil(DcMotor motor, double distance, double power){

        while (opModeIsActive()){
            dist = distanceSensor.getDistance(DistanceUnit.CM);
            if (dist>distance){

                motor.setPower(power);
            } else {
                motor.setPower(0);
            }
            telemetry.addData("distance",dist);
            telemetry.update();
        }

    }

    public void runWithEncoder(double rightTurns, double leftTurns, double speed, int ticks){



        double LeftTarget = ticks * leftTurns;
        double rightTarget = ticks * rightTurns;

        leftPos+=LeftTarget;
        rightPos+=rightTarget;

        frontLeft.setTargetPosition(leftPos);
        backLeft.setTargetPosition(leftPos);
        frontRight.setTargetPosition(rightPos);
        backRight.setTargetPosition(rightPos);

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        power(speed);

        while (opModeIsActive() && (frontLeft.isBusy() || frontRight.isBusy() || backLeft.isBusy() || backRight.isBusy()) ){
            idle();
        }

        power(0);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    public void power(double strength){
        frontLeft.setPower(strength);
        backLeft.setPower(strength);
        frontRight.setPower(strength);
        backRight.setPower(strength);
    }
    public void righting(double strength){
        frontLeft.setPower(strength);
        backLeft.setPower(strength);
        frontRight.setPower(-strength);
        backRight.setPower(-strength);
    }
    public void lefting(double strength){
        frontLeft.setPower(-strength);
        backLeft.setPower(-strength);
        frontRight.setPower(strength);
        backRight.setPower(strength);
    }

}

