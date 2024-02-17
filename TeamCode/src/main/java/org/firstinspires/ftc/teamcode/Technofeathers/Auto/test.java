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
import org.checkerframework.checker.units.qual.K;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Autonomous
public class test extends LinearOpMode{

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

        double Kp = PIDNowConstants.Kp;
        double Ki = PIDNowConstants.Ki;
        double Kd = PIDNowConstants.Kd;

        double PStraight = PIDNowConstants.PForTurns;
        double IStraight = PIDNowConstants.IForTurns;
        double DStraight = PIDNowConstants.DForTurns;

        double PforMove = PIDNowConstants.PforStraigth;
        double IforMove = PIDNowConstants.IforStraight;
        double DforMove = PIDNowConstants.DforStraight;


        double Yaw;
        IMU imu;
        double currentAngle;
        int intakeTicks;

        IMU.Parameters imuParameters;

        double state;
        double targetAngle;

        double lastAngle = 0;
        double secondLastAngle = 0;
        double thirdLastAngle = 0;
        double fourthLastAngle = 0;
        double fifthLastAngle = 0;
        double sixthLastAngle = 0;
        double seventhLastAngle = 0;
        double eigthLastAngle = 0;
        YawPitchRollAngles robotOrientation;

        public void runOpMode() throws InterruptedException{

            frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
            backLeft = hardwareMap.get(DcMotor.class, "backLeft");
            frontRight = hardwareMap.get(DcMotor.class, "frontRight");
            backRight = hardwareMap.get(DcMotor.class, "backRight");
            distanceSensor = hardwareMap.get(DistanceSensor.class,"distSense1");
            leftDistanceSensor = hardwareMap.get(DistanceSensor.class, "distSense2");
            rightDistanceSensor = hardwareMap.get(DistanceSensor.class, "distSense3");
            imu = hardwareMap.get(IMU.class, "imu");
            lift1 = hardwareMap.get(DcMotor.class,"lift1");
            lift2 = hardwareMap.get(DcMotor.class,"lift2");
            intake = hardwareMap.get(DcMotor.class, "intake");
            stopper = hardwareMap.get(Servo.class,"stopper");
            pivot1 = hardwareMap.get(Servo.class, "pivot1");
            grabber = hardwareMap.get(Servo.class, "grabber");
            intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
            double currentVoltage = 12.85;
            setPIDValues((12.75/currentVoltage));
            waitForStart();

            turnRight(90);
            //move(0,0.05,2);



        }
        public void setPIDValues(double voltage){
            Kp = Kp * voltage;
            Ki = Ki * voltage;
            Kd = Kd * voltage;
            PStraight = PStraight * voltage;
            IStraight = IStraight * voltage;
            DStraight = DStraight * voltage;
            PforMove = PforMove * voltage;
            IforMove = IforMove * voltage;
            DforMove = DforMove * voltage;
        }
        public void move(double straightAngle, double distanceWantedInMeters, int oneRightTwoLeftThreeBack){
            resetAngles();
            double distance;
            double powering;
            double leftPower;
            double rightPower;
            double erroring = 2;
            double lasterroring;
            double angleErroring = 2;
            double lastAngleErroring ;
            do {
                robotOrientation = imu.getRobotYawPitchRollAngles();
                Yaw = robotOrientation.getYaw(AngleUnit.DEGREES);
                double state = Math.toRadians(Yaw);
                lastAngleErroring = angleErroring;
                angleErroring = Math.abs(straightAngle-Yaw);
                if (oneRightTwoLeftThreeBack == 1) {
                    distance = rightDistanceSensor.getDistance(DistanceUnit.METER);
                } else if(oneRightTwoLeftThreeBack ==2) {
                    distance = leftDistanceSensor.getDistance(DistanceUnit.METER);
                } else {
                    distance = distanceSensor.getDistance(DistanceUnit.METER);
                }
                telemetry.addData("Distance:", distance);
                powering= PIDControlForStraight(distanceWantedInMeters,distance);
                lasterroring = erroring;
                erroring = Math.abs(distanceWantedInMeters - distance);
                telemetry.addData("Distance Error", erroring);

                double targetAng = Math.toRadians(straightAngle);
                double pidCorrection = PIDControl(targetAng,state,PforMove,DforMove,IforMove);
                leftPower = powering - pidCorrection;
                rightPower = powering + pidCorrection;

                telemetry.addData("leftpower", leftPower);
                telemetry.addData("rightpower", rightPower);
                telemetry.update();
                frontLeft.setPower(leftPower);
                backLeft.setPower(leftPower);
                frontRight.setPower(rightPower);
                backRight.setPower(rightPower);
                sleep(6);
            } while (opModeIsActive() && ((lasterroring > 0.02 || lastAngleErroring > 2.2) || (erroring > 0.02 || angleErroring > 2.2)));
            power(0);
        }
        public  void turnRight(double targetDegree){
            robotOrientation = imu.getRobotYawPitchRollAngles();
            Yaw = robotOrientation.getYaw(AngleUnit.DEGREES);
            resetAngles();
            double lasteror;
            double erroring = 5;
            do {
                secondLastAngle = lastAngle;
                lastAngle = Yaw;
                robotOrientation = imu.getRobotYawPitchRollAngles();
                Yaw = robotOrientation.getYaw(AngleUnit.DEGREES);
                state = Math.toRadians(Yaw);
                double distance = distanceSensor.getDistance(DistanceUnit.CM);
                telemetry.addData("Current Angle", Yaw);
                telemetry.addData("Distance:", distance);
                targetAngle = Math.toRadians(-targetDegree);
                double strength = PIDControl(targetAngle,state,PStraight,DStraight,IStraight);
                lasteror = erroring;
                erroring = Math.abs(-targetDegree-Yaw);
                telemetry.addData("lasteror",lasteror);
                telemetry.addData("error for lasteror",erroring);
                telemetry.update();
                lefting(strength);
                sleep(12);
            }while (opModeIsActive() && (lasteror>0.5 || erroring>0.5));
            power(0);
        }
        public void turnLeft(double targetDegree){
            robotOrientation = imu.getRobotYawPitchRollAngles();
            Yaw = robotOrientation.getYaw(AngleUnit.DEGREES);
            resetAngles();
            double lasteror;
            double error = 5;
            do {
                secondLastAngle = lastAngle;
                lastAngle = Yaw;
                robotOrientation = imu.getRobotYawPitchRollAngles();
                Yaw = robotOrientation.getYaw(AngleUnit.DEGREES);
                state = Math.toRadians(Yaw);
                double distance = distanceSensor.getDistance(DistanceUnit.CM);
                telemetry.addData("Current Angle", Yaw);
                telemetry.addData("Distance:", distance);
                telemetry.update();
                targetAngle = Math.toRadians(targetDegree);
                double strength = PIDControl(targetAngle,state,PStraight,DStraight,IStraight);
                lasteror = error;
                error = Math.abs(targetDegree-Yaw);
                lefting(strength);
                sleep(12);
            } while (opModeIsActive() && (lasteror>0.5 || error>0.5));
            power(0);
        }

        public void resetAngles() {
            lastAngle = 0;
            secondLastAngle = 0;
        }

        public void releaseIntake(){
            telemetry.addData("intake", intakeTicks);
            telemetry.update();
            intake.setPower(-0.3);
            sleep(2000);
            intake.setPower(0);
        }

        public double PIDControl(double reference, double state,double p, double d, double i){
            double error = angleReset(reference-state);
            telemetry.addData("Error",error);
            double angle = Math.toDegrees(error);
            telemetry.addData("Error in Degrees", angle);
            integralSum += error * timer.seconds();
            double derivative = (error - lastError) / (timer.seconds());
            lastError = error;
            timer.reset();
            double returning = (error * p) + (derivative * d) + (integralSum * i);
            return returning;
        }

        public double PIDControlForStraight(double reference, double state){
            double error2 = reference - state;
            integralSum2 += error2 * timer.seconds();
            double derivative = (error2 - lastError2) / (timer.seconds());
            lastError2 = error2;
            timer.reset();
            telemetry.addData("Error2", error2);
            double returning2 = (error2 * PforMove) + (derivative * DforMove) + (integralSum2 * IforMove);
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

        public void lefting(double strength){
            frontLeft.setPower(-strength);
            backLeft.setPower(-strength);
            frontRight.setPower(strength);
            backRight.setPower(strength);
        }

    }
