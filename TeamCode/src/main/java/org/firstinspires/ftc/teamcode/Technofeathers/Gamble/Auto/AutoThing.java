package org.firstinspires.ftc.teamcode.Technofeathers.Gamble.Auto;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Autonomous
public class AutoThing extends LinearOpMode{

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
    double Kp = PIDNow.Kp;
    double Ki = PIDNow.Ki;
    double Kd = PIDNow.Kd;
    double PStraight = PIDNow.PForTurns;
    double IStraight = PIDNow.IForTurns;
    double DStraight = PIDNow.DForTurns;
    double PforMove = PIDNow.PforStraigth;
    double IforMove = PIDNow.IforStraight;
    double DforMove = PIDNow.DforStraight;
    double Yaw;
    IMU imu;
    double currentAngle;
    int intakeTicks;
    IMU.Parameters imuParameters;
    double state;
    double targetAngle;
    double lastAngle = 0;
    double secondLastAngle = 0;
    YawPitchRollAngles robotOrientation;

    public void runOpMode() throws InterruptedException{

        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        imu = hardwareMap.get(IMU.class, "imu");

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);


        imuParameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );
        imu.initialize(imuParameters);
        double currentVoltage = 13.18;
        //setPIDValues((12.5/currentVoltage));
        waitForStart();



        resetAngles();
        double strength;
        double targetDegree = 90;
        while (opModeIsActive()){
            robotOrientation=imu.getRobotYawPitchRollAngles();
            Yaw = robotOrientation.getYaw(AngleUnit.DEGREES);
            state = Math.toRadians(Yaw);
            targetAngle = Math.toRadians(-targetDegree);
            strength = PIDControl(targetAngle,state,PStraight,DStraight,IStraight);
            lefting(strength);
            telemetry.addData("Current Angle", Yaw);
            telemetry.update();
        }
        power(0);
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
            telemetry.addData("Current Angle", Yaw);
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

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


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

