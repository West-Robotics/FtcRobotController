package org.firstinspires.ftc.teamcode.hardware.drive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.PIDController;
import org.firstinspires.ftc.teamcode.hardware.BaseHardware;
import org.firstinspires.ftc.teamcode.hardware.Gyro;

public abstract class HolonomicDrive extends BaseHardware {

    protected boolean thirdWheel = false;
    // Boolean function for moveUntil()
    public interface BoolCommand {

        boolean check();

    }

    // Motors
    protected DcMotor frontLeft;
    protected DcMotor frontRight;
    protected DcMotor backLeft;
    protected DcMotor backRight;

    // Gyro
    protected Gyro gyro;

    // Modes
    protected boolean isDrivePOV = true;
    protected boolean isSlow = false;
    protected boolean isSquaredInputs = false;

    // For autonomous driving
    protected double wheelDiameter = 4;   // Diameter of driving wheels, default is 4
    protected double ticksPerRev = 1120;  // TPR of driving motors, defalut is 1120
    protected double currFlbr = 0; // Current theoretical x-value
    protected double currFrbl = 0; // Current theoretical y-value
    protected double currAngle = 0; // Current theoretical angle

    // For PID corrections
    protected PIDController pidDrive = new PIDController(0, 0, 0);
    protected PIDController pidFLBR = new PIDController(0, 0, 0);
    protected PIDController pidFRBL = new PIDController(0, 0, 0);
    protected PIDController pidTurn = new PIDController(0, 0, 0);
    protected double prevAngle;
    protected double correction;
    protected boolean isPID = true;

    protected static final double SLOW_MULTIPLIER = 0.5;

    // Set pidDrive values
    protected void setPidDrive(double p, double i, double d) {

        pidDrive = new PIDController(p, i, d);

    }

    // Set pidSpeed values
    protected void setPidSpeed(double p, double i, double d) {

        pidFLBR = new PIDController(p, i, d);
        pidFRBL = new PIDController(p, i, d);

    }

    // Set pidTurn values
    protected void setPidTurn(double p, double i, double d) {

        pidTurn = new PIDController(p, i, d);

    }

    // Call this function to stop correction
    protected void setPIDFalse() {

        isPID = false;

    }

    protected void setMotorDir(boolean flDir, boolean frDir, boolean blDir, boolean brDir) {

        if (flDir) frontLeft.setDirection(DcMotor.Direction.FORWARD);
        else frontLeft.setDirection(DcMotor.Direction.REVERSE);

        if (frDir) frontRight.setDirection(DcMotor.Direction.FORWARD);
        else frontRight.setDirection(DcMotor.Direction.REVERSE);

        if (blDir) backLeft.setDirection(DcMotor.Direction.FORWARD);
        else backLeft.setDirection(DcMotor.Direction.REVERSE);

        if (brDir) backRight.setDirection(DcMotor.Direction.FORWARD);
        else backRight.setDirection(DcMotor.Direction.REVERSE);

    }

    // Set the wheel diameter
    protected void setWheelDiameter(double wheelDiameter) {

        this.wheelDiameter = wheelDiameter;

    }

    // Set ticks per revolution
    protected void setTicksPerRev(double ticksPerRev) {

        this.ticksPerRev = ticksPerRev;

    }

    // For teleop
    public HolonomicDrive(OpMode opMode, HardwareMap hwMap) {

        super(opMode);

        gyro = new Gyro(hwMap, false);
        prevAngle = gyro.getAngleDegrees();
        gyro.reset();

        setupMotors(hwMap);

    }

    // For autonomous
    public HolonomicDrive(LinearOpMode opMode, HardwareMap hwMap) {

        super(opMode);

        gyro = new Gyro(hwMap);
        prevAngle = gyro.getAngleDegrees();
        gyro.reset();

        setupMotors(hwMap);

    }

    // Define and initialize motors
    private void setupMotors(HardwareMap hwMap) {

        frontLeft = hwMap.get(DcMotor.class, "frontLeft");
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setPower(0);

        frontRight = hwMap.get(DcMotor.class, "frontRight");
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setPower(0);

        backLeft = hwMap.get(DcMotor.class, "backLeft");
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setPower(0);

        backRight = hwMap.get(DcMotor.class, "backRight");
        backRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setPower(0);

        setupEncoders();

    }

    private void setupEncoders() {

        resetMotors();

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if (linearOpMode == null) {

            frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        } else {

            frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        }

    }

    public boolean isPOVMode() {
        return isDrivePOV;
    }

    // Toggle POV mode and field oriented mode
    public void togglePOV(boolean button) {

        if (button) {

            isDrivePOV = !isDrivePOV;
            gyro.reset();
            prevAngle = gyro.getAngleDegrees();

        }

    }

    // Enable squared inputs
    public void enableSquaredInputs() {

        isSquaredInputs = true;

    }

    // Toggle slow mode
    public void toggleSlow(boolean button) {

        if (button) isSlow = !isSlow;

    }

    // TeleOp Drive
    public void drive(double joystickX, double joystickY, double joystickTurn) {

        double x = joystickX;
        double y = joystickY;

        // Squared inputs mode
        if (isSquaredInputs) {

            // Store sign of value
            double signX;
            double signY;
            if (x < 0) signX = -1;
            else signX = 1;
            if (y < 0) signY = -1;
            else signY = 1;

            // Square the inputs
            x = Math.pow(x, 2) * signX;
            y = Math.pow(y, 2) * signY;

        }

        // Reduce joystick turn
        double turn = joystickTurn / 1.8;

        // PID calculation
        pidDrive.setSetpoint(prevAngle);
        correction = pidDrive.performPID(gyro.getAngleDegrees());

        if (!thirdWheel) {

            // If turning or not moving then don't correct
            if (Math.abs(turn) != 0 || (x == 0 && y == 0)) {

                pidDrive.disable();
                pidDrive.reset();
                prevAngle = gyro.getAngleDegrees();
                correction = 0;

            } else pidDrive.enable();

        } else {

            // Continuous tuning
            if (Math.abs(turn) != 0) {

                pidDrive.disable();
                pidDrive.reset();
                prevAngle = gyro.getAngleDegrees();
                correction = 0;

            } else pidDrive.enable();

        }

        // If field oriented drive, then set angleCompensation to gyro angle
        double angleCompensation = 0;
        if (!isDrivePOV)
            angleCompensation = gyro.getAngleRadians();

        // Holonomic drive calculations

        // The magnitude of the joystick
        double r = Math.hypot(x, y);

        // The angle of the robot it is supposed to move towards
        double robotAngle = Math.atan2(-y, x) - Math.PI / 4 - angleCompensation;

        // Takes the x and y components of the vector
        double cosinePow = r * Math.cos(robotAngle);
        double sinePow = r * Math.sin(robotAngle);

        // Takes the bigger number and multiplies it by a ratio to speed up the robot
        double maxPow = Math.max(Math.abs(cosinePow), Math.abs(sinePow));
        double ratio = 0;
        if (maxPow != 0) ratio = r / maxPow;

        // Motor powers without PID corrections
        double v1 = cosinePow * ratio + turn;
        double v2 = sinePow * ratio - turn;
        double v3 = sinePow * ratio + turn;
        double v4 = cosinePow * ratio - turn;

        // Divide by ratio to make turning easier

        double flbrMax = Math.max(Math.abs(v1), Math.abs(v4));
        if (flbrMax > 1.0) {

            v1 /= flbrMax;
            v4 /= flbrMax;

        }

        double frblMax = Math.max(Math.abs(v2), Math.abs(v3));
        if (frblMax > 1.0) {

            v2 /= frblMax;
            v3 /= frblMax;

        }

        // Don't correct if isPID is false
        if (!isPID) correction = 0;
        //correction = 0;

        // Add PID corrections
        v1 -= correction;
        v2 += correction;
        v3 -= correction;
        v4 += correction;

        // Set motor powers
        double multiplier;
        if (isSlow) multiplier = SLOW_MULTIPLIER;
        else multiplier = 1;

        frontLeft.setPower(v1 * multiplier);
        frontRight.setPower(v2 * multiplier);
        backLeft.setPower(v3 * multiplier);
        backRight.setPower(v4 * multiplier);

        // Telemetry values
        //print("FLPow: ", frontLeft.getPower());
        //print("FRPow: ", frontRight.getPower());
        //print("BLPow: ", backLeft.getPower());
        //print("BRPow: ", backRight.getPower());
        print("Correction: ", correction);
        print("IsPOV: ", isDrivePOV);
        print("IsSlow: ", isSlow);
        print("Encoder fl: ", frontLeft.getCurrentPosition());
        print("Encoder fr: ", frontRight.getCurrentPosition());
        print("Encoder bl: ", backLeft.getCurrentPosition());
        print("Encoder br: ", backRight.getCurrentPosition());
        print("Gyro: ", gyro.getAngleDegrees());

    }

    public void setPowers(double fl, double fr, double bl, double br) {

        frontLeft.setPower(fl);
        frontRight.setPower(fr);
        backLeft.setPower(bl);
        backRight.setPower(br);

    }

    public void moveUntil(double speed, double angleMove, BoolCommand command) {

        // Reset motors and gyro
        gyro.reset();
        resetMotors();

        // For correction turning
        pidDrive.reset();
        pidDrive.setSetpoint(0);
        pidDrive.enable();

        // Calculate speed
        double flbrSpeed = speed * Math.sin((45.0 + angleMove) * Math.PI / 180);
        double frblSpeed = speed * Math.cos((45.0 + angleMove) * Math.PI / 180);

        // Maximize speed by calculating ratio
        double max = Math.max(Math.abs(flbrSpeed), Math.abs(frblSpeed));
        flbrSpeed = speed * (flbrSpeed / max);
        frblSpeed = speed * (frblSpeed / max);

        do {

            correction = pidDrive.performPID(gyro.getAngleDegrees());

            frontLeft.setPower(flbrSpeed - correction);
            frontRight.setPower(frblSpeed + correction);
            backLeft.setPower(frblSpeed - correction);
            backRight.setPower(flbrSpeed + correction);

            print("flbrP", flbrSpeed);
            print("frblP", frblSpeed);
            print("Boolean value: ", command.check());

            linearOpMode.telemetry.update();

        } while (linearOpMode.opModeIsActive() && !command.check());

        stop();

        linearOpMode.sleep(500);

    }

    public void move(double speed, double distance, double angleMove) {

        // The number of ticks the motors have to move without considering direction of motors
        int goalTicks = (int) ((distance / (wheelDiameter * Math.PI)) * ticksPerRev);

        // Calculate distance of motors
        int flbrDist = (int) (goalTicks * Math.sin(Math.toRadians(45.0 + angleMove)));
        int frblDist = (int) (goalTicks * Math.cos(Math.toRadians(45.0 + angleMove)));

        // Calculate speeds
        double max = Math.max(Math.abs(flbrDist), Math.abs(frblDist));
        double goalFLBRSpeed = speed * (flbrDist / max);
        double goalFRBLSpeed = speed * (frblDist / max);

        // For correction turning
        pidDrive.reset();
        pidDrive.setSetpoint(currAngle);
        pidDrive.enable();

        // For frontleft and backright motors
        pidFLBR.reset();
        pidFLBR.setInputRange(currFlbr, currFlbr + flbrDist);
        pidFLBR.setOutputRange(0, goalFLBRSpeed);
        pidFLBR.setSetpoint(currFlbr + flbrDist);
        pidFLBR.setTolerance(80);
        pidFLBR.enable();

        // For frontright and backleft motors
        pidFRBL.reset();
        pidFRBL.setInputRange(currFrbl, currFrbl + frblDist);
        pidFRBL.setOutputRange(0, goalFRBLSpeed);
        pidFRBL.setSetpoint(currFrbl + frblDist);
        pidFRBL.setTolerance(80);
        pidFRBL.enable();

        boolean isFLBROnTarget = false;
        boolean isFRBLOnTarget = false;
        if (flbrDist == 0) isFLBROnTarget = true;
        else if (frblDist == 0) isFRBLOnTarget = true;

        do {

            correction = pidDrive.performPID(gyro.getAngleDegrees());

            double avgFLBRPos = (frontLeft.getCurrentPosition() + backRight.getCurrentPosition()) / 2.0;
            double speedFLBR = pidFLBR.performPID(avgFLBRPos);

            double avgFRBLPos = (frontRight.getCurrentPosition() + backLeft.getCurrentPosition()) / 2.0;
            double speedFRBL = pidFRBL.performPID(avgFRBLPos);

            frontLeft.setPower(speedFLBR - correction);
            frontRight.setPower(speedFRBL + correction);
            backLeft.setPower(speedFRBL - correction);
            backRight.setPower(speedFLBR + correction);

            //print("Gyro: ", gyro.getAngleDegrees());
            print("FL: ", frontLeft.getCurrentPosition());
            print("FR: ", frontRight.getCurrentPosition());
            print("BL: ", backLeft.getCurrentPosition());
            print("BR: ", backRight.getCurrentPosition());
            //print("Correction: ", correction);
            print("FLBR Setpoint: ", pidFLBR.getSetpoint() + " " + Double.toString(flbrDist));
            print("FRBL Setpoint: ", pidFRBL.getSetpoint() + " " + Double.toString(frblDist));
            //print("FLBR Speed: ", speedFLBR);
            //print("FRBL Speed: ", speedFRBL);
            //print("FLBR Ontarget: ", pidFLBR.onTarget() || isFLBROnTarget);
            //print("FRBL Ontarget: ", pidFRBL.onTarget() || isFRBLOnTarget);

            linearOpMode.telemetry.update();

        } while (linearOpMode.opModeIsActive() && ((!pidFRBL.onTarget() && !isFRBLOnTarget) || (!pidFLBR.onTarget() && !isFLBROnTarget)));

        // Reset turn
        gyro.reset();
        currAngle = 0;

        // Stop motors
        stop();

        // Add theoretical current position
        currFlbr += flbrDist;
        currFrbl += frblDist;

        // Add slight delay
        linearOpMode.sleep(500);

    }

    public void turn(double speed, int angle) {

        if (Math.abs(angle) > 359) angle = (int) Math.copySign(359, angle);

        pidTurn.reset();
        pidTurn.setInputRange(currAngle, currAngle - angle);
        pidTurn.setOutputRange(0, speed);
        pidTurn.setSetpoint(currAngle - angle);
        pidTurn.setTolerance(2);
        pidTurn.enable();

        do {

            speed = pidTurn.performPID(gyro.getAngleDegrees());

            frontLeft.setPower(-speed);
            frontRight.setPower(speed);
            backLeft.setPower(-speed);
            backRight.setPower(speed);

            print("FL: ", frontLeft.getCurrentPosition());
            print("FR: ", frontRight.getCurrentPosition());
            print("BL: ", backLeft.getCurrentPosition());
            print("BR: ", backRight.getCurrentPosition());
            print("Curr Angle: ", gyro.getAngleDegrees());
            print("Setpoint: ", pidTurn.getSetpoint());
            print("Is setpoint: ", pidTurn.onTarget());

            linearOpMode.telemetry.update();

        } while (linearOpMode.opModeIsActive() && !pidTurn.onTarget());

        // Add to current angle
        currAngle -= angle;

        // Stop motors
        stop();

        // Reset motors
        resetMotors();

        // Add slight delay
        linearOpMode.sleep(500);

    }

    public void stop() {

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

    }

    public void resetMotors() {

        stop();

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        currFlbr = 0;
        currFrbl = 0;

    }

    public double getVoltage(HardwareMap hwMap) {

        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hwMap.voltageSensor)
            if (sensor.getVoltage() > 0) result = Math.min(result, sensor.getVoltage());
        return result;

    }

}
