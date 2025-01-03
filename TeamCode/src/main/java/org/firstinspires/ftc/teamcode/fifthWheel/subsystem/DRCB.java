package org.firstinspires.ftc.teamcode.fifthWheel.subsystem;

import java.lang.Math;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.PIDController;
import org.firstinspires.ftc.teamcode.util.control.Control;

import com.acmerobotics.dashboard.config.Config;

@Config
public class DRCB {
    public DcMotorEx liftLeft;
    public DcMotorEx liftRight;
    public TouchSensor touch;

    public int level = 0;
    public int oldLevel = 0;

    // intake, low, mid, high,
    // top cone, top-1 cone, top-2 cone, top-3 cone, off-stack
    // fifth cone is just intake level
    // lift up to low for picking them off the stack
    public static double LEVELS[] = {  -15,  350, 600,   1100,
                                      197,  175,  120,     60, 310 };

    private static final double TICKS_PER_REV = 1425.1;
    public static double kTau_ff = 0.1; // gain for torque feedforward
    public static double kV = 0.0007;
    // public static double kV = 0.0015;
    public static double kA = 0.0;
    // public static double kA = 0.0007;
    public static double maxV = 2700;
    public static double maxA = 2700;
    public static double offset = 0.6;
    public static double bumpFactor = 1;

    public static double p = 0.004;
    public static double i = 0.0;
    public static double d = 0.01;
    public PIDController pid = new PIDController(p, i, d);

    public double ff = 0.0;
    public double output = 0.0;
    public double total = 0.0;

    public double setpoint = 0.0;
    public ElapsedTime timer = new ElapsedTime();

    public Boolean useMotionProfile = true;
    public Boolean justFeedforward = false;

    private HardwareMap hardwareMap;

    public DRCB(HardwareMap hwMap, String lm, String rm, String ts) {
        pid.reset();
        pid.setInputRange(0, LEVELS[3]);
        pid.setOutputRange(0.0, 1.0);
        pid.setTolerance(0);
        pid.enable();

        liftLeft = hwMap.get(DcMotorEx.class, lm);
        liftLeft.setDirection(DcMotor.Direction.REVERSE);
        // TODO: coast
        liftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        liftLeft.setPower(0);
        liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        liftRight = hwMap.get(DcMotorEx.class, rm);
        liftRight.setDirection(DcMotor.Direction.FORWARD);
        liftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        liftRight.setPower(0);
        liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        touch = hwMap.get(TouchSensor.class, ts);

        hardwareMap = hwMap;
    }

    public void setPower(double input) {
        liftLeft.setPower(input);
        liftRight.setPower(input);
    }

    public void setLevel(int l) {
        oldLevel = level;
        level = l;
        pid.reset();
        pid.enable();
//        pid.setSetpoint(LEVELS[level]);
        timer.reset();
    }

    public void run(double input) {
        setpoint = Control.trapMotionP(maxV, maxA, LEVELS[oldLevel], LEVELS[level], timer.seconds());
        updatePID();
        if (useMotionProfile) {
            pid.setSetpoint(setpoint);
        }
        if (level == 0) {
            ff = kTau_ff*calculateFeedforward(getPosition() - 350)
                    + kV*Control.trapMotionV(maxV, maxA/3, LEVELS[oldLevel], LEVELS[level], timer.seconds())
                    + kA*Control.trapMotionA(maxV, maxA/3, LEVELS[oldLevel], LEVELS[level], timer.seconds());
        } else if (level == 8) {
            // angle of motor between lift rest and lift horizontal in ticks
            ff = kTau_ff * calculateFeedforward(getPosition() - 350)
                    + kV * Control.trapMotionV(maxV/4, maxA/8, LEVELS[oldLevel], LEVELS[level], timer.seconds())
                    + kA * Control.trapMotionA(maxV/4, maxA/8, LEVELS[oldLevel], LEVELS[level], timer.seconds());
        } else {
            // angle of motor between lift rest and lift horizontal in ticks
            ff = kTau_ff * calculateFeedforward(getPosition() - 350)
                    + kV * Control.trapMotionV(maxV, maxA, LEVELS[oldLevel], LEVELS[level], timer.seconds())
                    + kA * Control.trapMotionA(maxV, maxA, LEVELS[oldLevel], LEVELS[level], timer.seconds());
        }
        if (ff < -0.1) {
            ff = -0.1;
        }

        output = pid.performPID(getPosition());
        total = ff + output;
        if (justFeedforward) {
            total = ff;
        }
        // set motor power and compensate for different battery voltages
        total = total*12.0/hardwareMap.voltageSensor.iterator().next().getVoltage();
        if (total > 1) {
            total = 1;
        }
//        } else if (total < -0.2) {
//            total = -0.2;
//        }
        if (touch.isPressed() && level == 0) {
            if (getPosition() != 0) {
                reset();
            }
            liftLeft.setPower(0);
            liftRight.setPower(0);
        } else {
            liftLeft.setPower(total+input);
            liftRight.setPower(total+input);
        }
    }

    public int getPosition() {
        return liftRight.getCurrentPosition();
    }

    public double getVelocity() {
        return liftRight.getVelocity();
    }

    public double getTargetPosition() {
        return Control.trapMotionP(maxV, maxA, LEVELS[oldLevel], LEVELS[level], timer.seconds());
    }
    public double getTargetVelocity() {
        return Control.trapMotionV(maxV, maxA, LEVELS[oldLevel], LEVELS[level], timer.seconds());
    }
    public double getTargetAcceleration() {
        return Control.trapMotionA(maxV, maxA, LEVELS[oldLevel], LEVELS[level], timer.seconds());
    }

    public void setGains(double kP, double kV, double kA) {
        this.kTau_ff = kP;
        this.kV = kV;
        this.kA = kA;
    }

    private void reset() {
        liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void updatePID() {
        pid.setPID(p, i, d);
    }

    // returns arbitrary units
    public double calculateFeedforward(double ticks) {
        double theta = 2*Math.PI*ticks/TICKS_PER_REV;
        double result = Math.cos(theta) + offset;
        if (600-350 < ticks && ticks < 730-350) {
            result = result * bumpFactor;
        }

        return result;
    }

    public Boolean arrived() {
        return pid.onTarget();
    }

    public int getCurrentLeftTicks() {
        return liftLeft.getCurrentPosition();
    }

    public int getCurrentRightTicks() {
        return liftRight.getCurrentPosition();
    }
}
