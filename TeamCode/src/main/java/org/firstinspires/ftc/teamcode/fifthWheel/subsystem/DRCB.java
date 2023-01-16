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
    public DcMotorEx leftMotor;
    public DcMotorEx rightMotor;
    public TouchSensor touch;

    public int level = 0;
    public int oldLevel = 0;

    // intake, low, mid, high,
    // top cone, top-1 cone, top-2 cone, top-3 cone
    // fifth cone is just intake level
    // lift up to low for picking them off the stack
    public static double LEVELS[] = {0, 105, 180, 285, 70, 50, 35, 20};
    private static final double LIFT_POWER = 1.0;

    private static final double TICKS_PER_REV = 1120;
    private static final double L_0 = 3.5; // motor to pivot dist
    private static final double L_A = 2.0; // bottom linkage
    private static final double L_B = 3.75; // top linkage
    private static final double L_OFFSET = 2.362; // linkage attachment dist
    private static final double THETA_0 = 2.094; // angle between horizontal and L_0 in rad
    public static double kTau_ff = 0.17; // gain for torque feedforward
    public static double kV = 0.002;
    public static double kA = 0.0;
    public static double downMultiplier = 0.93;
    public static double maxV = 1000;
    public static double maxA = 1000;

    public static double p = 0.03;
    public static double i = 0.0003;
    public static double d = 0.025;
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
        pid.setOutputRange(0.0, LIFT_POWER);
        pid.setTolerance(0);
        pid.enable();

        leftMotor = hwMap.get(DcMotorEx.class, lm);
        leftMotor.setDirection(DcMotor.Direction.FORWARD);
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotor.setPower(0);
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightMotor = hwMap.get(DcMotorEx.class, rm);
        rightMotor.setDirection(DcMotor.Direction.REVERSE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setPower(0);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        touch = hwMap.get(TouchSensor.class, ts);

        hardwareMap = hwMap;
    }

    public void setLevel(int l) {
        oldLevel = level;
        level = l;
        pid.reset();
        pid.enable();
//        pid.setSetpoint(LEVELS[level]);
        timer.reset();
    }

    public void run() {
        if (touch.isPressed() && getPosition() != 0) {
            reset();
        }
        setpoint = Control.trapMotionP(maxV, maxA, LEVELS[oldLevel], LEVELS[level], timer.seconds());
        updatePID();
        if (useMotionProfile) {
            pid.setSetpoint(setpoint);
        }
        // angle of motor between lift rest and lift horizontal in ticks
        ff = kTau_ff*calculateFeedforward(getPosition() - 100)
            + kV*Control.trapMotionV(maxV, maxA, LEVELS[oldLevel], LEVELS[level], timer.seconds())
            + kA*Control.trapMotionA(maxV, maxA, LEVELS[oldLevel], LEVELS[level], timer.seconds());

        output = pid.performPID(getPosition());
        total = ff + output;
        if (justFeedforward) {
            total = ff;
        }
        // if going down, reduce output cause gravity
        // TODO: take care of this in the model
        // HACK: this sucks
        if (total < 0.02) {
            total = total + (0.02 - total)/downMultiplier;
        }
        // set motor power and compensate for different battery voltages
        leftMotor.setPower(total*12/hardwareMap.voltageSensor.iterator().next().getVoltage());
        rightMotor.setPower(total*12/hardwareMap.voltageSensor.iterator().next().getVoltage());
    }

    public int getPosition() {
        return rightMotor.getCurrentPosition();
    }

    public double getVelocity() {
        return rightMotor.getVelocity();
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
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void updatePID() {
        pid.setPID(p, i, d);
    }

    // returns arbitrary units
    public double calculateFeedforward(double ticks) {
        double angle = 2*Math.PI*ticks/TICKS_PER_REV;
        double l_1 = Math.sqrt(Math.pow(L_A, 2)
                               + Math.pow(L_0, 2)
                               - 2*L_A*L_0*Math.cos(THETA_0 - angle));
        double theta_ab = lawOfCos(L_0, l_1, L_A) + lawOfCos(L_OFFSET, l_1, L_B);
        double theta_bc = lawOfCos(l_1, L_B, L_OFFSET);
        double theta = Math.PI - (theta_ab - angle) - theta_bc;

        double result = Math.cos(theta)/(Math.sin(theta_ab)*Math.sin(theta_bc));
        // HACK: cause my feedforward model sucks
        if ((-30 <= ticks) && (ticks < 175)) {
            result += 0.45;
        } else if (ticks > 180) {
            result -= 0.75;
        }
        return result;
    }

    // calculate angle of an SSS triangle
    // @param a is the side opposite to the angle you want to find
    private double lawOfCos(double a, double b, double c) {
        return Math.acos((Math.pow(b, 2) + Math.pow(c, 2) - Math.pow(a, 2))
                         / (2*b*c));
    }

    public Boolean arrived() {
        return pid.onTarget();
    }

    public int getCurrentLeftTicks() {
        return leftMotor.getCurrentPosition();
    }

    public int getCurrentRightTicks() {
        return rightMotor.getCurrentPosition();
    }
}