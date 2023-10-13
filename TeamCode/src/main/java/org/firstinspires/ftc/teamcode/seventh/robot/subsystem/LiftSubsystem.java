package org.firstinspires.ftc.teamcode.seventh.robot.subsystem;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.seventh.robot.hardware.Globals;
import org.firstinspires.ftc.teamcode.seventh.robot.hardware.Hardware;

public class LiftSubsystem {
    private Hardware hardware;

    private double lastPower = 0.0;
//    private double power = 0.0;

    public LiftSubsystem(Hardware hardware) {
        this.hardware = hardware;
        hardware.liftLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        hardware.liftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hardware.liftLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hardware.liftRight.setDirection(DcMotorSimple.Direction.REVERSE);
        hardware.liftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hardware.liftRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hardware.liftLeftEnc.setDirection(Motor.Direction.FORWARD);
        hardware.liftLeftEnc.setDistancePerPulse(Globals.LIFT_DISTANCE_PER_PULSE);
        hardware.liftLeftEnc.reset();
        hardware.liftRightEnc.setDirection(Motor.Direction.FORWARD);
        hardware.liftRightEnc.setDistancePerPulse(Globals.LIFT_DISTANCE_PER_PULSE);
        hardware.liftRightEnc.reset();
        update(0);
    }

    public void update(double p) {
        if (hardware.limit.isPressed()) {
            hardware.liftLeftEnc.reset();
            hardware.liftRightEnc.reset();
        }
        write(p);
    }

    private void write(double p) {
        if ((hardware.liftLeftEnc.getDistance() <= Globals.LIFT_MIN && p < 0)
                || (Globals.LIFT_MAX < hardware.liftLeftEnc.getDistance() && p > 0)) {
            p = p/2;
        }
        if (lastPower != p) {
            lastPower = p;
            hardware.liftLeft.setPower(p);
            hardware.liftRight.setPower(p);
        }
    }

    public double getLeftDistance() {
        return hardware.liftLeftEnc.getDistance();
    }
    public double getRightDistance() {
        return hardware.liftRightEnc.getDistance();
    }
}
