package org.firstinspires.ftc.teamcode.thirdWheel.hardware.lift;

import java.lang.Math;
// import java.lang.Thread;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.control.DcMotorUtils;

public class LinearSlide {
    private DcMotor slideMotor;
    private int level = 0;
    private double endPos = 0;

    private final double LEV_ZERO_TICKS     = 0.0;
    private final double LEV_ONE_TICKS      = 1000.0;
    private final double LEV_TWO_TICKS      = 2200.0;
    private final double LEV_THREE_TICKS    = 3500.0;
    private final double LEV_FOUR_TICKS     = 3600.0;

    public LinearSlide(HardwareMap hwMap) {
        init(hwMap);
    }

    private void init(HardwareMap hwMap) {
        slideMotor = hwMap.get(DcMotor.class, "slideMotor");
        slideMotor.setDirection(DcMotor.Direction.REVERSE);
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotor.setPower(0);
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // Move the slide to the position specified, in ticks
    // Except it only sets the power and you have to run it in a loop, because yes
    public int moveSlide(double desiredTicks, double startDec, double tolerance) {
        if (desiredTicks > LEV_THREE_TICKS) {
            return 1;
        }
        DcMotorUtils.moveByTicks(slideMotor, desiredTicks, startDec, tolerance);
        return 0;
    }

    public int setLevel(int desiredLevel) {
        switch (desiredLevel) {
        case 0:
            endPos = LEV_ZERO_TICKS;
            level = desiredLevel;
            break;
        case 1:
            endPos = LEV_ONE_TICKS;
            level = desiredLevel;
            break;
        case 2:
            endPos = LEV_TWO_TICKS;
            level = desiredLevel;
            break;
        case 3:
            endPos = LEV_THREE_TICKS;
            level = desiredLevel;
            break;
        case -1: // keep the same level
            break;
        default:
            // telemetry.addData("bruh", "Invalid level");
            // telemetry.update();
            break;
        }
        moveSlide(endPos, 230.0, 5.0);
        return 0;
    }

    public boolean arrived() { return DcMotorUtils.arrived(slideMotor, endPos, 5); }
    public double getEndPos() { return endPos; }
    public int getLevel() { return level; }
    public double getCurrentTicks() { return DcMotorUtils.getCurrentTicks(slideMotor); }
}
