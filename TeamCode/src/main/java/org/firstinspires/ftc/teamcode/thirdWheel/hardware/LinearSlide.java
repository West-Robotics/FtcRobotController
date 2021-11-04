package org.firstinspires.ftc.teamcode.thirdWheel.hardware;

import java.lang.Math;
import java.lang.Thread;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.BaseHardware;

public class LinearSlide extends BaseHardware {
    private DcMotor slideMotor;
    private int level = 0;
    private int endPos = 0;

    private final int LEV_ZERO_TICKS     = 0;
    private final int LEV_ONE_TICKS      = 560;
    private final int LEV_TWO_TICKS      = 1680;
    private final int LEV_THREE_TICKS    = 2800;

    public LinearSlide(LinearOpMode opMode, HardwareMap hwMap) {
        super(opMode);
        init(hwMap);
    }

    public LinearSlide(OpMode opMode, HardwareMap hwMap) {
        super(opMode);
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
    private void move(int desiredTicks, int tolerance, int startDec) {
        int direction = getDifference(desiredTicks)/Math.abs(getDifference(desiredTicks));
        // Ignore this: Deceleration: -1.02^(-x) + 1 where x = difference
        // Deceleration: min(x, 1) where x = difference
        while (Math.abs(getDifference(desiredTicks)) > tolerance ) {       // BUG: When 1/startDec < 1, it gets stuck in an infinite loop?
            slideMotor.setPower((Math.min(Math.abs(getDifference(desiredTicks))*(1/startDec), 1)) * direction);
            // slideMotor.setPower(-Math.pow(-1.02, -getDifference(desiredTicks)) + 1);
        }
        slideMotor.setPower(0);
    }

    public int setLevel(int desiredLevel) {
        switch (desiredLevel) {
            case 0:
                endPos = LEV_ZERO_TICKS;
                break;
            case 1:
                endPos = LEV_ONE_TICKS;
                break;
            case 2:
                endPos = LEV_TWO_TICKS;
                break;
            case 3:
                endPos = LEV_THREE_TICKS;
                break;
            default:
                // telemetry.addData("bruh", "Invalid level");
                // telemetry.update();
                return 1;
        }
        move(endPos, 50, 10);    // Move to position quickly (inaccurate)
        level = desiredLevel;
        return 0;
    }

    public int getDifference(int desiredTicks) { return desiredTicks - slideMotor.getCurrentPosition(); }
    public int getEndPos() { return endPos; }
    public int getLevel() { return level; }
    public int getCurrentTicks() { return slideMotor.getCurrentPosition(); }
}
