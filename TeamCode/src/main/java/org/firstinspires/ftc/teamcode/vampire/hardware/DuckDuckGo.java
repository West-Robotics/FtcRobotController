package org.firstinspires.ftc.teamcode.vampire.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.BaseHardware;

public class DuckDuckGo extends BaseHardware {

    // Motor and motor power
    private DcMotor duckSpin;
    private static final double SLOW_AUTO_RED = 0.38;
    private static final double SLOW_AUTO_BLUE = 0.42;
    private static final double SLOW_TELEOP = 0.55;
    private static final int SPEED_TICKS_AUTO = 1700;
    private static final int SPEED_TICKS_TELEOP = 1550;
    private static final int STOP_TICKS_TELEOP = 2300;
    private static final int STOP_TICKS_AUTO = 2300;
    private double speed = SLOW_AUTO_RED;

	// For autonomous
	public static final double AUTO_TIME = 2;

    // Teleop constructor
    public DuckDuckGo(OpMode opMode, HardwareMap hwMap) {

        super(opMode);
        setup(hwMap);

    }

    // Autonomous constructor
    public DuckDuckGo(LinearOpMode opMode, HardwareMap hwMap) {

        super(opMode);
        setup(hwMap);

    }

    private void setup(HardwareMap hwMap) {

        // Set up motor
        duckSpin = hwMap.get(DcMotor.class, "spin");
        duckSpin.setDirection(DcMotor.Direction.REVERSE);
        duckSpin.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        duckSpin.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        duckSpin.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    public void spin(boolean red, boolean blue) {

        // Print out values
        print("Carousel Speed", speed);
        print("Carousel Position", duckSpin.getCurrentPosition());

        // Variable speed depending on encoder value
        if (opMode != null) {

            if (isStopPosition()) speed = 0;
            else if (Math.abs(duckSpin.getCurrentPosition()) > SPEED_TICKS_TELEOP) speed = 1;
            else speed = SLOW_TELEOP;

        } else {

            if (isStopPosition()) speed = 0;
            else if (Math.abs(duckSpin.getCurrentPosition()) > SPEED_TICKS_AUTO) speed = 1;
            else {

                if (red) speed = SLOW_AUTO_RED;
                else speed = SLOW_AUTO_BLUE;

            }

        }

        // Power the motors (stop if above the stopping threshold)
        if (red) duckSpin.setPower(-speed);
        else if (blue) duckSpin.setPower(speed);
        else stop();

    }

    public void stop() {

        duckSpin.setPower(0);
        duckSpin.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        duckSpin.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    public boolean isStopPosition() {

        if (opMode != null) return Math.abs(duckSpin.getCurrentPosition()) > STOP_TICKS_AUTO;
        else return Math.abs(duckSpin.getCurrentPosition()) > STOP_TICKS_TELEOP;

    }

}
