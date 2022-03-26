package org.firstinspires.ftc.teamcode.vampire.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.BaseHardware;

public class DuckDuckGo extends BaseHardware {

    // Motor and motor power
    private DcMotor duckSpin;
    private static final double SLOW_SPEED = 0.6;
    private static final double FAST_SPEED = 1;
    private static final int SPEED_TICKS = 3000;
    private static final int STOP_TICKS = 4000;
    private double speed = SLOW_SPEED;

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
        duckSpin.setDirection(DcMotor.Direction.FORWARD);
        duckSpin.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        duckSpin.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        duckSpin.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // TODO: Check if RUN_USING_ENCODER is better

    }

    public void spin(boolean red, boolean blue) {

        // Print out values
        print("Carousel Speed", speed);
        print("Carousel Position", duckSpin.getCurrentPosition());

        // Variable speed depending on encoder value
        if (isStopPosition()) speed = 0;
        else if (Math.abs(duckSpin.getCurrentPosition()) > SPEED_TICKS) speed = FAST_SPEED;
        else speed = SLOW_SPEED;

        // Power the motors (stop if above the stopping threshold)
        if (red) duckSpin.setPower(speed);
        else if (blue) duckSpin.setPower(-speed);
        else stop();

    }

    public void stop() {

        duckSpin.setPower(0);
        duckSpin.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        duckSpin.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // TODO: Here too

    }

    public boolean isStopPosition() { return Math.abs(duckSpin.getCurrentPosition()) > STOP_TICKS; }

}
