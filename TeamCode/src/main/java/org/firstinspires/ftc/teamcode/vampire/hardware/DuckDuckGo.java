package org.firstinspires.ftc.teamcode.vampire.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.BaseHardware;

public class DuckDuckGo extends BaseHardware {

    // Motor and motor power
    private DcMotor duckSpin;
    private static final double INIT_POWER = 0.45;
    private static final double INIT_ACCEL = 0.01;
	private static final double AUTO_POWER = 0.6;
    private static final double JERK = 0.001;
    private static final double FINAL_POWER = 1;
    private double speed = INIT_POWER;
    private double accel = INIT_ACCEL;
	
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

        // Set up servo
        duckSpin = hwMap.get(DcMotor.class, "spin");
        duckSpin.setDirection(DcMotor.Direction.FORWARD);

    }

    public void spin(boolean red, boolean blue) {

        print("Carousel Speed", speed);
        print("Carousel Accel", accel);
        if (red || blue) {

            speed += accel;
            accel += JERK;

        }
        if (speed > FINAL_POWER) speed = FINAL_POWER;
        if (red) spinRed();
        else if (blue) spinBlue();
        else stop();

    }

    public void spinRed() {

		if (opMode == null) duckSpin.setPower(-AUTO_POWER);
		else duckSpin.setPower(-speed);

    }

    public void spinBlue() {

		if (opMode == null) duckSpin.setPower(AUTO_POWER);
		else duckSpin.setPower(speed);

    }

    public void stop() {

        speed = INIT_POWER;
        accel = INIT_ACCEL;
        duckSpin.setPower(0);

    }

}
