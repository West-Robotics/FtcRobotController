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
	private static final double AUTO_INIT_POWER = 0.35;
	private static final double AUTO_INIT_ACCEL = 0.00015;
    private static final double JERK = 0.001;
    private static final double AUTO_JERK = 0.000001;
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

        // Set default values
        if (linearOpMode == null) {

            speed = INIT_POWER;
            accel = INIT_ACCEL;

        } else {

            speed = AUTO_INIT_POWER;
            accel = AUTO_INIT_ACCEL;

        }

    }

    public void spin(boolean red, boolean blue) {

        print("Carousel Speed", speed);
        print("Carousel Accel", accel);
        if (red || blue) {

            if (linearOpMode == null) {

                speed += accel;
                accel += JERK;

            } else {

                speed += accel;
                accel += AUTO_JERK;

            }

        }
        if (speed > FINAL_POWER) speed = FINAL_POWER;
        if (red) spinRed();
        else if (blue) spinBlue();
        else stop();

    }

    public void spinRed() {

		duckSpin.setPower(speed);

    }

    public void spinBlue() {

		duckSpin.setPower(-speed);

    }

    public void stop() {

        if (linearOpMode == null) {

            speed = INIT_POWER;
            accel = INIT_ACCEL;

        } else {

            speed = AUTO_INIT_POWER;
            accel = AUTO_INIT_ACCEL;

        }
        duckSpin.setPower(0);

    }

}
