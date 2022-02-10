package org.firstinspires.ftc.teamcode.vampire.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.BaseHardware;

public class DuckDuckGo extends BaseHardware {

    // Motor and motor power
    private DcMotor duckSpin;
    private static final double INIT_POWER = 0.35;
    private static final double INIT_ACCEL = 0.005;
    private static final double JERK = 0.0012;
    private static final double FINAL_POWER = 1;
    private double speed = INIT_POWER;
    private double accel = INIT_ACCEL;

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

    private void spinRed() {

        duckSpin.setPower(-speed);

    }

    private void spinBlue() {

        duckSpin.setPower(speed);

    }

    public void stop() {

        speed = INIT_POWER;
        accel = INIT_ACCEL;
        duckSpin.setPower(0);

    }

}
