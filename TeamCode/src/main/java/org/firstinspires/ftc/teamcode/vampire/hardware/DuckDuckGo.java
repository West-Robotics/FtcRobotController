package org.firstinspires.ftc.teamcode.vampire.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.BaseHardware;

public class DuckDuckGo extends BaseHardware {

    // Motor and motor power
    private CRServo duckSpin;
    private static final double INIT_POWER = 0.5;
    private static final double ACCEL = 0.001;
    private double speed = INIT_POWER;

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
        duckSpin = hwMap.get(CRServo.class, "spin");
        duckSpin.setDirection(DcMotor.Direction.FORWARD);

    }

    public void spin(boolean red, boolean blue) {

        print("Carousel Speed:", speed);
        if (red || blue) speed += ACCEL;
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
        duckSpin.setPower(0);

    }

}
