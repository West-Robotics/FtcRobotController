package org.firstinspires.ftc.teamcode.vampire.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.BaseHardware;

public class TapeArm extends BaseHardware {

    // Servos
    private Servo horz;
    private Servo vert;
    private CRServo rollTop;
    private CRServo rollBottom;

    // Constants
    private static final double SLOW_SPEED = -0.003;
    private static final double FAST_SPEED = -0.015;
    private static final double ROLL_POW = 0.75;
    private static final double MAX_POS = 0.75;
    private static final double VERT_MULTIPLIER = 2;
    private double speed = SLOW_SPEED;

    // Teleop constructor
    public TapeArm(OpMode opMode, HardwareMap hwMap) {

        super(opMode);
        setup(hwMap);

    }

    // Autonomous constructor
    public TapeArm(LinearOpMode opMode, HardwareMap hwMap) {

        super(opMode);
        setup(hwMap);

    }

    private void setup(HardwareMap hwMap) {

        // Set up servos
        horz = hwMap.get(Servo.class, "horz");
        horz.setPosition(0);
        vert = hwMap.get(Servo.class, "vert");
        vert.setPosition(MAX_POS / 2);
        rollTop = hwMap.get(CRServo.class, "rollTop");
        rollBottom = hwMap.get(CRServo.class, "rollBottom");

    }

    public void toggleSpeed(boolean button) {

        if (button) speed = FAST_SPEED;
        else speed = SLOW_SPEED;

    }

    public void horzMove(boolean cw, boolean ccw) {

        print("horz", horz.getPosition());
        if (cw) horz.setPosition(horz.getPosition() - speed);
        if (ccw) horz.setPosition(horz.getPosition() + speed);

    }

    public void vertMove(boolean up, boolean down) {

        print("vert", vert.getPosition());
        if (up) {

            if (vert.getPosition() < MAX_POS) vert.setPosition(vert.getPosition() + speed * VERT_MULTIPLIER);
            else vert.setPosition(MAX_POS);

        }
        if (down) vert.setPosition(vert.getPosition() - speed * VERT_MULTIPLIER);

    }

    public void roll(boolean out, boolean in) {

        if (out) {

            rollTop.setPower(ROLL_POW);
            rollBottom.setPower(ROLL_POW);

        } else if (in) {

            rollTop.setPower(-ROLL_POW);
            rollBottom.setPower(-ROLL_POW);

        } else {

            rollTop.setPower(0);
            rollBottom.setPower(0);

        }

    }

}
