package org.firstinspires.ftc.teamcode.vampire.hardware;

import android.text.method.Touch;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.PIDController;
import org.firstinspires.ftc.teamcode.hardware.BaseHardware;

public class Arm extends BaseHardware {

    // Objects
    private DcMotor liftMotor;
    private TouchSensor armTouch;

    // Constants
    private static final double LIFT_POWER = 1;
    private static final double LOWER_POWER = 0.75;
    private static final int[] ARM_STAGES = { -70, 500, 1500, 2000 };

    // Mutable variables
    private int stage = 0;
    private boolean isAuto = false;

    // PID
    private PIDController pidArm = new PIDController(0.005, 0, 0.004);

    // Teleop constructor
    public Arm(OpMode opMode, HardwareMap hwMap) {

        super(opMode);
        setup(hwMap);

    }

    // Autonomous constructor
    public Arm(LinearOpMode opMode, HardwareMap hwMap) {

        super(opMode);
        setup(hwMap);

    }

    private void setup(HardwareMap hwMap) {

        // Set up PID
        pidArm.setInputRange(0, ARM_STAGES[3]);
        pidArm.setOutputRange(0, LIFT_POWER);
        pidArm.setTolerance(1);

        // Set up lift motor
        liftMotor = hwMap.get(DcMotor.class, "lift");
        liftMotor.setDirection(DcMotor.Direction.REVERSE);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setPower(0);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Set up touch sensors
        armTouch = hwMap.get(TouchSensor.class, "armTouch");

    }

    public void toggleAuto(boolean button) {

        if (button) isAuto = !isAuto;

    }

    public void lift(boolean up, boolean down) {

        print("Position Arm", liftMotor.getCurrentPosition());

        // Manually move arm
        if (!isAuto) {

            if (armTouch.isPressed() && !up) {

                liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                liftMotor.setPower(0);

            } else {

                liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                if (up) liftMotor.setPower(LIFT_POWER);
                else if (down) liftMotor.setPower(-LOWER_POWER);
                else liftMotor.setPower(0);

            }

        }

    }

    public void setLift(int s) {

        stage = s;
        pidArm.reset();
        pidArm.enable();
        pidArm.setSetpoint(ARM_STAGES[stage]);
        pidArm.setTolerance(1);

        do {

            // Telemetry
            print("Position Arm", liftMotor.getCurrentPosition());
            linearOpMode.telemetry.update();

            // Power the motors
            liftMotor.setPower(pidArm.performPID(liftMotor.getCurrentPosition()));

        } while (!pidArm.onTarget() ||
                (stage == 0 && !armTouch.isPressed()));

        if (stage == 0) {

            liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        }

    }

    public void changeStage(boolean up, boolean down) {

        // Print arm info
        print("Touch Arm", armTouch.isPressed());
        print("Stage", stage);
        print("Is Auto", isAuto);

        // Print PID info
        print("P", pidArm.getP());
        print("I", pidArm.getI());
        print("D", pidArm.getD());

        if (isAuto) {

            // Move up and down stage
            if (up && stage != 3) stage++;
            if (down && stage != 0) stage--;

            // Set PID
            pidArm.setSetpoint(ARM_STAGES[stage]);

            // If lift is touching touch sensor
            if (armTouch.isPressed() && stage == 0) {

                pidArm.disable();
                liftMotor.setPower(0);
                liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            } else {

                pidArm.enable();
                liftMotor.setPower(pidArm.performPID(liftMotor.getCurrentPosition()));
                liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            }

        }

    }

}