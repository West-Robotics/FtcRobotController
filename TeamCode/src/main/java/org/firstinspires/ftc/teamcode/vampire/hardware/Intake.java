package org.firstinspires.ftc.teamcode.vampire.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.hardware.BaseHardware;

public class Intake extends BaseHardware {

    // Motor and motor power
    private DcMotor intakeMotor;
    private static final double INTAKE_POWER = 1;
    private static final double OUTTAKE_POWER = 0.7;

    // Servo
    private Servo flag;
    private static final double POS_UP = 0;
    private static final double POS_DOWN = 0.6;

    // For distance sensor
    private static final double DISTANCE_THRESHOLD = 1.4;
    private ColorRangeSensor CRSensor;
    private boolean isOverride = false;

    // Teleop constructor
    public Intake(OpMode opMode, HardwareMap hwMap) {

        super(opMode);
        setup(hwMap);

    }

    // Autonomous constructor
    public Intake(LinearOpMode opMode, HardwareMap hwMap) {

        super(opMode);
        setup(hwMap);

    }

    private void setup(HardwareMap hwMap) {

        // Set up motors
        intakeMotor = hwMap.get(DcMotor.class, "intake");
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor.setPower(0);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Set up servo
        flag = hwMap.get(Servo.class, "flag");
        flag.setPosition(POS_DOWN);

        // Set up distance sensor
        //distanceSensor = hwMap.get(DistanceSensor.class, "distance");
        CRSensor = hwMap.get(ColorRangeSensor.class, "CR");

    }

    public void intake(boolean intake, boolean reverse) {

        // Control intake
        print("Distance", CRSensor.getDistance(DistanceUnit.INCH));
        if (intake && (!isFreight() || isOverride)) intake();
        else if (reverse) reverse();
        else stop();

        // Raise the flag if block is in
        if (isFreight()) flag.setPosition(POS_UP);
        else flag.setPosition(POS_DOWN);

    }

    public boolean isFreight() {

        return CRSensor.getDistance(DistanceUnit.INCH) < DISTANCE_THRESHOLD;

    }

    public void intake() {

        intakeMotor.setPower(-INTAKE_POWER);

    }

    public void reverse() {

        intakeMotor.setPower(OUTTAKE_POWER);

    }

    public void toggleOverride(boolean button) {

        if (button) isOverride = !isOverride;

    }

    public void freightStop(double seconds) {

        new Thread() {

            @Override
            public void run() {

                ElapsedTime runtime = new ElapsedTime();
                runtime.reset();
                while (runtime.seconds() < seconds)
                    if (isFreight()) {

                        linearOpMode.sleep(500);
                        Intake.this.stop();

                    }

            }

        }.start();

    }

    public void stop() {

        intakeMotor.setPower(0);

    }

}
