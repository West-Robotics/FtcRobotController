package org.firstinspires.ftc.teamcode.vampire.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.*;

import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.hardware.BaseHardware;

public class Intake extends BaseHardware {

    // Motor and motor power
    private DcMotor intakeMotor;
    private static final double INTAKE_POWER = 0.8;
    private static final double INTAKE_SLOW = 0.6;
    private static final double OUTTAKE_POWER = 0.4;
    public static final double OUTTAKE_TIME = 1.25;

    // Servo
    private Servo flag;
    private static final double POS_UP = 0;
    private static final double POS_DOWN = 0.6;

    // For distance sensor
    private int counter = 0;
    private ElapsedTime runtime = new ElapsedTime();
    private static final double WAIT_TIME = 1;
    private static final double DISTANCE_THRESHOLD = 2;
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
        CRSensor = hwMap.get(ColorRangeSensor.class, "CR");
        runtime.reset();

    }

    public void intake(boolean intake, boolean reverse) {

        // Control intake
        print("Distance", CRSensor.getDistance(DistanceUnit.INCH));
        if ((intake && ((!isFreight() && runtime.seconds() > WAIT_TIME)) || isOverride)) intake();
        else if (reverse) reverse();
        else stop();

        // Raise the flag if block is in
        if (isFreight()) {

            counter++;
            flag.setPosition(POS_UP);

        }
        else {

            counter = 0;
            flag.setPosition(POS_DOWN);

        }

        // First time the freight is detected
        if (counter == 1) runtime.reset();

    }

    public boolean isFreight() {

        return CRSensor.getDistance(DistanceUnit.INCH) < DISTANCE_THRESHOLD;

    }

    public void intake() {

        intakeMotor.setPower(-INTAKE_POWER);

    }

    public void intakeSlow() {

        intakeMotor.setPower(-INTAKE_SLOW);

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
