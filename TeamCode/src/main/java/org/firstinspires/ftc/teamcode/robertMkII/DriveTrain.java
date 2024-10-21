package org.firstinspires.ftc.teamcode.robertMkII;

// for controlling the robot

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class DriveTrain {

    private DcMotor leftFront;
    private DcMotor leftBack;
    private DcMotor rightFront;
    private DcMotor rightBack;
    private DcMotor armExtender;
    private DcMotor armRotater;
    private CRServo leftHandIntake;
    private CRServo rightHandIntake;
    private Servo wrist;
    private HandPosition handPos;
    private HandPosition lastHandPos;

    public DriveTrain(HardwareMap hardwareMap) /* INIT */ {
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        armExtender = hardwareMap.get(DcMotor.class, "armExtender");
        armRotater = hardwareMap.get(DcMotor.class, "armRotater");

        leftHandIntake = hardwareMap.get(CRServo.class, "leftHandIntake");
        rightHandIntake = hardwareMap.get(CRServo.class, "rightHandIntake");
        handRotater = hardwareMap.get(Servo.class, "handRotater");

        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightHandIntake.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        armExtender.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        armRotater.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        armExtender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armRotater.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        armExtender.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armRotater.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        handPos = HandPosition.LEVEL;
        lastHandPos = HandPosition.LEVEL;
    }

    public void tankDrive(double straightSpeed, double strafeSpeed, double rotationSpeed) {

        leftFront.setPower(straightSpeed - rotationSpeed - strafeSpeed);
        leftBack.setPower(straightSpeed - rotationSpeed + strafeSpeed);
        rightFront.setPower(straightSpeed + rotationSpeed + strafeSpeed);
        rightBack.setPower(straightSpeed + rotationSpeed - strafeSpeed);

    }

    public void manipulateArm(double extendSpeed, double rotateSpeed) {
        // motor uses 537.7 ppr
        // diameter of pulley gear is 120mm
        // slide extension amount is 107 cm
        // full rotations until extended is 59
        // worm gear is 28:1 or 14:0.5
        double extenderRevCount = armExtender.getCurrentPosition() / 537.7;
        double rotationCount = armRotater.getCurrentPosition() / 537.7;
        if ((extenderRevCount < 89 && extendSpeed > 0) || (extenderRevCount > 2 && extendSpeed < 0)) {
            armExtender.setPower(extendSpeed);
        } else { armExtender.setPower(0); }
        if ((rotationCount < 13.5 && rotateSpeed > 0) || (rotationCount > 2 && rotateSpeed < 0)) {
            armRotater.setPower(rotateSpeed);
        } else { armRotater.setPower(0); }

    }

    enum HandPosition {
        DUMP,
        LEVEL
    }
    public void moveHand(boolean toggleHandPos, double handIntakeSpeed) {
        leftHandIntake.setPower(handIntakeSpeed);
        rightHandIntake.setPower(handIntakeSpeed);

        lastHandPos = handPos;
        handPos = (toggleHandPos && handPos==HandPosition.LEVEL) ? HandPosition.DUMP : HandPosition.LEVEL;
        if (handPos==HandPosition.DUMP && lastHandPos!=HandPosition.DUMP) {
            wrist.setPosition(0);
        } else if (handPos==HandPosition.LEVEL && lastHandPos!=HandPosition.LEVEL) {
            wrist.setPosition(1);
        }
    }
}
