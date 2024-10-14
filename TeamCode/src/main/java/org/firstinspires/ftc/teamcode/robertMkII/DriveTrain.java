package org.firstinspires.ftc.teamcode.robertMkII;

// for controlling the robot

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DriveTrain {

    private DcMotor leftFront;
    private DcMotor leftBack;
    private DcMotor rightFront;
    private DcMotor rightBack;
    private DcMotor armExtender;
    private DcMotor armRotater;
    public DriveTrain(HardwareMap hardwareMap) {
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        armExtender = hardwareMap.get(DcMotor.class, "armExtender");
        armRotater = hardwareMap.get(DcMotor.class, "armRotater");

        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);

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

        armExtender.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armRotater.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void tankDrive(double straightSpeed, double strafeSpeed, double rotationSpeed) {

        leftFront.setPower(straightSpeed - rotationSpeed - strafeSpeed);
        leftBack.setPower(straightSpeed - rotationSpeed + strafeSpeed);
        rightFront.setPower(straightSpeed + rotationSpeed + strafeSpeed);
        rightBack.setPower(straightSpeed + rotationSpeed - strafeSpeed);

    }

    public void manipulateArm(double extendSpeed, double rotateSpeed) {

        armExtender.setPower(extendSpeed);
        armRotater.setPower(rotateSpeed);

    }
}
