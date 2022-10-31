package org.firstinspires.ftc.teamcode.fifthWheel;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Gyro;
import org.firstinspires.ftc.teamcode.Controller;

import org.firstinspires.ftc.teamcode.fifthWheel.hardware.DriveFifthWheel;
import org.firstinspires.ftc.teamcode.fifthWheel.hardware.intake.Intake;
import org.firstinspires.ftc.teamcode.fifthWheel.hardware.outtake.DRCB;

import java.util.ArrayList;

@TeleOp(name = "FifthWheel TeleOp")
public class TeleopFifthWheel extends OpMode {

    private DriveFifthWheel drive;
    private Intake intake;
    private DRCB drcb;
    private Gyro gyro;
    private Controller controller;

    private int level = 0;

    private static final int STORE_NUM = 8;
    private ArrayList<Double> x = new ArrayList<>();
    private ArrayList<Double> y = new ArrayList<>();
    private ArrayList<Double> turn = new ArrayList<>();

    @Override
    public void init() {
        drive = new DriveFifthWheel(this, hardwareMap);
        intake = new Intake(hardwareMap);
        intake.setLevel(0);
        drcb = new DRCB(hardwareMap);
        gyro = new Gyro(hardwareMap, false);
        controller = new Controller(gamepad1);
        drive.togglePOV(true);
        drive.enableSquaredInputs();
        gyro.reset();
    }

    @Override
    public void loop() {
        telemetry.addData("gyro", gyro.getAngleDegrees());
        telemetry.addData("leftPos", intake.leftPos());
        telemetry.addData("rightPos", intake.rightPos());
        telemetry.addData("left drcb", drcb.getCurrentLeftTicks());
        telemetry.addData("right drcb", drcb.getCurrentRightTicks());
        telemetry.update();
        controller.update();

        x.add(controller.left_stick_x);
        y.add(controller.left_stick_y);
        turn.add(controller.right_stick_x);

        // Remove
        if (x.size() > STORE_NUM) x.remove(0);
        if (y.size() > STORE_NUM) y.remove(0);
        if (turn.size() > STORE_NUM) turn.remove(0);

        double avgX = 0;
        for (int i = 0; i < x.size(); i++) avgX += x.get(i);
        avgX /= x.size();

        double avgY = 0;
        for (int i = 0; i < y.size(); i++) avgY += y.get(i);
        avgY /= y.size();

        double avgTurn = 0;
        for (int i = 0; i < turn.size(); i++) avgTurn += turn.get(i);
        avgTurn /= turn.size();

        if (controller.right_trigger > 0) {
            drive.drive(avgX/2, avgY/2, avgTurn/2);
        } else {
            drive.drive(avgX, avgY, avgTurn);
        }

        if (controller.AOnce()) {
            intake.open();
        } else if (controller.BOnce()) {
            intake.close();
        }

        if (controller.dpadDown()) {
//            level = 0;
            drcb.leftMotor.setPower(-0.01);
            drcb.rightMotor.setPower(-0.01);
            // drcb.setLevel(level);
            // intake.setLevel();
        } else if (controller.dpadUp()) {
//            level = 4;
            drcb.leftMotor.setPower(0.57);
            drcb.rightMotor.setPower(0.57);
            // drcb.setLevel(level);
            // intake.setLevel(level);
        } else if (controller.X()) {
            drcb.leftMotor.setPower(-0.3);
            drcb.rightMotor.setPower(-0.3);
        } else {
            drcb.leftMotor.setPower(0.17);
            drcb.rightMotor.setPower(0.17);
        }

        if (controller.dpadRightOnce()) {
//            intake.lower();
            if (level > 0) {
                level -= 1;
                intake.setLevel(level);
            }
            // drcb.setLevel(level);
            // intake.setLevel();
        } else if (controller.dpadLeftOnce()) {
//            intake.raise();
            if (level < 3) {
                level += 1;
                intake.setLevel(level);
            }
            // drcb.setLevel(level);
            // intake.setLevel();
        }

//        if (controller.YOnce()) {
//            drcb.setLevel(2);
//        } else if (controller.XOnce()) {
//            drcb.setLevel(0);
//        }

//        if (controller.leftBumperOnce()) {
//            intake.rightDecrease();
//        } else if (controller.rightBumperOnce()) {
//            intake.rightIncrease();
//        }

        // if (controller.dpadDown()) {
        //     drcb.leftMotor.setPower(-0.2);
        // } else if (controller.dpadUp()) {
        //     drcb.leftMotor.setPower(0.75);
        // } else {
        //     drcb.leftMotor.setPower(0);
        // }
        // if (controller.A()) {
        //     intake.open();
        // }
        // if (controller.B()) {
        //     intake.close();
        // }
        // if (controller.leftBumperOnce()) {
        //     intake.lower();
        // }
        // if (controller.rightBumperOnce()) {
        //     intake.raise();
        // }
        // if (controller.dpadLeftOnce()) {
        //     intake.gripDecrease();
        // }
        // if (controller.dpadRightOnce()) {
        //     intake.gripIncrease();
        // }
        // if (controller.leftBumperOnce()) {
        //     intake.flipDecrease();
        // }
        // if (controller.rightBumperOnce()) {
        //     intake.flipIncrease();
        // }
    }
}