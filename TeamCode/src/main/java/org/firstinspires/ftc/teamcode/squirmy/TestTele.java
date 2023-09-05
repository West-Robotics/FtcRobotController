package org.firstinspires.ftc.teamcode.squirmy;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.List;

@TeleOp(name = "Squirmy TestTeleOp")
public class TestTele extends OpMode {
    private List<LynxModule> allHubs;

    double lasty = 0.0;
    double loopTime = 0.0;
    DcMotorEx leftFront;
    DcMotorEx leftRear;
    DcMotorEx rightRear;
    DcMotorEx rightFront;

    @Override
    public void init() {

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }

    @Override
    public void loop() {
        // try caching values
        if (lasty != -gamepad1.left_stick_y) {
            lasty = -gamepad1.left_stick_y;
            leftFront.setPower(lasty);
            leftRear.setPower(lasty);
            rightRear.setPower(lasty);
            rightFront.setPower(lasty);
        }
        double loop = System.nanoTime();
        telemetry.addData("hz", 1000000000 / (loop - loopTime));
        telemetry.addData("loop time", (loop - loopTime)/1000000);
        telemetry.addData("lf current", leftFront.getCurrentPosition());
        telemetry.addData("lr current", leftRear.getCurrentPosition());
        telemetry.addData("rr current", rightRear.getCurrentPosition());
        telemetry.addData("rf current", rightFront.getCurrentPosition());
        loopTime = loop;
        telemetry.update();
        for (LynxModule module : allHubs) {
            module.clearBulkCache();
        }
    }
}
