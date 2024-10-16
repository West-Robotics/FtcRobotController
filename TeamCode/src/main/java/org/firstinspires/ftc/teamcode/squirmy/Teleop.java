package org.firstinspires.ftc.teamcode.squirmy;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.node.*;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Squirmy TeleOp")
public class Teleop extends OpMode {

    NodeScheduler nodeScheduler = new NodeScheduler(false, new BulkReadNode(hardwareMap));
    ElapsedTime timer = new ElapsedTime();

    @Override
    public void init() {
        nodeScheduler.init();
    }

    @Override
    public void loop() {
        timer.reset();
        nodeScheduler.update();
        telemetry.addData("d_t", nodeScheduler.extract("d_t").get("d_t"));
        telemetry.addData("opmode loop time", timer.milliseconds());
        telemetry.update();
    }
}