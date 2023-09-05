package org.firstinspires.ftc.teamcode.squirmy;

//import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.node.*;
import org.firstinspires.ftc.teamcode.node.util.GamepadNode;
import org.firstinspires.ftc.teamcode.node.util.LoopTimerNode;

import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Squirmy TeleOp")
public class Teleop extends OpMode {
    NodeScheduler nodeScheduler;
    ElapsedTime timer = new ElapsedTime();

    @Override
    public void init() {
//        PhotonCore.enable();
        nodeScheduler = new NodeScheduler(true, new LoopTimerNode(), new DrivetrainNode(hardwareMap), new GamepadNode(gamepad1));
        nodeScheduler.init();
    }

    @Override
    public void loop() {
        nodeScheduler.update();
        telemetry.addData("d_t", nodeScheduler.extract("d_t").get("d_t"));
        telemetry.addData("opmode loop time", timer.milliseconds());
        timer.reset();
        telemetry.update();
    }
}
