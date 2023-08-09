package org.firstinspires.ftc.teamcode.squirmy;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.SampleTankDrive;
import org.firstinspires.ftc.teamcode.node.Node;

import java.util.HashMap;

public class DrivetrainNode extends Node {
//    SampleMecanumDrive drive;
    SampleTankDrive drive;
    public DrivetrainNode(HardwareMap hardwareMap) {
        drive = new SampleTankDrive(hardwareMap);
        subscriptions.add("gamepad_left_x");
        subscriptions.add("gamepad_left_y");
        subscriptions.add("gamepad_right_x");
    }
    @Override
    public void end() {
        drive.setMotorPowers(0.0, 0.0);
    }
    @Override
    public void loop() {
        drive.setWeightedDrivePower(new Pose2d(-(float) data.get("gamepad_left_y"),
                                               0,
                                               (float) data.get("gamepad_right_x")));
    }
    // override in child
    public HashMap<String, Object> publish() { return new HashMap<String, Object>(); }
}