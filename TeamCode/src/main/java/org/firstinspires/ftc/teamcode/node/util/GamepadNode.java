package org.firstinspires.ftc.teamcode.node.util;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Controller;
import org.firstinspires.ftc.teamcode.node.Node;

import java.util.HashMap;
import java.util.List;

public class GamepadNode extends Node {
    private Gamepad gamepad;

    public GamepadNode(Gamepad g) {
        gamepad = g;
    }

    @Override
    public HashMap<String, Object> publish() {
        HashMap<String, Object> message = new HashMap<String, Object>();
        message.put("gamepad_left_x", gamepad.left_stick_x);
        message.put("gamepad_left_y", gamepad.left_stick_y);
        message.put("gamepad_right_x", gamepad.right_stick_x);
        message.put("gamepad_right_y", gamepad.right_stick_y);
        message.put("gamepad_left_x", gamepad.a);
        message.put("gamepad_left_x", gamepad.b);
        message.put("gamepad_left_x", gamepad.x);
        message.put("gamepad_left_x", gamepad.y);
        message.put("gamepad_left_x", gamepad.dpad_up);
        message.put("gamepad_left_x", gamepad.dpad_down);
        message.put("gamepad_left_x", gamepad.dpad_left);
        message.put("gamepad_left_x", gamepad.dpad_right);
        message.put("gamepad_left_x", gamepad.left_trigger);
        message.put("gamepad_left_x", gamepad.right_trigger);
        message.put("gamepad_left_x", gamepad.left_bumper);
        message.put("gamepad_left_x", gamepad.right_bumper);
        return message;
    }

}
