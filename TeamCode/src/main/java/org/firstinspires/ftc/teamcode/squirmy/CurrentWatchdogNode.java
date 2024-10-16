package org.firstinspires.ftc.teamcode.squirmy;

import org.firstinspires.ftc.teamcode.node.Node;

import java.util.HashMap;
import java.util.List;

public class CurrentWatchdogNode extends Node {
    boolean danger = false;

    @Override
    public void init() {
        subscriptions.add("current1");
        subscriptions.add("current2");
        subscriptions.add("current3");
        subscriptions.add("current4");
    }

    @Override
    public void loop() {
        danger = ((int) data.get("current1") + (int) data.get("current2") + (int) data.get("current3") + (int) data.get("current4")) > 16;
    }

    @Override
    public HashMap<String, Object> publish() {
        HashMap<String, Object> message = new HashMap<String, Object>();
        message.put("current_danger", danger);
        return message;
    }

}