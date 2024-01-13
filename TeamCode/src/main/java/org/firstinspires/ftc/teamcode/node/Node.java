package org.firstinspires.ftc.teamcode.node;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
//use gitgub actions loosers (devops)
public class Node {
    // append to this list w/ subscriptions in child class
    protected List<String> subscriptions = new ArrayList<>(Arrays.asList("default"));
    protected HashMap<String, Object> data = new HashMap<String, Object>();
    // override in child
    public void init() { }
    // override in child
    public void end() { }
    // override in child
    public void loop() { }
    public List<String> getSubscriptions() { return subscriptions; }
    // override in child
    public HashMap<String, Object> publish() { return new HashMap<String, Object>(); }
    public void receive(HashMap<String, Object> d) { data = d; }
}