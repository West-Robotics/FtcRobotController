package org.firstinspires.ftc.teamcode.node;

import androidx.annotation.NonNull;

import java.lang.reflect.Array;
import java.text.SimpleDateFormat;
import java.time.ZonedDateTime;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Date;
import java.util.HashMap;
import java.util.List;

public class NodeScheduler {
    // TODO: finish logging, add time stamps
    // TODO: add special case if a node does not subscribe to anything
    // TODO: add error handling so you don't get obliterated by NPEs
    // TODO: add type handling so you don't have to explicitly cast every get
    // INFO: why node-based programming? it's deterministic (more predictable robot actions), highly
    // modular, easy for the programmer to use, functional-style, and the structure makes it trivial
    // to achieve thorough logging, which gives us the capability to perform replays of an opmode run.
    // also atomic (kind of) and safe, the only thing a node can do is publish and receive data
    // and whatever it does in its loop
    // style points for being ROS-inspired

    // ARCH: a robot has multiple subsystems or services that are running concurrently: running the
    // drivetrain, controlling a lift, visual localization, filtering data, calculating odo, etc.
    // A node can do something when it starts, when it ends, and on each loop iteration, and
    // publishes and subscribes to data provided by other nodes. Every loop, every node will publish
    // data to the node scheduler, which will hold every piece of data and pass to each node data
    // from their respective subscriptions.

    // Reserved topics: default, d_t

    List<Node> nodes;
    HashMap<String, Object> data = new HashMap<String, Object>();
    HashMap<Node, List<String>> subscriptions = new HashMap<Node, List<String>>();

    Datalogger datalog;
    List<Datalogger.LoggableField> fields = new ArrayList<Datalogger.LoggableField>();
    List<String> topics = new ArrayList<String>();
    boolean logging = false;
    boolean replay = false;

    public NodeScheduler(boolean log, Node... nodeList) {
        nodes = Arrays.asList(nodeList);
        data.put("default", null);
        logging = log;
    }

    public void init() {
        for (Node n : nodes) {
            n.init();
        }
        for (Node n : nodes) {
            subscriptions.put(n, n.getSubscriptions());
        }
        if (logging) {
            // FIX: data has no entries at the beginning
//            publish();
//            exchange();
//            update();
            // publish and exchange data
            publish();
//            exchange();
            // loop through node actions
//            for (Node n : nodes) {
//                n.loop();
//            }
            topics.addAll(new ArrayList<>(data.keySet()));
            for (String topic : topics) {
                fields.add(new Datalogger.LoggableField(topic));
            }
            datalog = new Datalogger.Builder()
                    .setFilename(String.valueOf(new SimpleDateFormat("yyyy-MM-dd-HH-mm-ss").format(new Date())))
//                    .setFilename(String.valueOf(new SimpleDateFormat("yyyy-MM-dd-HH-mm-ss").format(ZonedDateTime.now()))) <-- this causes robot restart
                    .setAutoTimestamp(Datalogger.AutoTimestamp.DECIMAL_SECONDS)
                    .setFields(fields)
                    .build();
        }
    }
    public void end() {
        for (Node n : nodes) {
            n.end();
        }
    }
    public void update() {
        // publish and exchange data
        publish();
        exchange();
        // loop through node actions
        for (Node n : nodes) {
            n.loop();
        }
        if (logging) {
            for (int i = 0; i < topics.size(); i++) {
                // +1 to fields because timestamp is added as the first header
                if (data.get(topics.get(i)) instanceof Double) {
                    datalog.fields[i+1].set((Double) data.get(topics.get(i)));
                } else if (data.get(topics.get(i)) instanceof Boolean) {
                    datalog.fields[i+1].set((Boolean) data.get(topics.get(i)));
                } else if (data.get(topics.get(i)) instanceof Integer) {
                    datalog.fields[i+1].set((Integer) data.get(topics.get(i)));
                } else if (data.get(topics.get(i)) instanceof String) {
                    datalog.fields[i+1].set((String) data.get(topics.get(i)));
                } else if (data.get(topics.get(i)) instanceof Float) {
                    datalog.fields[i+1].set((Float) data.get(topics.get(i)));
                }

            }
            datalog.writeLine();
        }
    }
    // for anything external (not a node) that needs data, e.g. opmode telemetry
    public HashMap<String, Object> extract(@NonNull String... queries) {
        HashMap<String, Object> d = new HashMap<>();
        for (String q : queries) {
            d.put(q, data.get(q));
        }
        return d;
    }

    private void publish() {
        for (Node n : nodes) {
            data.putAll(n.publish());
        }
    }
    private void exchange() {
        for (Node n : nodes) {
            HashMap<String, Object> message = new HashMap<>();
            for (String s : subscriptions.get(n)) {
                message.put(s, data.get(s));
            }
            n.receive(message);
        }
    }
}
