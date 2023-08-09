package org.firstinspires.ftc.teamcode.node;

import android.os.Environment;
import androidx.annotation.NonNull;

import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.channels.AsynchronousFileChannel;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.nio.file.StandardOpenOption;
import java.text.SimpleDateFormat;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;

public class NodeScheduler {
    // TODO: finish logging, add time stamps
    // TODO: add special case if a node does not subscribe to anything
    // TODO: add error handling so you don't get obliterated by NPEs
    // TODO: add type handling so you don't have to explicitly cast every get
    // INFO: why node-based programming? it's deterministic (more predictable robot actions), highly
    // modular, easy for the programmer to use, functional-style, and the structure makes it trivial
    // to achieve thorough logging, which gives us the capability to perform exact replays of an
    // opmode run from a software control perspective.
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

    boolean logging = false;
    Path path;
    AsynchronousFileChannel asyncFileChannel;
    ByteBuffer buffer;
    List<String> topics;

    public NodeScheduler(boolean log, Node... nodeList) {
        nodes = Arrays.asList(nodeList);
        data.put("default", null);
        logging = log;
    }

    public void init() {
        if (logging) {
            path = Paths.get(Environment.getExternalStorageDirectory().getPath() + "/FIRST/" + new SimpleDateFormat("yyyy-MM-dd-HH-mm-ss") + ".csv");
            try {
                asyncFileChannel = AsynchronousFileChannel.open(path, StandardOpenOption.APPEND);
            } catch (IOException e) {
                e.printStackTrace();
            }
            buffer = ByteBuffer.allocate(1024);
            publish();
            exchange();
            topics.addAll(data.keySet());
            buffer.put(String.join(",", topics).getBytes());
        }
        for (Node n : nodes) {
            n.init();
        }
        for (Node n : nodes) {
            subscriptions.put(n, n.getSubscriptions());
        }
    }
    public void end() {
        for (Node n : nodes) {
            n.end();
        }
        if (logging) {
            try {
                asyncFileChannel.close();
            } catch (IOException e) {
                e.printStackTrace();
            }
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
            // TODO: async thread for logging operations
            // TODO: use file locks for data integrity
            // WARNING: currently there will probably be multiple attempted concurrent writes
            // so don't use logging right now
            buffer.clear();
            for (String topic : topics) {
                buffer.put((Byte) data.get(topic));
                buffer.put((byte) ',');
            }
            buffer.flip();
            try {
                asyncFileChannel.write(buffer, asyncFileChannel.size());
            } catch (IOException e) {
                e.printStackTrace();
            }
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
