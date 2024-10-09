package org.firstinspires.ftc.teamcode.util.localization;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.hardware.BaseHardware;
//import org.firstinspires.ftc.teamcode.utils.localization.Point;

//import com.acmerobotics.roadrunner.drive.Drive;
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.acmerobotics.roadrunner.geometry.Vector2d;

import java.util.ArrayList;
import java.util.Collections;
import java.lang.Math;

enum State {
    SEARCH, HORIZ, VERT, DONE
}

public class PointMapLoc { //extends BaseHardware {
    /*
    public Drive drive;
    protected ModernRoboticsI2cRangeSensor f;
    protected ModernRoboticsI2cRangeSensor l;
    protected ModernRoboticsI2cRangeSensor b;
    protected ModernRoboticsI2cRangeSensor r;
    protected Servo rotator;

    protected ArrayList<Point> pointMap = new ArrayList<Point>();
    int mapSize = 0;

    // boolean dir = true;
    int dir = 1;
    double pos = 0.00;
    double step = 0.05;
    // protected double ticksPerRev = 100;
    // protected double quarterTicks = ticksPerRev / 4;
    protected double distFromCenter = 2;

    public PointMapLoc(LinearOpMode opMode, HardwareMap hwMap, Drive d) {
        super(opMode);
        init(hwMap, d);
    }

    public PointMapLoc(OpMode opMode, HardwareMap hwMap, Drive d) {
        super(opMode);
        init(hwMap, d);
    }

    protected void init(HardwareMap hwMap, Drive d) {
        drive = d;
        f = hwMap.get(ModernRoboticsI2cRangeSensor.class, "f");
        l = hwMap.get(ModernRoboticsI2cRangeSensor.class, "l");
        b = hwMap.get(ModernRoboticsI2cRangeSensor.class, "b");
        r = hwMap.get(ModernRoboticsI2cRangeSensor.class, "r");
        rotator = hwMap.get(Servo.class, "rotator");
    }

    public void scan() {
        ArrayList<Point> q1 = new ArrayList<>();
        ArrayList<Point> q2 = new ArrayList<>();
        ArrayList<Point> q3 = new ArrayList<>();
        ArrayList<Point> q4 = new ArrayList<>();
        double d1;
        double d2;
        double d3;
        double d4;
        double x;
        double y;
        double theta;
        Pose2d poseAtCapture;
        double target;

        if (dir == 1) {
            target = 0.3;
        } else {
            target = 0;
        }
        while (pos != target) {
            d1 = f.getDistance(DistanceUnit.INCH);
            d2 = l.getDistance(DistanceUnit.INCH);
            d3 = b.getDistance(DistanceUnit.INCH);
            d4 = r.getDistance(DistanceUnit.INCH);
            theta = drive.getExternalHeading() + posToRadians(pos);
            poseAtCapture = drive.getPoseEstimate();

            // roadrunner-style coords are used:
            // theta at 0 is on the positive x-axis
            x = d1 * Math.cos(theta);
            y = d1 * Math.sin(theta);
            q1.add(new Point(x, y, theta, poseAtCapture));
            x = d2 * Math.cos(theta + Math.PI/2);
            y = d2 * Math.sin(theta + Math.PI/2);
            q2.add(new Point(x, y, theta + Math.PI/2, poseAtCapture));
            x = d3 * Math.cos(theta + Math.PI);
            y = d3 * Math.sin(theta + Math.PI);
            q3.add(new Point(x, y, theta + Math.PI, poseAtCapture));
            x = d4 * Math.cos(theta + 3*Math.PI/2);
            y = d4 * Math.sin(theta + 3*Math.PI/2);
            q4.add(new Point(x, y, theta + 3*Math.PI/2, poseAtCapture));
            pos += step*dir;
        }
        // rotator.setPower(0);
        if (dir < 0) {
            Collections.reverse(q1);
            Collections.reverse(q2);
            Collections.reverse(q3);
            Collections.reverse(q4);
        }
        dir = -dir;
        pointMap.addAll(q1);
        pointMap.addAll(q2);
        pointMap.addAll(q3);
        pointMap.addAll(q4);
        mapSize = pointMap.size();
    }

    public void correctDists() {
        for (int i = 0; i < pointMap.size(); ++i) {
            pointMap.get(i).x = distFromCenter*Math.cos(pointMap.get(i).theta);
            pointMap.get(i).y = distFromCenter*Math.sin(pointMap.get(i).theta);
        }
    }

    public void undistort() {
        for (int i = 0; i < mapSize; ++i) {
            pointMap.get(i).x = pointMap.get(i).x - (pointMap.get(pointMap.size()-1).poseAtCapture.getX() - pointMap.get(i).x);
            pointMap.get(i).y = pointMap.get(i).y - (pointMap.get(pointMap.size()-1).poseAtCapture.getY() - pointMap.get(i).y);
        }
    }

    // // TODO: use an actual filter instead of rounding, probably kalman or something
    // public void filter() {
    //     for (int i = 0; i < mapSize; ++i) {
    //         pointMap.get(i).x = Math.round(pointMap.get(i).x);
    //         pointMap.get(i).y = Math.round(pointMap.get(i).y);
    //     }
    // }

    public void correctToField() {
        for (int i = 0; i < mapSize; ++i) {
            if (pointMap.get(i).x > 0) {
                pointMap.get(i).x = 144 - pointMap.get(i).x; // field is 144x144 inches
            } else if (pointMap.get(i).x < 0) {
                pointMap.get(i).x = -pointMap.get(i).x;
            }
            if (pointMap.get(i).y > 0) {
                pointMap.get(i).y = 144 - pointMap.get(i).y;
            } else if (pointMap.get(i).y < 0) {
                pointMap.get(i).y = -pointMap.get(i).y;
            }
        }
    }

    public void process() {
        correctDists();
        undistort();
        // filter();
        correctToField();
    }

    public Vector2d getPos() {
        double x = 0;
        double y = 0;
        Point first = new Point(0, 0, 0, new Pose2d());
        State state = State.SEARCH;
        int i = 0;
        boolean wall = true;
        boolean horiz = false;
        boolean vert = false;
        while (state != State.DONE) {
            switch (state) {
                case SEARCH:
                    if ((horiz && vert) || (i >= pointMap.size()-1)) {
                        state = State.DONE;
                        break;
                    }
                    if (Math.abs(pointMap.get(i).y - pointMap.get(i+1).y) < 0.5) {
                        state = State.HORIZ;
                        i += 1;
                        first = pointMap.get(i);
                        break;
                    }
                    if (Math.abs(pointMap.get(i).x - pointMap.get(i+1).x) < 0.5) {
                        state = State.VERT;
                        i += 1;
                        first = pointMap.get(i);
                        break;
                    }
                    i += 1;
                    break;
                case HORIZ:
                    wall = true;
                    for (int count = 2; count < 4; ++count) {
                        if (!(Math.abs(first.y - pointMap.get(i).y) < 0.5)) {
                            wall = false;
                            count = 0;
                            break;
                        }
                        i += 1;
                    }
                    if (wall) {
                        y = pointMap.get(i).y;
                        horiz = true;
                    }
                    break;
                case VERT:
                    wall = true;
                    for (int count = 2; count < 4; ++count) {
                        if (!(Math.abs(first.x - pointMap.get(i).x) < 0.5)) {
                            wall = false;
                            count = 0;
                            break;
                        }
                        i += 1;
                    }
                    if (wall) {
                        x = pointMap.get(i).x;
                        vert = true;
                    }
                    break;
            }
        }
        return new Vector2d(x, y);
    }

    public Vector2d update() {
        scan();
        process();
        return getPos();
    }

    protected double posToRadians(double p) {
        return Math.PI*p;
    }

    public ArrayList<Point> getMap() {
        return pointMap;
    }

     */
}
