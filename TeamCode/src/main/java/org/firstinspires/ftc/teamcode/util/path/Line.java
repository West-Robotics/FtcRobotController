package org.firstinspires.ftc.teamcode.util.path;

import org.firstinspires.ftc.teamcode.util.geometry.Point;

public class Line extends PathSegment {
    Point a;
    Point b;

    public Line(Point a, Point b) {
        this.a = a;
        this.b = b;
    }

}
