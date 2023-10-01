package org.firstinspires.ftc.teamcode.util.geometry;

public class Point {
    public double x = 0.0;
    public double y = 0.0;

    public Point(double x, double y) {
        this.x = x;
        this.y = y;
    }
    public Point() {
        this.x = 0;
        this.y = 0;
    }

    public double displacement(Point p) {
        return new Vector2d(p.x - x, p.y - y).getMag();
    }

    public Angle getOrigAng() {
        return new Angle(Math.atan2(y, x));
    }

    public Vector2d toVector() { return new Vector2d(x, y); }
}
