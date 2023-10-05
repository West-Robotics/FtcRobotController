package org.firstinspires.ftc.teamcode.util.geometry;

public class Vector2d {
    double u;
    double v;

    public Vector2d(double u, double v) {
        this.u = u;
        this.v = v;
    }
    public Vector2d() {
        this.u = 0;
        this.v = 0;
    }

    public double getMag() {
        // can you have an integer exponent on a double and not lose info?
        return Math.sqrt(Math.pow(u, 2) + Math.pow(v, 2));
    }

    public Angle getAng() {
        return new Angle(Math.atan2(v, u));
    }

    public Vector2d scale(double s) {
        return new Vector2d(u*s, v*s);
    }
    public Vector2d normalize() { return new Vector2d(u/getMag(), v/getMag()); }

    public Vector2d add(Vector2d vector) { return new Vector2d(u + vector.u, v + vector.v); }
    public Vector2d sub(Vector2d vector) { return new Vector2d(u - vector.u, v - vector.v); }

    public Point toPoint() {
        return new Point(u, v);
    }
}
