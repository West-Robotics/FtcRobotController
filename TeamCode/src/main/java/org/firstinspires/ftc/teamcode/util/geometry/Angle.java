package org.firstinspires.ftc.teamcode.util.geometry;

public class Angle {
    // radians
    private double theta = 0.0;

    public Angle(double theta) {
        this.theta = wrap(theta);
    }
    public Angle() {
        this.theta = 0;
    }

    public double getAng() {
        return wrap(theta);
    }

    public static double wrap(double d) {
        return d % 2*Math.PI;
    }

    public Angle add(Angle a) {
        return new Angle(wrap(getAng() + a.getAng()));
    }
    public Angle sub(Angle a) {
        return new Angle(wrap(getAng() - a.getAng()));
    }
}
