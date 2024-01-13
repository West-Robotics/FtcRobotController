package org.firstinspires.ftc.teamcode.Technofeathers;

public class TechnofeathersPDTest {
    private double setPoint;
    private double kp;

    public TechnofeathersPDTest(double kp) {
        this.kp = kp;
    }

    public void setDesiredPoint(double input) {
        setPoint = input;
    }

    public double update(double currentPoint) {
        return kp * (setPoint-currentPoint);
    }
}
