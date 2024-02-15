package org.firstinspires.ftc.teamcode.Technofeathers.Teleop.Subsystems;

//import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Pivot {
    public Servo pivot1;
    public Telemetry telemetry;
    public boolean pivotReadyToDrop = false;
    public void setUpPivot(HardwareMap hardwareMap, Telemetry telemetry) {
        pivot1 = hardwareMap.get(Servo.class, "pivot1");
        this.telemetry = telemetry;
    }
    /*
    public void out() {

    }

    public void in() {

    }

     */

    public void move() {
        if (!pivotReadyToDrop) {
            pivot1.setPosition(0);
            pivotReadyToDrop = true;
            telemetry.addLine("pivot out");
        }
        else {
            pivot1.setPosition(1);
            pivotReadyToDrop = false;
            telemetry.addLine("pivot in");
        }
    }
    public double getPosition() {
        return pivot1.getPosition();
    }
}
