package org.firstinspires.ftc.teamcode.Technofeathers.Teleop.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Pivot {
    public Servo pivot1;
    public boolean pivotReadyToDrop = false;
    public void setUpPivot(HardwareMap hardwareMap) {
        pivot1 = hardwareMap.get(Servo.class, "pivot");
    }
    public void out() {
        if (!pivotReadyToDrop) {
            pivot1.setPosition(0);
            pivotReadyToDrop = true;
        }
    }

    public void in() {
        if (pivotReadyToDrop) {
            pivot1.setPosition(1);
            pivotReadyToDrop = false;
        }
    }
}
