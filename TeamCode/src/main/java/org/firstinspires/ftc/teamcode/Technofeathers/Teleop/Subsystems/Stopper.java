package org.firstinspires.ftc.teamcode.Technofeathers.Teleop.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Stopper {
    public Servo stopper;
    public boolean stopperDown = false;
    public void setUpStopper(HardwareMap hardwareMap) {
        stopper = hardwareMap.get(Servo.class, "stopper");
    }
    public void down() {
        if (!stopperDown) {
            stopper.setPosition(0.9);
            stopperDown = true;
        }
    }

    public void up() {
        if (stopperDown) {
            stopper.setPosition(0.37);
            stopperDown = false;
        }
    }
}
