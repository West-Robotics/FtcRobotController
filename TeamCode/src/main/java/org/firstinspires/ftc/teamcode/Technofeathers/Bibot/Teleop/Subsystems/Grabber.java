package org.firstinspires.ftc.teamcode.Technofeathers.Bibot.Teleop.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Grabber {
    public Servo grabber;
    public boolean grabbedPixels = false;

    public void setUpGrabber(HardwareMap hardwareMap) {
        grabber = hardwareMap.get(Servo.class, "grabber");
    }

    public void move() {
        if (!grabbedPixels) {
            grabber.setPosition(0.67);
            grabbedPixels = true;
        }
        else {
            grabber.setPosition(1);
            grabbedPixels = false;
        }
    }
    public double getPosition() {
        return grabber.getPosition();
    }

}