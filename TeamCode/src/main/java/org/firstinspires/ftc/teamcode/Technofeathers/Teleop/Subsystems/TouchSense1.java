package org.firstinspires.ftc.teamcode.Technofeathers.Teleop.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class TouchSense1 {
    public TouchSensor touchSense1;
    public void setUpTouchSense1 (HardwareMap hardwareMap) {
        touchSense1 = hardwareMap.get(TouchSensor.class, "touchSense1");
    }
    public boolean pressedDown() {
        return touchSense1.isPressed();
    }
}
