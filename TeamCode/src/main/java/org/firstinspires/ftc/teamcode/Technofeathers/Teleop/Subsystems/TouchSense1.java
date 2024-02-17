package org.firstinspires.ftc.teamcode.Technofeathers.Teleop.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TouchSense1 {
    public TouchSensor touchSense1;
    public Telemetry telemetry;
    public double a;

    public void setUpTouchSense1 (HardwareMap hardwareMap, Telemetry telemetry) {
        touchSense1 = hardwareMap.get(TouchSensor.class, "touchSense1");
        this.telemetry = telemetry;
    }
    public boolean pressedDown() {
        if (touchSense1.isPressed()) {
            telemetry.addLine("touch Sense 1 is pressed");
            telemetry.addData("Touch Sense 1 Press down how much: ", touchSense1.getValue());
            return true;
        }
        else {
            return false;
        }
    }
    public boolean getA() {
        return touchSense1.isPressed();
    }
}
