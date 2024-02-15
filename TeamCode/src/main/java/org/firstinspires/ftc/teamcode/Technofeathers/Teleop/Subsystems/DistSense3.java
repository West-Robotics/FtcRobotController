package org.firstinspires.ftc.teamcode.Technofeathers.Teleop.Subsystems;

import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DistSense3 {
    public DistanceSensor distSense3;
    public void setUpDistSense3(HardwareMap hardwareMap) {
        distSense3 = hardwareMap.get(DistanceSensor.class, "distSense1");
    }
    public double getDistance() {
        return distSense3.getDistance(INCH);
    }
}
