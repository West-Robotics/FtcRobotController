package org.firstinspires.ftc.teamcode.Technofeathers.Bibot.Teleop.Subsystems;

import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DistSense1 {
    public DistanceSensor distSense1;
    public void setUpDistSense1(HardwareMap hardwareMap) {
        distSense1 = hardwareMap.get(DistanceSensor.class, "distSense1");
    }
    public double getDistance() {
        return distSense1.getDistance(INCH);
    }
}