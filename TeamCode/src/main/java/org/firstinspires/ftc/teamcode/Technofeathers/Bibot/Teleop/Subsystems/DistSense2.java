package org.firstinspires.ftc.teamcode.Technofeathers.Bibot.Teleop.Subsystems;

import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DistSense2 {
    public DistanceSensor distSense2;
    public void setUpDistSense2(HardwareMap hardwareMap) {
        distSense2 = hardwareMap.get(DistanceSensor.class, "distSense1");
    }
    public double getDistance() {
        return distSense2.getDistance(INCH);
    }

}
