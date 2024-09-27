package org.firstinspires.ftc.teamcode.Technofeathers.Bibot.Teleop.Subsystems;



import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class AirplaneLauncher {
    public Servo airplaneLauncher;
    public void setUpAirplaneLauncher(HardwareMap hardwareMap) {
        airplaneLauncher = hardwareMap.get(Servo.class, "airplaneLauncher");
    }

    public void releaseAirplane() {
        airplaneLauncher.setPosition(0.5);
    }

    public void retractAirplane() {
        airplaneLauncher.setPosition(0.9);
    }
}