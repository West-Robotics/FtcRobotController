package org.firstinspires.ftc.teamcode.Technofeathers.Teleop.Subsystems;



import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.PIDController;
import org.firstinspires.ftc.teamcode.hardware.BaseHardware;
import org.firstinspires.ftc.teamcode.hardware.Gyro;
import org.firstinspires.ftc.teamcode.hardware.drive.odometry.OdometryGlobalCoordinatePosition;

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