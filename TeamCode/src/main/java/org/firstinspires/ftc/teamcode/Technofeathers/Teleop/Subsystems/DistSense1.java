package org.firstinspires.ftc.teamcode.Technofeathers.Teleop.Subsystems;

import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Controller;
import org.firstinspires.ftc.teamcode.Technofeathers.TechnofeathersDrive;
import org.firstinspires.ftc.teamcode.Technofeathers.TechnofeathersPDTest;
public class DistSense1 {
    public DistanceSensor distSense1;
    public void setUpDistSense1(HardwareMap hardwareMap) {
        distSense1 = hardwareMap.get(DistanceSensor.class, "distSense1");
    }
    public double getDistance() {
        return distSense1.getDistance(INCH);
    }
}