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
}