package org.firstinspires.ftc.teamcode.Technofeathers.Teleop;

import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Controller;
import org.firstinspires.ftc.teamcode.Technofeathers.TechnofeathersDrive;
import org.firstinspires.ftc.teamcode.Technofeathers.TechnofeathersPDTest;

public class Limitations extends Automations{
    double lift1CurrentRotation = lift1.getCurrentPosition()/537.7;
    public boolean LiftLimitation() {
        if (lift1CurrentRotation > 4) {
            //liftTooHigh = 1;
            return true;
        }
        else {
            //liftTooHigh = 0;
            return false;
        }
    }
    public boolean BackdropVeryClose() {
        if (distSense1.getDistance(INCH) <= 10 && 0 < controller1.left_stick_y) {
            //tooCloseToBackdrop = 1;
            return true;
        }
        else {
            //tooCloseToBackdrop = 0;
            return false;
        }
    }
}
