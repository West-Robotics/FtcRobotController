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

public class Functions extends Limitations{
    ElapsedTime e = new ElapsedTime();
    public void LiftGoUp() {
        if (LiftLimitation()) {
            lift1.setPower(0);
            lift2.setPower(0);
        }
        else {
            lift1.setPower(1);
            lift2.setPower(1);
        }
    }
    public void LiftGoDown() {
        lift1.setPower(-1);
        lift2.setPower(-1);
    }

    public void StopLift() {
        lift1.setPower(0);
        lift2.setPower(0);
    }


    public void Dylan() {
        if (BackdropVeryClose()) {
            ScoringPosition();
            dylanRan = 1;
        }
    }

    public void Dave() {
        PixelDropAndReset();
    }
}
