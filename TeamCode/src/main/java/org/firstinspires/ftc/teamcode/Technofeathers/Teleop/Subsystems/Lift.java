package org.firstinspires.ftc.teamcode.Technofeathers.Teleop.Subsystems;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
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
public class Lift {
    public DcMotor lift1;
    public DcMotor lift2;
    public void setUpLift(HardwareMap hardwareMap) {
        lift1 = hardwareMap.get(DcMotor.class, "lift1");
        lift1.setDirection(DcMotorSimple.Direction.FORWARD);
        lift1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift1.setZeroPowerBehavior(BRAKE);

        lift2 = hardwareMap.get(DcMotor.class, "lift2");
        lift2.setDirection(DcMotorSimple.Direction.REVERSE);
        lift2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift2.setZeroPowerBehavior(BRAKE);
    }
    public void goUp() {
        lift1.setPower(1);
        lift2.setPower(1);
    }

    public void goDown() {
        lift1.setPower(-1);
        lift2.setPower(-1);
    }

    public void stop() {
        lift1.setPower(0);
        lift2.setPower(0);
    }

    public boolean limitReached() {
        if (lift1.getCurrentPosition()/537.7 < 4) {
            return true;
        }
        else {
            return false;
        }
    }
}
