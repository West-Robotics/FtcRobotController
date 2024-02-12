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
public class Intake {
    public DcMotor intake;
    public void setUpIntake(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotor.class, "intake");
        //intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setZeroPowerBehavior(BRAKE);
    }
    public void rotateForwards() {
        intake.setPower(1);
    }
    public void rotateBackwards() {
        intake.setPower(-1);
    }
    public void off() {
        intake.setPower(0);
    }
}
