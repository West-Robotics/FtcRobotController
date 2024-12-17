package org.firstinspires.ftc.teamcode.Technofeathers.UntitledRobot.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


import org.firstinspires.ftc.teamcode.Controller;
import org.firstinspires.ftc.teamcode.Technofeathers.TechnofeathersDrive;

@TeleOp
public class mainbotteleop extends OpMode {

    public Controller controller1;
    public TechnofeathersDrive drive;

    public DcMotor verticalRight;
    public DcMotor verticalLeft;


    @Override
    public void init(){
        drive = new TechnofeathersDrive();
        controller1 = new Controller(gamepad1);
        drive.setupMotors(hardwareMap);
    }
    @Override
    public void loop(){
        controller1.update();
        drive.drive(controller1.left_stick_x, -controller1.left_stick_y/1.25, controller1.right_stick_x/1.25);



    }
}
