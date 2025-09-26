package org.firstinspires.ftc.teamcode.IronNestUNCODE;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Controller;

@TeleOp(name="Scrimtele")
public class UncodeScrim extends LinearOpMode{

    private DcMotor FL;
    private DcMotor FR;
    private DcMotor BL;
    private DcMotor BR;
    private Controller Gamepad1;




    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        FL = hardwareMap.get(DcMotor.class, "FrontL");
        FR = hardwareMap.get(DcMotor.class, "FrontR");
        BR = hardwareMap.get(DcMotor.class, "BackR");
        BL = hardwareMap.get(DcMotor.class, "BackL");
        Gamepad1 = new Controller(gamepad1);
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            FL.setPower(Gamepad1.left_stick_y+Gamepad1.left_stick_x+Gamepad1.right_stick_x);
            FR.setPower(Gamepad1.left_stick_y-Gamepad1.left_stick_x-Gamepad1.right_stick_x);
            BL.setPower(Gamepad1.left_stick_y-Gamepad1.left_stick_x+Gamepad1.right_stick_x);
            BR.setPower(Gamepad1.left_stick_y+Gamepad1.left_stick_x-Gamepad1.right_stick_x);


        }


    }

}
