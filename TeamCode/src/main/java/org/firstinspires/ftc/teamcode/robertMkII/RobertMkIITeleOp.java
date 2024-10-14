package org.firstinspires.ftc.teamcode.robertMkII;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name = "RobertMkIITeleOp")
public class RobertMkIITeleOp extends OpMode {

    private DriveTrain drivetrain;

    @Override
    public void init() {
        drivetrain = new DriveTrain(hardwareMap);
    }

    @Override
    public void loop() {
        drivetrain.tankDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
    }

}
