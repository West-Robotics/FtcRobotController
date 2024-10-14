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
        if (gamepad1.left_bumper) {
            drivetrain.tankDrive(-gamepad1.left_stick_y/2, gamepad1.left_stick_x/2, gamepad1.right_stick_x/2);
        } else {
            drivetrain.tankDrive(-gamepad1.left_stick_y/2, gamepad1.left_stick_x/2, gamepad1.right_stick_x/2);
        }
        drivetrain.manipulateArm(-gamepad2.left_stick_y, -gamepad2.right_stick_y);
    }
}
