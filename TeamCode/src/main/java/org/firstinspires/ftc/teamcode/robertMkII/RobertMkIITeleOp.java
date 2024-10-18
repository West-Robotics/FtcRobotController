package org.firstinspires.ftc.teamcode.robertMkII;
/*
TODO:
- make hand pos preset
- rebind the other things
- implement the old controller new controller thing? (or use bools to define what pos or enum)
- buy bricks
*/
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
            drivetrain.tankDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        }
        drivetrain.manipulateArm(-gamepad2.left_stick_y, gamepad2.left_stick_x);
        drivetrain.moveHand(-gamepad1.right_stick_y,gamepad2.left_trigger-gamepad2.right_trigger);
    }
}
