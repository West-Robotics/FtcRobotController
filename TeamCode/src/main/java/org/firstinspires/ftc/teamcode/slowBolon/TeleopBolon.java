package org.firstinspires.ftc.teamcode.slowBolon;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Controller;
import org.firstinspires.ftc.teamcode.hardware.Gyro;


@TeleOp(name = "BOLON teleop")
public class TeleopBolon extends OpMode {

    private DriveBolon drive;
    private Controller controller;
    private Gyro gyro;
    //private AndroidOrientation orientation;


    @Override
    public void init() {
        drive = new DriveBolon(this, hardwareMap);
        gyro = new Gyro(hardwareMap, false);
        //orientation = new AndroidOrientation();
        controller = new Controller(gamepad1);
        drive.togglePOV(true);
        //orientation.startListening();
        gyro.reset();
    }

    @Override
    public void loop() {

        telemetry.addData("milfs", gyro.getAngleDegrees());
        /*telemetry.addData("angle",orientation.getAngle());
        telemetry.addData("angle2", orientation.getAzimuth());
        telemetry.addData("tobias commited arson", orientatigon-/.etMagnitude());
        telemetry.addData("tobias executed for arson", orientation.getPitch());
        telemetry.addData("ben", orientation.getRoll());
        telemetry.addData("available",orientation.isAvailable());*/
        telemetry.update();
        controller.update();
        drive.drive(controller.left_stick_x, controller.left_stick_y, controller.right_stick_x);
        drive.togglePOV(controller.leftStickButtonOnce());
    }

}