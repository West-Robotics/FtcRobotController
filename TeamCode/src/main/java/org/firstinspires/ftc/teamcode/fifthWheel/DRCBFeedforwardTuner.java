package org.firstinspires.ftc.teamcode.fifthWheel;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.fifthWheel.subsystem.DRCB;
import org.firstinspires.ftc.teamcode.fifthWheel.subsystem.Gripper;
import org.firstinspires.ftc.teamcode.Controller;

import java.util.List;

/*
 * This routine is designed to tune the PID coefficients used by the REV Expansion Hubs for closed-
 * loop velocity control. Although it may seem unnecessary, tuning these coefficients is just as
 * important as the positional parameters. Like the other manual tuning routines, this op mode
 * relies heavily upon the dashboard. To access the dashboard, connect your computer to the RC's
 * WiFi network. In your browser, navigate to https://192.168.49.1:8080/dash if you're using the RC
 * phone or https://192.168.43.1:8080/dash if you are using the Control Hub. Once you've successfully
 * connected, start the program, and your robot will begin moving forward and backward according to
 * a motion profile. Your job is to graph the velocity errors over time and adjust the PID
 * coefficients (note: the tuning variable will not appear until the op mode finishes initializing).
 * Once you've found a satisfactory set of gains, add them to the DriveConstants.java file under the
 * MOTOR_VELO_PID field.
 */
@Config
@Autonomous(group = "fifthWheel")
public class DRCBFeedforwardTuner extends LinearOpMode {

    @Override
    public void runOpMode() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        DRCB drcb = new DRCB(hardwareMap, "liftLeft", "liftRight", "touch");
        Gripper gripper = new Gripper(hardwareMap, "flipLeft", "flipRight", "grip", Gripper.Alliance.RED);

        Controller controller = new Controller(gamepad1);
        gripper.close();
        gripper.setLevel(-1);

        //NanoClock clock = NanoClock.system();

        telemetry.addLine("Ready!");
        telemetry.update();
        telemetry.clearAll();

        waitForStart();

        if (isStopRequested()) return;

        boolean up = true;
        double lastTime = 0.0;
        /*
        while (!isStopRequested()) {

            if (clock.seconds() - lastTime > 3) {
                if (up) {
                    drcb.setLevel(3);
                    up = false;
                } else {
                    drcb.setLevel(0);
                    up = true;
                }
                lastTime = clock.seconds();
            }

            drcb.run(0);
            if (drcb.getPosition() < 30) {
                gripper.setLevel(0);
            } else {
                gripper.setLevel(8);
            }

            double velocity = drcb.getVelocity();

            // update telemetry
            telemetry.addData("targetPosition", drcb.getTargetPosition());
            telemetry.addData("targetAcceleration", drcb.getTargetAcceleration());
            telemetry.addData("targetVelocity", drcb.getTargetVelocity());
            telemetry.addData("measuredPosition", drcb.getPosition());
            telemetry.addData("measuredVelocity", velocity);
            telemetry.addData(
                    "error",
                    drcb.getTargetVelocity() - velocity
            );

            telemetry.update();

        }


         */
    }


}
