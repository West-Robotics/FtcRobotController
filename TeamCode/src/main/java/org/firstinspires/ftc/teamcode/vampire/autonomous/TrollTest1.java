package org.firstinspires.ftc.teamcode.vampire.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.vampire.hardware.Intake;
import org.firstinspires.ftc.teamcode.vampire.hardware.VampireDrive;

@Autonomous(name="TrollTest1", group="Vampire")
public class TrollTest1 extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        VampireDrive drive = new VampireDrive(this, hardwareMap);
        Intake intake = new Intake(this, hardwareMap);
        ElapsedTime timer = new ElapsedTime();

        waitForStart();
        if (isStopRequested()) return;

        intake.intake();
        intake.freightStop(4);
        drive.move(0.6, 25, 36.87);
        timer.reset();
        while (timer.seconds() < 2) idle();
        intake.stop();

    }

}
