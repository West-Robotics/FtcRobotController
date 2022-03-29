package org.firstinspires.ftc.teamcode.vampire.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.vampire.hardware.Intake;
import org.firstinspires.ftc.teamcode.vampire.hardware.VampireDrive;

@Autonomous(name="TrollTest2", group="Vampire")
public class TrollTest2 extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        VampireDrive drive = new VampireDrive(this, hardwareMap);
        Intake intake = new Intake(this, hardwareMap);
        ElapsedTime timer = new ElapsedTime();

        waitForStart();
        if (isStopRequested()) return;

        intake.intake();
        timer.reset();
        intake.freightStop(4);
        drive.move(0.6, 30, -20);
        timer.reset();
        while (timer.seconds() < 2) idle();
        intake.stop();

    }

}
