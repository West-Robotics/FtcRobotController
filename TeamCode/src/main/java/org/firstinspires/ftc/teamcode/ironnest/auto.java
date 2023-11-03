package org.firstinspires.ftc.teamcode.ironnest;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Controller;
import org.firstinspires.ftc.teamcode.technomancers.TechnomancersDrive;

import java.util.concurrent.TimeUnit;

@Autonomous(name="Iron Nest Auto", group="IronNest")
public class auto extends LinearOpMode {

    //So whoever looks at this code, I have no idea if it will work I just copied the code from Technomancers

    @Override public void runOpMode() throws InterruptedException {

        IronNestDrive drive = new IronNestDrive(this, hardwareMap);

        waitForStart();
        ElapsedTime e = new ElapsedTime();
        e.reset();
        e.startTime();
        while(e.time(TimeUnit.SECONDS) < 1.5) {
            drive.drive(0,0.3,0);
        }
        e.reset();
        //drive.move(0.4, 60, 0);
    }
}
