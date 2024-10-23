package org.firstinspires.ftc.teamcode.robertMkII;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
TODO:
- change length of moving right
- add more function than moving right
 */

@Autonomous(name = "RobertMkIIAuto")
public class RobertMkIIAuto extends LinearOpMode {

    @Override public void runOpMode() throws InterruptedException {
        DriveTrain driveTrain = new DriveTrain(hardwareMap);

        ElapsedTime e = new ElapsedTime();
        waitForStart();
        e.reset();
        while (opModeIsActive()) {
            if (e.seconds() < 2.5) {
                driveTrain.tankDrive(0,1,0);
            } else if (e.seconds() > 2.5 && e.seconds() < 6){
                driveTrain.tankDrive(0,-1,0);
            } else {
                driveTrain.tankDrive(0,0,0);
            }
        }
    }

}
