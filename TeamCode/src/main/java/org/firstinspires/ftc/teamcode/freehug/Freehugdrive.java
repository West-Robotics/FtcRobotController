package org.firstinspires.ftc.teamcode.freehug;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.drive.HolonomicDrive;
// you guys  nooed to reorganise your folders and files and maybe make a fork to hide this stuff 
public class Freehugdrive extends HolonomicDrive {


    // For teleop
    public Freehugdrive(OpMode opMode, HardwareMap hwMap) {

        super(opMode, hwMap);
        setup();

    }

    // For autonomous
    public Freehugdrive(LinearOpMode opMode, HardwareMap hwMap) {

        super(opMode, hwMap);
        setup();

    }

    // PID setup
    private void setup() {

        setPidTurn(0.04, 0, 0);
        setPidAutoSpeed(0.05, 0.001, 0);
        setPidAutoTurn(0.028, 0.001, 0);

        setMotorDir(true, true, false, false);

        setWheelDiameter(3.89);
        setTicksPerRev(537.6);
    }

}

