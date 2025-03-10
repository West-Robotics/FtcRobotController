package org.firstinspires.ftc.teamcode.slowBolon;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.drive.HolonomicDrive;

public class DriveBolon extends HolonomicDrive {


    // For autonomous

    public DriveBolon(LinearOpMode opMode, HardwareMap hwMap) {

        super(opMode, hwMap);
        setup();

    }

    // teleop
    public DriveBolon(OpMode opMode, HardwareMap hwMap) {
        super(opMode, hwMap);
    }

    // PID setup
    public void setup() {

        setPidTurn(0.0, 0, 0);
        setPidAutoSpeed(0.05, 0.001, 0);
        setPidAutoTurn(0.028, 0.001, 0);

        setMotorDir(false, true, false, true);

        setWheelDiameter(3.89);
        setTicksPerRev(537.6);
    }

}

// removed