package org.firstinspires.ftc.teamcode.technomancers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.drive.HolonomicDrive;

public class TechnomancersDrive extends HolonomicDrive {


    //Autonomous code yes
    public TechnomancersDrive(LinearOpMode opMode, HardwareMap hwMap) {
        super(opMode, hwMap);
        setup();
    }

    //Teleop code
    public TechnomancersDrive(OpMode opMode, HardwareMap hwMap) {
        super(opMode, hwMap);
        setup();
    }

    //PID setup
    private void setup() {
        setPidTurn(0,0,0);
        setPidAutoSpeed(0, 0, 0);
        setPidAutoTurn(0, 0, 0);

        setMotorDir(false, true, false, true);
        setWheelDiameter(3.7795);
        setTicksPerRev(537.7);
    }
}
