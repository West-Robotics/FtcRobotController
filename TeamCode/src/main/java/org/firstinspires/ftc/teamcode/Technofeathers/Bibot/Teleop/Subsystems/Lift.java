package org.firstinspires.ftc.teamcode.Technofeathers.Bibot.Teleop.Subsystems;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Lift {
    public DcMotor lift1;
    public DcMotor lift2;
    public byte liftStatus = 0;
    public double liftCurrentRotation;
    public void setUpLift(HardwareMap hardwareMap) {
        lift1 = hardwareMap.get(DcMotor.class, "lift1");
        lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset the motor encoder
        lift1.setDirection(DcMotorSimple.Direction.FORWARD);
        lift1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift1.setZeroPowerBehavior(BRAKE);

        lift2 = hardwareMap.get(DcMotor.class, "lift2");
        lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset the motor encoder
        lift2.setDirection(DcMotorSimple.Direction.REVERSE);
        lift2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift2.setZeroPowerBehavior(BRAKE);

        liftCurrentRotation = -lift1.getCurrentPosition()/537.7;
    }
    public void resetEncoders() {
        lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset the motor encoder
        lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset the motor encoder
    }
    public void setPower(double power) {
        lift1.setPower(power);
        lift2.setPower(power);
        if (power < 0) {
            liftStatus = -1;
        }
        else if (power > 0) {
            liftStatus = 1;
        }
        else {
            liftStatus = 0;
        }
    }
    public void goUp() {
        lift1.setPower(1);
        lift2.setPower(1);
        liftStatus = 1;
    }
    public double getLiftCurrentRotation() {
        return -lift1.getCurrentPosition()/537.7;
    }

    public void goDown() {
        lift1.setPower(-1);
        lift2.setPower(-1);
        liftStatus = -1;
    }

    public void stop() {
        lift1.setPower(0);
        lift2.setPower(0);
        liftStatus = 0;
    }

    public byte getStatus() {
        return liftStatus;
    }
    public boolean maxLimitReached() {
        if (-lift1.getCurrentPosition()/537.7 >= 4) {
            return true;
        }
        else {
            return false;
        }
    }

    public boolean minLimitReached() {
        if (-lift1.getCurrentPosition()/537.7 <= 0.05) {
            return true;
        }
        else {
            return false;
        }
    }
    /*public double getRotation() {
        return liftCurrentRotation;
    }

     */
}
