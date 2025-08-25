package org.firstinspires.ftc.teamcode.intoTheDeep_ironNest;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
@TeleOp(name="shdfijoagrskhoegsij[o")
public class Basic_teleOp extends LinearOpMode{

    private DcMotor leftFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightFrontDrive;
    private DcMotor rightBackDrive;





    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if(gamepad1.dpad_up){
                goforward();
            }else if(gamepad1.dpad_down){
                gobackward();
            }

        }


    }
    public void goforward(){
        leftFrontDrive.setPower(1);
        rightFrontDrive.setPower(-1);
        leftBackDrive.setPower(1);
        rightBackDrive.setPower(-1);
    }
    public void gobackward(){
        leftFrontDrive.setPower(-1);
        rightFrontDrive.setPower(1);
        leftBackDrive.setPower(-1);
        rightBackDrive.setPower(1);
    }
}
