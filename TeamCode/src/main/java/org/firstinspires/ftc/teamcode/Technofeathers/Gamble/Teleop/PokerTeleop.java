package org.firstinspires.ftc.teamcode.Technofeathers.Gamble.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


import org.firstinspires.ftc.teamcode.Controller;
import org.firstinspires.ftc.teamcode.Technofeathers.Gamble.Teleop.Subsystems.Lift;
import org.firstinspires.ftc.teamcode.Technofeathers.TechnofeathersDrive;

@TeleOp
public class PokerTeleop extends OpMode {

    public Controller controller1;
    public TechnofeathersDrive drive;
    public Lift lift;
    /*
    public DcMotor verticalLeftLift;
    public DcMotor verticalRightLift;
    public DcMotor horizontalLeftLift;
    public DcMotor horizontalRightLift;

     */




    @Override
    public void init(){
        drive = new TechnofeathersDrive();
        controller1 = new Controller(gamepad1);
        drive.setupMotors(hardwareMap);
        lift.setupMotors(hardwareMap);
        /*
        verticalLeftLift = hardwareMap.get(DcMotor.class,  "verticalLeftLift");
        verticalRightLift = hardwareMap.get(DcMotor.class, "verticalRightLift");
        horizontalLeftLift = hardwareMap.get(DcMotor.class, "horizontalLeftLift");
        horizontalRightLift = hardwareMap.get(DcMotor.class, "horizontalRightLift");

        verticalLeftLift.setDirection(DcMotorSimple.Direction.FORWARD);
        verticalRightLift.setDirection(DcMotorSimple.Direction.FORWARD);
        horizontalLeftLift.setDirection(DcMotorSimple.Direction.FORWARD);
        horizontalRightLift.setDirection(DcMotorSimple.Direction.FORWARD);

        verticalLeftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        verticalRightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        horizontalLeftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        horizontalRightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        verticalLeftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset the motor encoder
        verticalRightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset the motor encoder
        horizontalLeftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset the motor encoder
        horizontalRightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset the motor encoder

        verticalLeftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // Turn the motor back on when we are done
        verticalRightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // Turn the motor back on when we are done
        horizontalLeftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // Turn the motor back on when we are done
        horizontalRightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // Turn the motor back on when we are done

         */

        //functions.setUp(this.hardwareMap, telemetry);
        //pivot1.setPosition(1);

    }

    @Override
    public void loop(){
        controller1.update();
        drive.drive(controller1.left_stick_x, -controller1.left_stick_y/1.25, controller1.right_stick_x/1.25);
    }
}
