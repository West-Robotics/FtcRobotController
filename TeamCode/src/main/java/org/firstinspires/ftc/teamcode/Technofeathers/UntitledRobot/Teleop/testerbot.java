package org.firstinspires.ftc.teamcode.Technofeathers.UntitledRobot.Teleop;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.Controller;
@TeleOp
public class testerbot extends OpMode {

    public DcMotor motor;
    public Controller controller1;
    public float encodervalue;

    @Override
    public void init(){
        motor = hardwareMap.get(DcMotor.class,"arm");
        controller1 = new Controller(gamepad1);
    }
    @Override
    public void loop(){
        controller1.update();
        motor.setPower(controller1.right_stick_y);
        encodervalue= motor.getCurrentPosition();
        telemetry.addData("encoder",encodervalue);
        telemetry.update() ;
        if(controller1.AOnce()){
            motor.setPower(1);
        }
        if(controller1.BOnce()){
            motor.setPower(0);
        }
    }
}
