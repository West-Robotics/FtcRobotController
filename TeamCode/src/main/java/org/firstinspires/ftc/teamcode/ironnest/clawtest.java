package org.firstinspires.ftc.teamcode.ironnest;
import org.firstinspires.ftc.teamcode.Controller;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

public class clawtest {
    private Servo claw1;
    private Controller controller;
    private Servo claw2;
    public void init(){
        claw1= hardwareMap.get(Servo.class, "claw1");
        claw2= hardwareMap.get(Servo.class, "claw2");
        controller = new Controller(gamepad1);
    }
    public void loop(){
        if (gamepad1.a) {
            claw1.setPosition(0.17);
            claw2.setPosition(0.22);
        } else if (gamepad1.b) {
            claw1.setPosition(0);
            claw2.setPosition(0.30);
        }

    }
}
