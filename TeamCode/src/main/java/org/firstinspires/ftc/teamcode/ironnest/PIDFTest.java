package org.firstinspires.ftc.teamcode.ironnest;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Controller;
//import org.firstinspires.ftc.teamcode.PIDController;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import org.firstinspires.ftc.teamcode.Controller;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.TouchSensor;
@TeleOp

public class PIDFTest extends OpMode{




        public PIDController PID;
        public static double p=0, i=0, d=0;
        public static double f=0;
        public static int targetpos = 0;
        private final double ticksindegrees = 1800/1120;
        private TouchSensor armreset;
        private DcMotor armangle;
        public void init(){
                PID=new PIDController(p, i, d);
                armangle = hardwareMap.get(DcMotor.class, "armangle");
                armreset = hardwareMap.get(TouchSensor.class, "armreset");
        }
        public void loop() {
                PID.setPID(p, i, d);
                int armpos = armangle.getCurrentPosition()-10;
                double pid = PID.calculate(armpos, targetpos);
                double F = Math.abs(Math.cos(Math.toRadians(targetpos / ticksindegrees)))*f;
                armangle.setPower(pid + F);
                if(armreset.isPressed()==true||armpos<2||armpos>-2){
                        armangle.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                }

        }

}
