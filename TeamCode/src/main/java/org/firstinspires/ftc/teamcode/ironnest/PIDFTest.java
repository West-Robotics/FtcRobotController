package org.firstinspires.ftc.teamcode.IronNest;

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
        public double p=0.00, i=0, d=0;
        public double f= (float) 0.1;
        public int targetpos = 0;
        private final double ticksindegrees = 1800/1120;
        private TouchSensor armreset;
        private DcMotor Armangle;
        private Controller Gamepad1;
        public int armpos;
        public double F;
        @Override
        public void init(){
                PID=new PIDController(p, i, d);
                Armangle = hardwareMap.get(DcMotor.class, "armangle");
                armreset = hardwareMap.get(TouchSensor.class, "armreset");
                Gamepad1 = new Controller(gamepad1);

                if(armreset.isPressed()){
                        Armangle.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        Armangle.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                }
        }

        @Override
        public void loop() {

                PID.setPID(p, i, d);
                armpos = Armangle.getCurrentPosition()-(1120/180);
                double pid = PID.calculate(armpos, targetpos);
                F = Math.cos(Math.toRadians(armpos/ticksindegrees))*f;
                Armangle.setPower(F);
                telemetry.addData("armpos",armpos);
                telemetry.addData("p",p);
                telemetry.addData("targetpos",targetpos);
                telemetry.addData("d",d);
                telemetry.addData("f",f);
                telemetry.addData("F", F);
                telemetry.addData("ArmAgnelPower", Armangle.getPower());

                if(armreset.isPressed() && armpos<-4 && armpos>-8){
                        Armangle.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        Armangle.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                }
                if(Gamepad1.AOnce()){
                        f += 0.01;

                }
                if(Gamepad1.YOnce()){
                        f -= 0.01;


                }
                if(Gamepad1.XOnce()){
                        targetpos -= 1120/18;
                        telemetry.addData("targetpos", targetpos);

                }
                if(Gamepad1.BOnce()){
                        targetpos += 1120/18;
                        telemetry.addData("targetpos", targetpos*ticksindegrees);

                }
                telemetry.update();




        }

}
