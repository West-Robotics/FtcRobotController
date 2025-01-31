package org.firstinspires.ftc.teamcode.intoTheDeep_ironNest;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImpl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
@Autonomous(name = "Maybe even better auto?")
public class Into_the_deep_auto_test_2 extends LinearOpMode{
    DcMotor sliders;
    Servo secondaryArm;
    Servo claw;
    int maxPosition = -975;
    public int slider_position;
    public double F;
    public double f = (float) 0.0000005;
    DcMotor wheel_1;
    DcMotor wheel2;
    DcMotor wheel4;
    DcMotor wheel3;
    public double tilesToTicks = 537.7*(120*2*0.254*3.14);


    @Override
    public void runOpMode() throws InterruptedException {
        claw = hardwareMap.get(Servo.class, "claw");
        sliders = hardwareMap.get(DcMotor.class, "primary_arm");
        secondaryArm = hardwareMap.get(Servo.class, "secondaryArm");
        wheel_1 = hardwareMap.get(DcMotor.class, "left_front_drive");
        wheel2 = hardwareMap.get(DcMotor.class, "right_back_drive");
        wheel4 = hardwareMap.get(DcMotor.class, "right_front_drive");
        wheel3 = hardwareMap.get(DcMotor.class, "left_back_drive");

        sliders.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sliders.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheel_1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheel_1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wheel2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheel2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wheel3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheel3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wheel4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheel4.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();
//        while (opModeIsActive()) {
//            int currentPosition = sliders.getCurrentPosition();
//            if (currentPosition >= maxPosition) {
//                sliders.setPower(-0.5);
//            }else {
//                sliders.setPower(0);
//            }
//        /}
        while (opModeIsActive()) {
            claw.setPosition(1);
            secondaryArm.setPosition(0.25);
            slider_position = sliders.getCurrentPosition();
            F = slider_position * f;
            while (slider_position>= maxPosition){
                slider_position = sliders.getCurrentPosition();
                F = slider_position * f;
                sliders.setPower(-0.5 + F);
                telemetry.addData("F", F);
                telemetry.addData("wheels",wheel2.getCurrentPosition());
                telemetry.update();
            }
            sliders.setPower(F);
            while (wheel2.getCurrentPosition()<0.013*tilesToTicks){
                wheel_1.setPower(-0.5);
                wheel2.setPower(0.5);
                wheel3.setPower(-0.5);
                wheel4.setPower(-0.5);
            }
            wheel_1.setPower(0);
            wheel2.setPower(0);
            wheel3.setPower(0);
            wheel4.setPower(0);
            claw.setPosition(0.65);
            while (wheel2.getCurrentPosition()>0){
                wheel_1.setPower(0.5);
                wheel2.setPower(-0.5);
                wheel3.setPower(0.5);
                wheel4.setPower(0.5);
            }
            while (wheel2.getCurrentPosition() <0.013*tilesToTicks){
             wheel_1.setPower(-0.5); 
             wheel3.setPower(-0.5);
             wheel2.setPower(-0.5);
             wheel4.setPower(0.5);
            }
            wheel2.setPower(0);
            wheel_1.setPower(0);
            wheel3.setPower(0);
            wheel4.setPower(0);
            ElapsedTime timer = new ElapsedTime();
            while (timer.seconds() < 1.5) {
                wheel_1.setPower(-0.75);
                wheel2.setPower(0.75);
                wheel3.setPower(0.75);
                wheel4.setPower(0.75);
            }
            wheel_1.setPower(0.75);
            wheel2.setPower(-0.75);
            wheel3.setPower(-0.75);
            wheel4.setPower(-0.75);
            break;


        }
    }
}
