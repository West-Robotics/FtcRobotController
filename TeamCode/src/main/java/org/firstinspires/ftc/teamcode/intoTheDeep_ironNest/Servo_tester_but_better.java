

package org.firstinspires.ftc.teamcode.intoTheDeep_ironNest;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "better Servo Tester")

public class Servo_tester_but_better extends LinearOpMode {

 // Define class members
 Servo   servo;


 @Override
 public void runOpMode() {
waitForStart();
  servo = hardwareMap.get(Servo.class, "claw");
  servo.setPosition(1.0);
 }
}