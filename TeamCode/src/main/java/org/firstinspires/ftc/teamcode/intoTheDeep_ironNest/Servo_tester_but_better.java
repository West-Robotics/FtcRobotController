

package org.firstinspires.ftc.teamcode.intoTheDeep_ironNest;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "better Servo Tester")

public class Servo_tester_but_better extends LinearOpMode {

 // Define class members
 Servo clawservo;
 Servo secondaryArm;


 @Override
 public void runOpMode() {
waitForStart();
  clawservo = hardwareMap.get(Servo.class, "claw");
  secondaryArm = hardwareMap.get(Servo.class, "secondaryArm");
  while(opModeIsActive()){
      if(gamepad2.right_trigger>0.5){
          clawservo.setPosition(1.0);
      }
      if(gamepad2.left_trigger>0.5){
          clawservo.setPosition(0.8);
      }
      if(gamepad2.a){
          secondaryArm.setPosition(0.52);
      }
      if(gamepad2.b){
          secondaryArm.setPosition(0.2);
      }
      if(gamepad2.y){
          secondaryArm.setPosition(0);
      }


  }
 }
}