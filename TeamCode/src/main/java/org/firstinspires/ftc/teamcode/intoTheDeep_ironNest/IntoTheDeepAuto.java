package org.firstinspires.ftc.teamcode.intoTheDeep_ironNest;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImpl;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import java.util.concurrent.TimeUnit;
@Autonomous(name = "Into-the-deep Auto")
public class IntoTheDeepAuto extends LinearOpMode {
    @Override
            public void  runOpMode()throws InterruptedException {
        DcMotor wheel_1;
        DcMotor wheel2;
        DcMotor wheel4;
        DcMotor wheel3;
        ElapsedTime timer;
        wheel_1 = hardwareMap.get(DcMotorImpl.class, "left_front_drive");
        wheel2 = hardwareMap.get(DcMotor.class, "right_back_drive");
        wheel4 = hardwareMap.get(DcMotor.class, "right_front_drive");
        wheel3 = hardwareMap.get(DcMotor.class, "left_back_drive");
        wheel_1.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();
        timer = new ElapsedTime();
        while (opModeIsActive()) {
            telemetry.addData("FrontLeft", wheel_1.getCurrentPosition());
            telemetry.addData("Back_ right", wheel2.getCurrentPosition());
            if (timer.seconds() < 1) {
                wheel_1.setPower((double) 1 * 3 / 4);
                wheel2.setPower((double) 1 * 3 / 4);
                wheel3.setPower((double) 1 * 3 / 4);
                wheel4.setPower((double) 1 * 3 / 4);
                telemetry.addData(" wheel 1 moving ... ", wheel_1.getCurrentPosition());
                telemetry.addData("wheel 2 moving ", wheel2.getCurrentPosition());
            } else {
                wheel_1.setPower(0);
                wheel2.setPower(0);
                wheel3.setPower(0);
                wheel4.setPower(0);
            }
        }
    }
}