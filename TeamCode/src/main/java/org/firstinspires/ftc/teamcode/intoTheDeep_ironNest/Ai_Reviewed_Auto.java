package org.firstinspires.ftc.teamcode.intoTheDeep_ironNest;




import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

    @Autonomous(name = " AI auto")
    public class Ai_Reviewed_Auto extends LinearOpMode {
        private DcMotor sliders;
        private Servo secondaryArm;
        private Servo claw;
        private DcMotor wheel_1;
        private DcMotor wheel_2;
        private DcMotor wheel_3;
        private DcMotor wheel_4;

        private static final int MAX_POSITION = -975;
        private static final double F_COEFFICIENT = 0.0000005;
        private static final double TILES_TO_TICKS = 537.7 * (120 * 2 * 0.254 * 3.14);
        private static final int ROTATION_DISTANCE = 9;
        private static final double TURNING_NUMBER = ((double) ROTATION_DISTANCE / 24) * 537.7 * (120 * 2 * 0.254 * 3.14);

        private void stopMotors() {
            wheel_1.setPower(0);
            wheel_2.setPower(0);
            wheel_3.setPower(0);
            wheel_4.setPower(0);
        }

        private void goForward(double power) {
            wheel_1.setPower(-power);
            wheel_2.setPower(power);
            wheel_3.setPower(-power);
            wheel_4.setPower(-power);
        }

        private void goBackwards(double power) {
            wheel_1.setPower(power);
            wheel_2.setPower(-power);
            wheel_3.setPower(power);
            wheel_4.setPower(power);
        }

        private void turnRight() {
            setTargetPositionForAllWheels((int) TURNING_NUMBER);
            setModeForAllWheels(DcMotor.RunMode.RUN_TO_POSITION);
            setPowerForAllWheels(0.5);
            telemetry.addData("Current position", wheel_1.getCurrentPosition());
            telemetry.addData("target position", TURNING_NUMBER);
            telemetry.update();
        }

        private void setTargetPositionForAllWheels(int position) {
            wheel_1.setTargetPosition(position);
            wheel_2.setTargetPosition(position);
            wheel_3.setTargetPosition(-position);
            wheel_4.setTargetPosition(position);
        }

        private void setModeForAllWheels(DcMotor.RunMode mode) {
            wheel_1.setMode(mode);
            wheel_2.setMode(mode);
            wheel_3.setMode(mode);
            wheel_4.setMode(mode);
        }

        private void setPowerForAllWheels(double power) {
            wheel_1.setPower(power);
            wheel_2.setPower(power);
            wheel_3.setPower(power);
            wheel_4.setPower(power);
        }

        @Override
        public void runOpMode() throws InterruptedException {
            claw = hardwareMap.get(Servo.class, "claw");
            sliders = hardwareMap.get(DcMotor.class, "primary_arm");
            secondaryArm = hardwareMap.get(Servo.class, "secondaryArm");
            wheel_1 = hardwareMap.get(DcMotor.class, "left_front_drive");
            wheel_2 = hardwareMap.get(DcMotor.class, "right_back_drive");
            wheel_4 = hardwareMap.get(DcMotor.class, "right_front_drive");
            wheel_3 = hardwareMap.get(DcMotor.class, "left_back_drive");

            sliders.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            sliders.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            resetAndRunWithoutEncoder(wheel_1);
            resetAndRunWithoutEncoder(wheel_2);
            resetAndRunWithoutEncoder(wheel_3);
            resetAndRunWithoutEncoder(wheel_4);

            waitForStart();

            while (opModeIsActive()) {
                claw.setPosition(1);
                secondaryArm.setPosition(0.25);
                int sliderPosition = sliders.getCurrentPosition();
                double F = sliderPosition * F_COEFFICIENT;

                while (sliderPosition >= MAX_POSITION && opModeIsActive()) {
                    sliderPosition = sliders.getCurrentPosition();
                    F = sliderPosition * F_COEFFICIENT;
                    sliders.setPower(-0.5 + F);
                    telemetry.addData("F", F);
                    telemetry.addData("wheels", wheel_2.getCurrentPosition());
                    telemetry.update();
                }
                sliders.setPower(F);

                while (wheel_2.getCurrentPosition() < 0.013 * TILES_TO_TICKS && opModeIsActive()) {
                    goForward(0.5);
                }
                claw.setPosition(0.65);

                while (wheel_2.getCurrentPosition() > 0 && opModeIsActive()) {
                    goBackwards(0.5);
                }
                stopMotors();
                turnRight();
                stop();
                sleep(6000);


            }
        }

        private void resetAndRunWithoutEncoder(DcMotor motor) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

