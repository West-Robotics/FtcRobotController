package org.firstinspires.ftc.teamcode.intoTheDeep_ironNest;




import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = " AI auto")
    public class Ai_Reviewed_Auto extends LinearOpMode {
        private DcMotor sliders;
        private Servo secondaryArm;
        private Servo claw;
        private DcMotor wheel_1;
        private DcMotor wheel_2;
        private DcMotor wheel_3;
        private DcMotor wheel_4;
        private Servo tertiary_arm;
        private static final int MAX_POSITION = -975;
        private static final double F_COEFFICIENT = 0.0000005;
        private static final double TILES_TO_TICKS = 537.7 * (120 * 2 * 0.254 * 3.14);
        private static final int ROTATION_DISTANCE = 1;
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

        private void turn_right () {
            setTargetPositionForAllWheels((int) TURNING_NUMBER);
            setModeForAllWheels(DcMotor.RunMode.RUN_TO_POSITION);
            setPowerForAllWheels_to_turn_right(0.5);
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

        private void setPowerForAllWheels_to_turn_right(double power) {
            wheel_1.setPower(-power);
            wheel_2.setPower(-power);
            wheel_3.setPower(power);
            wheel_4.setPower(-power);
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
            tertiary_arm = hardwareMap.get(Servo.class, "tertiary arm");

            sliders.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            sliders.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            wheel_1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            wheel_2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            wheel_3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            wheel_4.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            waitForStart();

            while (opModeIsActive()) {
                // grab the sample and score it
                claw.setPosition(1);
                secondaryArm.setPosition(0.25);
                int sliderPosition = sliders.getCurrentPosition();
                double F = sliderPosition * F_COEFFICIENT;
                // sliders go up and stay up
                while (sliderPosition >= MAX_POSITION && opModeIsActive()) {
                    sliderPosition = sliders.getCurrentPosition();
                    F = sliderPosition * F_COEFFICIENT;
                    sliders.setPower(-0.5 + F);
                }
                sliders.setPower(F);
                // goes forward as long as the target position is met to hang up the specimen
                while (wheel_2.getCurrentPosition() < 0.013 * TILES_TO_TICKS && opModeIsActive()) {
                    goForward(0.5);
                    telemetry.addData("wheels position, we're going forward", wheel_2.getCurrentPosition());
                    telemetry.update();
                }
                claw.setPosition(0.65);
                // goes back to park position
                while (wheel_2.getCurrentPosition() > 0 && opModeIsActive()) {
                    goBackwards(0.5);
                    telemetry.addData("wheel postion, we're parking", wheel_2.getCurrentPosition());
                    telemetry.update();
                }
                stopMotors();
                // goes to take the other specimen
                while (wheel_2.getCurrentPosition() < ((double) 3 / 24) * 537.7 * (120 * 2 * 0.254 * 3.14) && opModeIsActive() ){
                    setPowerForAllWheels_to_turn_right(0.5);
                    telemetry.addData("Current Position", wheel_2.getCurrentPosition());
                    telemetry.update();
                }
                stopMotors();
                sleep(6000);


            }
        }

        /*private void resetAndRunWithoutEncoder(DcMotor motor) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }*/
    }

