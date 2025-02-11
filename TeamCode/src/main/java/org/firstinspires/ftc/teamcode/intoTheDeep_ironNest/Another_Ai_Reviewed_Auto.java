package org.firstinspires.ftc.teamcode.intoTheDeep_ironNest;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
@Autonomous(name = "AI? auto")




    public class Another_Ai_Reviewed_Auto extends LinearOpMode {
        private DcMotor sliders;
        private Servo secondaryArm;
        private Servo claw;
        private DcMotor wheel_1;
        private DcMotor wheel_2;
        private DcMotor wheel_3;
        private DcMotor wheel_4;
        private Servo tertiary_arm;
        private Servo secondary_claw;

        private static final int MAX_POSITION = -975;
        private static final double F_COEFFICIENT = 0.0000005;
        private static final double TILES_TO_TICKS = 537.7 * (120 * 2 * 0.254 * 3.14);
        private static final int ROTATION_DISTANCE = 1;
        private static final double TURNING_NUMBER = ((double) ROTATION_DISTANCE / 24) * 537.7 * (120 * 2 * 0.254 * 3.14);
        private static final double FORWARD_POWER = 0.5;
        private static final double BACKWARD_POWER = -0.5;
        private static final double TURN_POWER = 0.5;

        @Override
        public void runOpMode() throws InterruptedException {
            initializeHardware();

            waitForStart();

            while (opModeIsActive()) {
                grabAndScoreSample();
                moveToParkPosition();
                takeAnotherSpecimen();
                parkRobot();
            }
        }

        private void initializeHardware() {
            claw = hardwareMap.get(Servo.class, "claw");
            sliders = hardwareMap.get(DcMotor.class, "primary_arm");
            secondaryArm = hardwareMap.get(Servo.class, "secondaryArm");
            wheel_1 = hardwareMap.get(DcMotor.class, "left_front_drive");
            wheel_2 = hardwareMap.get(DcMotor.class, "right_back_drive");
            wheel_4 = hardwareMap.get(DcMotor.class, "right_front_drive");
            wheel_3 = hardwareMap.get(DcMotor.class, "left_back_drive");
            tertiary_arm = hardwareMap.get(Servo.class, "tertiary_arm");
            secondary_claw = hardwareMap.get(Servo.class, "secondary_claw");

            sliders.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            sliders.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            wheel_1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            wheel_2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            wheel_3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            wheel_4.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        private void grabAndScoreSample() {
            claw.setPosition(1);
            secondaryArm.setPosition(0.25);
            moveSlidersToPosition(MAX_POSITION, BACKWARD_POWER);
            moveForward(0.013 * TILES_TO_TICKS, FORWARD_POWER);
            claw.setPosition(0.65);
        }

        private void moveToParkPosition() {
            moveBackward(0, FORWARD_POWER);
        }

        private void takeAnotherSpecimen() {
            turnRight(0.007, TURN_POWER);
            moveForward(0.010, FORWARD_POWER);
            sleep(3000);
        }

        private void parkRobot() {
            moveBackward(0, FORWARD_POWER);
            turnRight(0.007, -TURN_POWER);
        }

        private void moveSlidersToPosition(int targetPosition, double power) {
            int sliderPosition = sliders.getCurrentPosition();
            double F = sliderPosition * F_COEFFICIENT;
            while (sliderPosition >= targetPosition && opModeIsActive()) {
                sliderPosition = sliders.getCurrentPosition();
                F = sliderPosition * F_COEFFICIENT;
                sliders.setPower(power + F);
            }
            sliders.setPower(F);
        }

        private void moveForward(double targetPosition, double power) {
            while (wheel_2.getCurrentPosition() < targetPosition && opModeIsActive()) {
                setWheelPower(-power, power, -power, -power);
                telemetry.addData("Moving Forward", wheel_2.getCurrentPosition());
                telemetry.update();
            }
            stopMotors();
        }

        private void moveBackward(double targetPosition, double power) {
            while (wheel_2.getCurrentPosition() > targetPosition && opModeIsActive()) {
                setWheelPower(power, -power, power, power);
                telemetry.addData("Moving Backward", wheel_2.getCurrentPosition());
                telemetry.update();
            }
            stopMotors();
        }

        private void turnRight(double targetPosition, double power) {
            while (wheel_2.getCurrentPosition() < targetPosition && opModeIsActive()) {
                setWheelPower(-power, -power, power, -power);
                telemetry.addData("Turning Right", wheel_2.getCurrentPosition());
                telemetry.update();
            }
            stopMotors();
        }

        private void setWheelPower(double power1, double power2, double power3, double power4) {
            wheel_1.setPower(power1);
            wheel_2.setPower(power2);
            wheel_3.setPower(power3);
            wheel_4.setPower(power4);
        }

        private void stopMotors() {
            setWheelPower(0, 0, 0, 0);
        }
    }

