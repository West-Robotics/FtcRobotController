package org.firstinspires.ftc.teamcode.intoTheDeep_ironNest;




import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

/** @noinspection FieldCanBeLocal*/
@Config
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
        private Servo secondary_claw;
        public static final int MAX_POSITION = -1125;
        public  static final int MIN_POSITION = 975;
        public static final double F_COEFFICIENT = 0.0000005;
        public static final double TILES_TO_TICKS = 537.7 * (120 * 2 * 0.254 * 3.14);
        public static final int ROTATION_DISTANCE = 1;
        public static final double TURNING_NUMBER = ((double) ROTATION_DISTANCE / 24) * 537.7 * (120 * 2 * 0.254 * 3.14);
        int specimen_number = 0;

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




        private void setModeForAllWheels(DcMotor.RunMode mode) {
            wheel_1.setMode(mode);
            wheel_2.setMode(mode);
            wheel_3.setMode(mode);
            wheel_4.setMode(mode);
        }

        private void setPowerForAllWheels_to_turn_right(double power) {
            // left
            wheel_1.setPower(-power);
            // right
            wheel_2.setPower(-power);
            //left
            wheel_3.setPower(-power);
            //right
            wheel_4.setPower(power);
        }

        @Override
        public void runOpMode() throws InterruptedException {
            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
            claw = hardwareMap.get(Servo.class, "claw");
            sliders = hardwareMap.get(DcMotor.class, "primary_arm");
            secondaryArm = hardwareMap.get(Servo.class, "secondaryArm");
            wheel_1 = hardwareMap.get(DcMotor.class, "left_front_drive");
            wheel_2 = hardwareMap.get(DcMotor.class, "right_back_drive");
            wheel_4 = hardwareMap.get(DcMotor.class, "right_front_drive");
            wheel_3 = hardwareMap.get(DcMotor.class, "left_back_drive");
            tertiary_arm = hardwareMap.get(Servo.class, "tertiary_arm");
            secondary_claw = hardwareMap.get(Servo.class, "secondary_claw");
            reset_encoders();
            sliders.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            sliders.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            sliders.setDirection(DcMotorSimple.Direction.FORWARD);
            wheel_1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            wheel_2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            wheel_3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            wheel_4.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            waitForStart();

            while (opModeIsActive()) {
                reset_encoders();
                int min_pos = sliders.getCurrentPosition();
                // grab the sample and score it
                claw.setPosition(1);
                int sliderPosition = sliders.getCurrentPosition();
                double F = sliderPosition * F_COEFFICIENT;
                // sliders go up and stay up
                while (sliderPosition >= MAX_POSITION && opModeIsActive() && sliderPosition != MAX_POSITION) {
                    sliderPosition = sliders.getCurrentPosition();
                    F = sliderPosition * F_COEFFICIENT;
                    sliders.setPower(-0.5 + F);
                    telemetry.addData("initial position read of sliders", min_pos);
                    telemetry.addData("Slider position", sliderPosition);
                    telemetry.addData("Target", MAX_POSITION);
                    telemetry.update();
                }
                sliders.setPower(F);
                secondaryArm.setPosition(0.25);
                min_pos = wheel_2.getCurrentPosition();
                // goes forward as long as the target position is met to hang up the specimen
                while (wheel_2.getCurrentPosition() < 0.013 * TILES_TO_TICKS && opModeIsActive()) {
                    goForward(0.5);
                    telemetry.addData("initial position read of wheels", min_pos);
                    telemetry.addData("wheels position, we're going forward", wheel_2.getCurrentPosition());
                    telemetry.addData("sliders position", sliders.getCurrentPosition());
                    telemetry.update();
                }
                claw.setPosition(0.65);
                // goes back to park position
                while (wheel_2.getCurrentPosition() > 0 && opModeIsActive()) {
                    goBackwards(0.5);
                    telemetry.addData("wheel position, we're parking", wheel_2.getCurrentPosition());
                    telemetry.addData("sliders position", sliders.getCurrentPosition());
                    telemetry.update();
                }
                stopMotors();
                // goes to take the other specimen
                while (wheel_2.getCurrentPosition() >  -1000 && opModeIsActive() ){
                    setPowerForAllWheels_to_turn_right(0.5);
                    telemetry.addData("target, turning ",-1000);
                    telemetry.addData("Current Position", wheel_2.getCurrentPosition());
                    telemetry.update();
                }
                stopMotors();
                reset_encoders();
                min_pos = wheel_2.getCurrentPosition();
                while (wheel_2.getCurrentPosition() <=1000 && opModeIsActive()){
                    goForward(0.5);
                    telemetry.addData("reset position of wheels",min_pos);
                    telemetry.addData("wheels position, we're going forward", wheel_2.getCurrentPosition());
                    telemetry.addData("target",1000 );
                    telemetry.addData("slider position", sliders.getCurrentPosition());
                    telemetry.update();
                }
                stopMotors();
                min_pos = sliders.getCurrentPosition();
                while (sliders.getCurrentPosition() > 1000 && opModeIsActive()) {
                    sliderPosition = sliders.getCurrentPosition();
                    F = sliderPosition * F_COEFFICIENT;
                    F = sliderPosition * F_COEFFICIENT;
                    sliders.setPower(0.5 + F);
                    telemetry.addData("sliders reset position", min_pos);
                    telemetry.addData("Slider position",sliders.getCurrentPosition());
                    telemetry.addData("Target",min_pos);
                    telemetry.addData("wheel position", wheel_2.getCurrentPosition());
                    telemetry.update();

                }
                sliders.setPower(F);
                stopMotors();
                secondaryArm.setPosition(1);
                telemetry.update();
                specimen_number = specimen_number + 1;
                if (specimen_number == 2){
                    break;
                }else {
                    sleep(5000);
                    telemetry.addData("specimen number", specimen_number);
                    telemetry.update();
                }
                claw.setPosition(1);
                while (wheel_2.getCurrentPosition() > 0 && opModeIsActive()){
                    goBackwards(0.5);
                    telemetry.addData("we're parking, position:", wheel_2.getCurrentPosition());
                    telemetry.addData("target is 0", 0);
                    telemetry.update();
                }
                stopMotors();
                while (wheel_2.getCurrentPosition() < 1000 && opModeIsActive() ){
                    setPowerForAllWheels_to_turn_right(-0.5);
                    telemetry.addData("target, turning back", 1000) ;
                    telemetry.addData("Current Position", wheel_2.getCurrentPosition());
                    telemetry.update();
                }
                stopMotors();
                reset_encoders();


            }
        }

        private void resetAndRunWithoutEncoder(DcMotor motor) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

    private void reset_encoders() {
    resetAndRunWithoutEncoder(wheel_1);
    resetAndRunWithoutEncoder(wheel_2);
    resetAndRunWithoutEncoder(wheel_3);
    resetAndRunWithoutEncoder(wheel_4);
    resetAndRunWithoutEncoder(sliders);
    }
}
