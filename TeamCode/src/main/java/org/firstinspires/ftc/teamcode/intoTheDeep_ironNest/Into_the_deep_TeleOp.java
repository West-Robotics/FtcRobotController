package org.firstinspires.ftc.teamcode.intoTheDeep_ironNest;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
@TeleOp(name="Into The Deep TeleOp")
public class Into_the_deep_TeleOp extends LinearOpMode {
    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightFrontDrive;
    private DcMotor rightBackDrive;
    private Servo clawservo;
    private Servo secondaryArm;
    private DcMotor sliders;
    private DcMotor sliders2;
    public int slider_position;
    int maxPosition = -2900;
    public double F;
    public double f = (float) 0.0000005;
    Servo tertiary_arm;
    Servo secondary_claw;
    int ideal_position= -1025;

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        clawservo = hardwareMap.get(Servo.class, "claw");
        secondaryArm = hardwareMap.get(Servo.class, "secondaryArm");
        sliders = hardwareMap.get(DcMotor.class, "primary_arm");
        sliders2 = hardwareMap.get(DcMotor.class, "primary_arm2");
        tertiary_arm = hardwareMap.get(Servo.class, "tertiary_arm");
        secondary_claw = hardwareMap.get(Servo.class, "secondary_claw");
        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        sliders.setMode(RUN_WITHOUT_ENCODER);
        sliders.setMode(STOP_AND_RESET_ENCODER);
        sliders2.setMode(RUN_WITHOUT_ENCODER);
        sliders2.setDirection(DcMotor.Direction.REVERSE);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower = axial - lateral + yaw;
            double rightBackPower = axial + lateral - yaw;
            double sliderPower = gamepad2.right_stick_y;


            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
                sliderPower /= max;
            }

            // This is test code:
            //
            // Uncomment the following code to test your motor directions.
            // Each button should make the corresponding motor run FORWARD.
            //   1) First get all the motors to take to correct positions on the robot
            //      by adjusting your Robot Configuration if necessary.
            //   2) Then make sure they run in the correct direction by modifying the
            //      the setDirection() calls above.
            // Once the correct motors move in the correct direction re-comment this code.

            /*
            leftFrontPower  = gamepad1.x ? 1.0 : 0.0;  // X gamepad
            leftBackPower   = gamepad1.a ? 1.0 : 0.0;  // A gamepad
            rightFrontPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
            rightBackPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad
            */

            // Send calculated power to wheels
// Speed up the motors
            boolean left_bumper = gamepad1.left_bumper;
            boolean right_bumper = gamepad1.right_bumper;
            if (left_bumper||right_bumper) {
                leftFrontDrive.setPower(leftFrontPower);
                rightFrontDrive.setPower(rightFrontPower);
                leftBackDrive.setPower(leftBackPower);
                rightBackDrive.setPower(rightBackPower);
            } else {
                leftFrontDrive.setPower(leftFrontPower / 2);
                rightFrontDrive.setPower(rightFrontPower / 2);
                leftBackDrive.setPower(leftBackPower / 2);
                rightBackDrive.setPower(rightBackPower / 2);
            }
            // main claw
            if (gamepad2.right_trigger > 0.5) {
                clawservo.setPosition(0.9);
            }
            if (gamepad2.left_trigger > 0.5) {
                clawservo.setPosition(0.65);
            }


            // secondary claw
            if (gamepad1.right_trigger > 0.5){
                secondary_claw.setPosition(0.0);
            }
            if (gamepad1.left_trigger > 0.5){
                secondary_claw.setPosition(0.3);
            }

            // secondary arm positions
            if (gamepad2.a) {
                secondaryArm.setPosition(0.575);
            }
            if (gamepad2.b) {
                secondaryArm.setPosition(0.225);
            }
            if (gamepad2.y) {
                secondaryArm.setPosition(0.0);
            }
            // tertiary arm
            if (gamepad1.dpad_up){
                tertiary_arm.setPosition(0.9);
            }
            if (gamepad1.dpad_right){
                tertiary_arm.setPosition(0.45);
            }
            if (gamepad1.dpad_down){
                tertiary_arm.setPosition(0.37);
            }

            //Pid
            int currentPosition = sliders.getCurrentPosition();
            F = currentPosition * f;
                if (currentPosition > maxPosition) {
                    sliders.setPower(sliderPower+F);
                    sliders2.setPower(sliderPower+F);

                }
                else {
                    sliders.setPower(Math.abs(sliderPower/5));
                    sliders2.setPower(Math.abs(sliderPower/5));
                }
                if (gamepad2.x) {
                    if (sliders.getCurrentPosition() > ideal_position && opModeIsActive() ) {
                        sliders.setPower(0.5);
                    }
                    sliders.setPower(0);
                }


                // Show
                telemetry.addData("sliderPower", sliderPower);
                telemetry.addData("slider position", sliders.getCurrentPosition());
                telemetry.addData("Status", "Run Time: " + runtime.toString());
                telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
                telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
                telemetry.update();

        }
    }}
