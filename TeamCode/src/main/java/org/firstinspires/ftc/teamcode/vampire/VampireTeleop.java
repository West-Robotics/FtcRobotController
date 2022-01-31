package org.firstinspires.ftc.teamcode.vampire;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Controller;
import org.firstinspires.ftc.teamcode.vampire.hardware.Arm;
import org.firstinspires.ftc.teamcode.vampire.hardware.DuckDuckGo;
import org.firstinspires.ftc.teamcode.vampire.hardware.Intake;
import org.firstinspires.ftc.teamcode.vampire.hardware.TapeArm;
import org.firstinspires.ftc.teamcode.vampire.hardware.VampireDrive;
import org.firstinspires.ftc.teamcode.vampire.hardware.Webcam;

import java.util.ArrayList;
import java.util.stream.IntStream;

@TeleOp(name = "VAMPIRE: TeleOp")
public class VampireTeleop extends OpMode {

    // Subsystems
    private VampireDrive drive;
    private Intake intake;
    private Arm arm;
    private DuckDuckGo spin;
    private Webcam webcam;
    private TapeArm tapeArm;
    private Controller controller1;
    private Controller controller2;

    // Variables for driving
    private static final int STORE_NUM = 8;
    private ArrayList<Double> x = new ArrayList<>();
    private ArrayList<Double> y = new ArrayList<>();

    @Override
    public void init() {

        // Initialize subsystems
        drive = new VampireDrive(this, hardwareMap);
        intake = new Intake(this, hardwareMap);
        arm = new Arm(this, hardwareMap);
        spin = new DuckDuckGo(this, hardwareMap);
        webcam = new Webcam(this, hardwareMap);
        tapeArm = new TapeArm(this, hardwareMap);
        controller1 = new Controller(gamepad1);
        controller2 = new Controller(gamepad2);

        // Turn on squared inputs
        //drive.enableSquaredInputs();

        // Debug mode
        //intake.debug();
        //drive.debug();
        //arm.debug();
        spin.debug();
        //webcam.debug();
        //tapeArm.debug();

    }

    @Override
    public void loop() {

        // Update controller values
        controller1.update();
        controller2.update();

        // Add drive power
        x.add(controller1.left_stick_x);
        y.add(controller1.left_stick_y);

        // Remove
        if (x.size() > STORE_NUM) x.remove(0);
        if (y.size() > STORE_NUM) y.remove(0);

        // Average out x
        double avgX = 0;
        for (int i = 0; i < x.size(); i++) avgX += x.get(i);
        avgX /= x.size();

        // Average out y
        double avgY = 0;
        for (int i = 0; i < y.size(); i++) avgY += y.get(i);
        avgY /= y.size();

        // Drive controls
        drive.drive(avgX, avgY, controller1.right_stick_x);
        drive.togglePOV(controller1.backOnce());

        // Other subsystem controls
        intake.intake(controller1.leftBumper(), controller1.rightBumper());
        spin.spin(controller1.A(), controller1.X());
        arm.lift(controller1.dpadUp(), controller1.dpadDown());
        arm.changeStage(controller1.dpadUpOnce(), controller1.dpadDownOnce());

        // Second controller
        arm.lift(controller2.Y(), controller2.A());
        arm.toggleAuto(controller2.X());

        // The tape measure...
        tapeArm.horzMove(controller2.dpadRight(), controller2.dpadLeft());
        tapeArm.vertMove(controller2.dpadUp(), controller2.dpadDown());
        tapeArm.roll(controller2.leftBumper(), controller2.rightBumper());
        tapeArm.toggleSpeed(controller2.X());

        // Update webcam values
        webcam.update();

        telemetry.update();

    }

}
