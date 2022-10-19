package org.firstinspires.ftc.teamcode.BoogerBoy;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;







@Autonomous(name = "minicat")
public class BoogerBoyAuto extends LinearOpMode{
    private BoogerBoyDrive drive;

    // makeing a webcam for open cv
    OpenCvWebcam webcam;

    private int AI_resualt;
    // This is for when the cone is in the first case

    private void uno()
    {
        drive.move(1,60,0);
    }
    // This is for when the cone is in the seconds case
    private void dos()
    {
        drive.move(1,60,0);
        drive.move(1,60,90);
    }
    // This is for when the cone is in the third case
    private void tres()
    {
        drive.move(1,60,0);
        drive.move(1,60,-90);
    }

    @Override
    public void runOpMode() throws InterruptedException {

        // Start the pipeline for OpenCV








            // AI Goes Here ^ ^ ^ ^

        AI_resualt = 1;

        // Theese detect what the AI Detected and run the corasponding code
        if(AI_resualt == 1)
        {
            uno();
        }
        if(AI_resualt == 2)
        {
            dos();
        }
        if(AI_resualt == 3)
        {
            tres();
        }



    }
}
