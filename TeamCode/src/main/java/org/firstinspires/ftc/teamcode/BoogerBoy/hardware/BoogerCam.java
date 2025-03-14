package org.firstinspires.ftc.teamcode.BoogerBoy.hardware;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.android.Utils;
import org.opencv.core.CvType;
import org.opencv.core.Size;
import org.opencv.osgi.OpenCVInterface;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.core.Core;

public class BoogerCam {
    private OpenCvCamera camera;
    private SignalPipeline pipeline;

    private Bitmap image;

    public BoogerCam(HardwareMap hwMap) {setupOpenCV(hwMap);}

    private void setupOpenCV(HardwareMap hwMap) {
        int cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
        WebcamName camName = hwMap.get(WebcamName.class, "coneCam");
        camera = OpenCvCameraFactory.getInstance().createWebcam(camName, cameraMonitorViewId);
        pipeline = new BoogerCam.SignalPipeline();
        camera.setPipeline(pipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode)
            {
//                telemetry.addData("IntakeCam", "could not start streaming");
            }
        });
    }
    public int getFaceFace() {return pipeline.face();}
    public int getAvgHue() {
        return pipeline.getAvgHue();
    }

    class SignalPipeline extends OpenCvPipeline {
        private volatile int face = 1;

        final Point REGION_TOPLEFT = new Point(500, 250); // move right by 2.3 rectangle widths, down by 0.35 rectangle widths
        static final int REGION_WIDTH = 100;
        static final int REGION_HEIGHT = 150;
        final Scalar RED = new Scalar(255, 0, 0);

        Point region_pointA = new Point(REGION_TOPLEFT.x, REGION_TOPLEFT.y);
        Point region_pointB = new Point(REGION_TOPLEFT.x + REGION_WIDTH,
                REGION_TOPLEFT.y + REGION_HEIGHT);

        private volatile int avgHue = 0;
        // yellow, cyan, magenta
        int[] hues = new int[]{70, 110,137}; // the colors: green, blue, red

        private Mat center;
        Mat hsv = new Mat();
        Mat hue = new Mat();

        @Override
        public void init(Mat firstFrame) {
            inputToHSV(firstFrame);
            center = hue.submat(new Rect(region_pointA, region_pointB));
        }

        private void inputToHSV(Mat input) {
            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);
            Core.extractChannel(hsv, hue, 0);
        }

        @Override
        public Mat processFrame(Mat in) {
            //Bitmap bmp = null;
            //Mat tmp = new Mat (height, width, CvType.CV_8U, new Scalar(4));
            try {
                //Imgproc.cvtColor(seedsImage, in, Imgproc.COLOR_RGB2BGRA);
                //Imgproc.cvtColor(seedsImage, tmp, Imgproc.COLOR_GRAY2RGBA, 4);

                image = Bitmap.createBitmap(in.cols(), in.rows(), Bitmap.Config.ARGB_8888);
                Utils.matToBitmap(in, image);
                Bitmap smol = Bitmap.createScaledBitmap(image,in.cols()/3,in.rows()/3,false);
                image = smol;
                /*image = Bitmap.createBitmap(in.cols(), in.rows(), Bitmap.Config.ARGB_8888);
                Mat resized = new Mat();
                Imgproc.resize(in,resized,new Size(in.cols(),in.rows()));
                Utils.matToBitmap(resized, image);
                boolean cool = (resized.width()==in.width() && image.getWidth()==resized.width() && resized.get(20,20).equals(in.get(20,20)));
                if(cool) {image.setPixel(0,0,16777215);}*/
            }
            catch (Exception e) {}
            inputToHSV(in);
            Imgproc.rectangle(in, region_pointA, region_pointB, RED, 2);
            avgHue = (int) Core.mean(center).val[0];
            face = colorMatch(avgHue);
            return in;
        }

        private int colorMatch(int hue) { // returns 1 for pink, 2 for blue, 123456 for black
            int closest = 1;
            int min = 361;
            for (int i = 0; i < hues.length; i++) {
                if (Math.abs(hue - hues[i]) < min) {
                    min = Math.abs(hue - hues[i]);
                    closest = i + 1;
                }
            }
            return closest;
        }

        public int face() {return face;}
        public int getAvgHue() {
            return avgHue;
        }
    }

    public Bitmap getImage() {if(image == null) {return null;} else {return image;}}
}
