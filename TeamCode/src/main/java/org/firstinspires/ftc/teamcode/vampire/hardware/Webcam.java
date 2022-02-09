package org.firstinspires.ftc.teamcode.vampire.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.hardware.BaseHardware;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

public class Webcam extends BaseHardware {

    // For OpenCV
    private OpenCvCamera webcam;
    private CargoDeterminationPipeline cargoPipeline;
    private DuckPositionPipeline duckPipeline;

    // Video resolution
    private static final int WIDTH = 800;
    private static final int HEIGHT = 600;

    // Modes
    private boolean isCargo = true;

    public Webcam(OpMode opMode, HardwareMap hwMap) {

        super(opMode);
        setupOpenCV(hwMap);

    }

    public Webcam(LinearOpMode opMode, HardwareMap hwMap) {

        super(opMode);
        setupOpenCV(hwMap);

    }

    private void setupOpenCV(HardwareMap hwMap) {

        // Webcam setup
        int cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hwMap.get(WebcamName.class, "webcam"), cameraMonitorViewId);
        cargoPipeline = new CargoDeterminationPipeline();
        duckPipeline = new DuckPositionPipeline();
        webcam.setPipeline(duckPipeline);
        webcam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        // Start video streaming
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {

            @Override
            public void onOpened() { webcam.startStreaming(WIDTH, HEIGHT, OpenCvCameraRotation.UPSIDE_DOWN); }

            @Override
            public void onError(int errorCode) {}

        });

    }

    public void update() {

        // For OpenCV
        print("Threshold Left", cargoPipeline.avgL);
        print("Threshold Mid", cargoPipeline.avgM);
        print("Threshold Right", cargoPipeline.avgR);
        print("X", duckPipeline.xNum);
        print("Y", duckPipeline.yNum);
        print("Average Hue", duckPipeline.desiredHSV[0]);
        print("Average Saturation", duckPipeline.desiredHSV[1]);
        print("Average Brightness", duckPipeline.desiredHSV[2]);
        print("Position", getCargoPos());
        print("Mode", isCargo);

    }

    public void toggleMode(boolean button) {

        if (button) {

            isCargo = !isCargo;

            if (isCargo) webcam.setPipeline(cargoPipeline);
            else webcam.setPipeline(duckPipeline);

        }

    }

    public int getCargoPos() {

        if (cargoPipeline.getPosition() == CargoDeterminationPipeline.CargoPosition.BOTTOM) return 1;
        else if (cargoPipeline.getPosition() == CargoDeterminationPipeline.CargoPosition.MIDDLE) return 2;
        else if (cargoPipeline.getPosition() == CargoDeterminationPipeline.CargoPosition.TOP) return 3;
        else return 0;

    }

    private static class DuckPositionPipeline extends OpenCvPipeline {

        private static final int REGION_WIDTH = 40;
        private static final int REGION_HEIGHT = 40;
        private static final int NUM_HORZ = WIDTH / REGION_WIDTH;
        private static final int NUM_VERT = HEIGHT / REGION_HEIGHT;
        private static final Mat[][][] regions = new Mat[3][NUM_HORZ][NUM_VERT]; // H = 0, S = 1, V = 2
        private Mat hsv = new Mat();
        private Mat[] hsvArray = new Mat[] { new Mat(), new Mat(), new Mat() };
        private static final int[] GOAL = new int[] { 24, 140, 180 };

        private int[] desiredHSV = new int[3];
        private int xNum = 0;
        private int yNum = 0;

        private void inputToHSV(Mat input) {

            // Convert to hsv
            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);
            Core.extractChannel(hsv, hsvArray[0], 0);
            Core.extractChannel(hsv, hsvArray[1], 1);
            Core.extractChannel(hsv, hsvArray[2], 2);

        }

        @Override
        public void init(Mat firstFrame) {

            inputToHSV(firstFrame);
            for (int i = 0; i < 3; i++) {

                for (int h = 0; h < NUM_HORZ; h++) {

                    for (int v = 0; v < NUM_VERT; v++) {

                        regions[i][h][v] = hsvArray[i].submat(new Rect(
                                new Point(h * REGION_WIDTH, v * REGION_HEIGHT),
                                new Point((h + 1) * REGION_WIDTH, (v + 1) * REGION_HEIGHT)));

                    }

                }

            }

        }

        @Override
        public Mat processFrame(Mat input) {

            inputToHSV(input);

            double closest = 10000;
            for (int h = 0; h < NUM_HORZ; h++) {

                for (int v = 0; v < NUM_VERT; v++) {

                    int avgHue = (int) Core.mean(regions[0][h][v]).val[0];
                    int avgSaturation = (int) Core.mean(regions[1][h][v]).val[0];
                    int avgBrightness = (int) Core.mean(regions[2][h][v]).val[0];

                    // Calculate difference between goal hue and calculated average hue
                    double difference = Math.hypot(Math.hypot(GOAL[0] - avgHue, GOAL[1] - avgSaturation), GOAL[2] - avgBrightness);
                    if (difference < closest) {

                        desiredHSV[0] = avgHue;
                        desiredHSV[1] = avgSaturation;
                        desiredHSV[2] = avgBrightness;

                        closest = difference;
                        xNum = h;
                        yNum = v;

                    }

                }

            }

            // Draw rectangles
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    new Point(xNum * REGION_WIDTH, yNum * REGION_HEIGHT), // First point which defines the rectangle
                    new Point((xNum + 1) * REGION_WIDTH, (yNum + 1) * REGION_HEIGHT), // Second point which defines the rectangle
                    new Scalar(0, 0, 255), // The color the rectangle is drawn in
                    6); // Thickness of the rectangle lines

            return input;

        }

    }

    private static class CargoDeterminationPipeline extends OpenCvPipeline {

        private enum CargoPosition {

            BOTTOM,
            MIDDLE,
            TOP

        }

        private static final int REGION_WIDTH = 80;
        private static final int REGION_HEIGHT = 80;
        private static final Point REGION_LEFT = new Point(-320, 0);
        private static final Point REGION_MID = new Point(-40, 0);
        private static final Point REGION_RIGHT = new Point(230, 0);

        // The core values which define the location and size of the sample regions
        private static final Point REGION_LEFT_TOPLEFT = new Point((int) (WIDTH / 2 + REGION_WIDTH / 2 + REGION_LEFT.x), (int) (HEIGHT / 2 + REGION_HEIGHT / 2 + REGION_LEFT.y));
        private static final Point REGION_MID_TOPLEFT = new Point((int) (WIDTH / 2 + REGION_WIDTH / 2 + REGION_MID.x), (int) (HEIGHT / 2 + REGION_HEIGHT / 2 + REGION_MID.y));
        private static final Point REGION_RIGHT_TOPLEFT = new Point((int) (WIDTH / 2 + REGION_WIDTH / 2 + REGION_RIGHT.x), (int) (HEIGHT / 2 + REGION_HEIGHT / 2 + REGION_RIGHT.y));

        // Regions
        private final Point REGION_LEFT_A = new Point(
                      REGION_LEFT_TOPLEFT.x,
                      REGION_LEFT_TOPLEFT.y);
        private final Point REGION_LEFT_B = new Point(
                   REGION_LEFT_TOPLEFT.x + REGION_WIDTH,
                   REGION_LEFT_TOPLEFT.y + REGION_HEIGHT);
        private final Point REGION_MID_A = new Point(
                REGION_MID_TOPLEFT.x,
                REGION_MID_TOPLEFT.y);
        private final Point REGION_MID_B = new Point(
                REGION_MID_TOPLEFT.x + REGION_WIDTH,
                REGION_MID_TOPLEFT.y + REGION_HEIGHT);
        private final Point REGION_RIGHT_A = new Point(
                REGION_RIGHT_TOPLEFT.x,
                REGION_RIGHT_TOPLEFT.y);
        private final Point REGION_RIGHT_B = new Point(
                REGION_RIGHT_TOPLEFT.x + REGION_WIDTH,
                REGION_RIGHT_TOPLEFT.y + REGION_HEIGHT);

        private Mat regionLeftCb;
        private Mat regionMidCb;
        private Mat regionRightCb;
        private Mat YCrCb = new Mat();
        private Mat Cb = new Mat();
        private int avgL;
        private int avgM;
        private int avgR;

        // Volatile since accessed by OpMode thread w/o synchronization
        private volatile CargoPosition position;

        private void inputToCb(Mat input) {

            // Convert to YCrCb
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);

            // Extracts the Cb channel to the 'Cb' variable
            Core.extractChannel(YCrCb, Cb, 1);

        }

        @Override
        public void init(Mat firstFrame) {

            inputToCb(firstFrame);
            regionLeftCb = Cb.submat(new Rect(REGION_LEFT_A, REGION_LEFT_B));
            regionMidCb = Cb.submat(new Rect(REGION_MID_A, REGION_MID_B));
            regionRightCb = Cb.submat(new Rect(REGION_RIGHT_A, REGION_RIGHT_B));

        }

        @Override
        public Mat processFrame(Mat input) {

            inputToCb(input);

            // The average cb value in the region
            avgL = (int) Core.mean(regionLeftCb).val[0];
            avgM = (int) Core.mean(regionMidCb).val[0];
            avgR = (int) Core.mean(regionRightCb).val[0];

            // Get position
            int max = Math.max(avgL, Math.max(avgM, avgR));
            if (max == avgL) position = CargoPosition.BOTTOM;
            else if (max == avgM) position = CargoPosition.MIDDLE;
            else position = CargoPosition.TOP;

            // Draw rectangle
            if (position == CargoPosition.BOTTOM) {

                Imgproc.rectangle(
                        input, // Buffer to draw on
                        REGION_LEFT_A, // First point which defines the rectangle
                        REGION_LEFT_B, // Second point which defines the rectangle
                        new Scalar(0, 0, 255), // The color the rectangle is drawn in
                        6); // Thickness of the rectangle lines

            } else if (position == CargoPosition.MIDDLE) {

                Imgproc.rectangle(
                        input, // Buffer to draw on
                        REGION_MID_A, // First point which defines the rectangle
                        REGION_MID_B, // Second point which defines the rectangle
                        new Scalar(0, 0, 255), // The color the rectangle is drawn in
                        6); // Thickness of the rectangle lines

            } else {

                Imgproc.rectangle(
                        input, // Buffer to draw on
                        REGION_RIGHT_A, // First point which defines the rectangle
                        REGION_RIGHT_B, // Second point which defines the rectangle
                        new Scalar(0, 0, 255), // The color the rectangle is drawn in
                        6); // Thickness of the rectangle lines

            }

            return input;

        }

        private CargoPosition getPosition() { return position; }

    }

}
