package org.firstinspires.ftc.teamcode.vampire.hardware;

import com.acmerobotics.dashboard.config.Config;
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

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

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
        webcam.setPipeline(cargoPipeline);
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
        print("HSV Difference", duckPipeline.HSVDiff);
        print("X", duckPipeline.getXNum());
        print("Y", duckPipeline.getYNum());
        print("Inch X", duckPipeline.getXDist());
        print("Inch Depth", duckPipeline.getDepth());
        print("Total Distance", duckPipeline.getTotalDist());
        print("Angle", duckPipeline.getAngle());
        print("Average HSV", duckPipeline.desiredHSV[0] + " " + duckPipeline.desiredHSV[1] + " " + duckPipeline.desiredHSV[2]);
        print("Test HSV", duckPipeline.testHSV[0] + " " + duckPipeline.testHSV[1] + " " + duckPipeline.testHSV[2]);
        print("Position", getCargoPos());
        print("Cargo Mode", isCargo);

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

    public double[] getDuckPose() { return new double[] { duckPipeline.getXDist(), duckPipeline.getDepth(), duckPipeline.getAngle() }; }

    public double getDuckDistance() { return Math.hypot(getDuckPose()[0], getDuckPose()[1]); }

    @Config
    public static class DuckPositionPipeline extends OpenCvPipeline {

        public static int REGION_WIDTH = 25;
        public static int REGION_HEIGHT = 25;
        public static int TOP_Y = 8;
        private static final int NUM_HORZ = WIDTH / REGION_WIDTH;
        private static final int NUM_VERT = HEIGHT / REGION_HEIGHT;
        private static final Mat[][][] regions = new Mat[3][NUM_HORZ][NUM_VERT]; // H = 0, S = 1, V = 2
        private Mat hsv = new Mat();
        private Mat[] hsvArray = new Mat[] { new Mat(), new Mat(), new Mat() };
        private static final int[] GOAL = new int[] { 40, 100, 180 };

        // For testing
        private static final int TEST_X = 10;
        private static final int TEST_Y = 9;
        private int[] testHSV = new int[3];

        // Calculate x, y, HSV
        private int[] desiredHSV = new int[3];
        private double HSVDiff = 0;
        public static double THRESHOLD_DIFF = 45;
        private int xNum = 0;
        private int yNum = 0;
        private static final int AVG_NUM = 30;
        private ArrayList<Integer> listX = new ArrayList<>();
        private Map<Integer, Integer> mapX = new HashMap<>();
        private ArrayList<Integer> listY = new ArrayList<>();
        private Map<Integer, Integer> mapY = new HashMap<>();

        // Calculate duck position
        public static double CAMERA_HEIGHT = 10.5;
        public static double MID_DEPTH = 28;
        public static double MID_DIFF = 2.85;
        public static double X_OFFSET = 0;
        public static double HALF_ANGLE = 0.5;

        private double getVertAngle(int index) {

            return Math.atan2(CAMERA_HEIGHT, getDepth(index));

        }

        private double getDiff(int index) {

            if (Math.abs(index) == 1) return MID_DIFF;
            double lastAngle;
            if (index < 0) lastAngle = getVertAngle(index + 1);
            else lastAngle = getVertAngle(index - 1);
            double firstAngle = getVertAngle(0);
            double numerator = MID_DIFF * Math.sin(firstAngle) * Math.sin(Math.PI / 2 + firstAngle - lastAngle);
            double denominator = Math.sin(lastAngle);
            return numerator / denominator;

        }

        private double getDepth(int index) {

            if (index == 0) return MID_DEPTH;
            else if (index < 0) return getDepth(index + 1) - getDiff(index);
            else return getDepth(index - 1) + getDiff(index);

        }

        private void inputToHSV(Mat input) {

            // Convert to hsv
            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);
            Core.extractChannel(hsv, hsvArray[0], 0);
            Core.extractChannel(hsv, hsvArray[1], 1);
            Core.extractChannel(hsv, hsvArray[2], 2);

        }

        public int getXNum() {

            if (mapX.size() == 0) return 0;

            Map.Entry<Integer, Integer> max = null;
            for (Map.Entry<Integer, Integer> e : mapX.entrySet())
                if (max == null || e.getValue() > max.getValue()) max = e;

            return max.getKey();

        }

        public int getYNum() {

            if (mapY.size() == 0) return 0;

            Map.Entry<Integer, Integer> max = null;
            for (Map.Entry<Integer, Integer> e : mapY.entrySet())
                if (max == null || e.getValue() > max.getValue()) max = e;

            return max.getKey();

        }

        private double getXPerBox() { return (2 * getDepth() * Math.tan(HALF_ANGLE)) / NUM_HORZ; }

        public double getXDist() { return getXNum() == -1 ? 0 : getXPerBox() * (getXNum() - NUM_HORZ / 2) + X_OFFSET; }

        public double getDepth() { return getXNum() == -1 ? 0 : getDepth(NUM_VERT / 2 - getYNum()); }

        public double getTotalDist() { return Math.hypot(getXDist(), getDepth()); }

        public double getAngle() { return Math.atan2(getXDist(), getDepth()); }

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
            boolean isDuck = false;
            for (int v = TOP_Y; v < NUM_VERT; v++) {

                // From the top so it only considers the closest duck
                for (int h = 0; h < NUM_HORZ; h++) {

                    int avgHue = (int) Core.mean(regions[0][h][v]).val[0];
                    int avgSaturation = (int) Core.mean(regions[1][h][v]).val[0];
                    int avgBrightness = (int) Core.mean(regions[2][h][v]).val[0];

                    // Calculate difference between goal hue and calculated average hue
                    double diff = Math.hypot(Math.hypot(GOAL[0] - avgHue, GOAL[1] - avgSaturation), GOAL[2] - avgBrightness);
                    if (diff < THRESHOLD_DIFF) {

                        HSVDiff = diff;
                        desiredHSV[0] = avgHue;
                        desiredHSV[1] = avgSaturation;
                        desiredHSV[2] = avgBrightness;

                        xNum = h;
                        yNum = v;
                        isDuck = true;

                    }

                }

            }

            // Check test
            for (int i = 0; i < 3; i++) testHSV[i] = (int) Core.mean(regions[i][TEST_X][TEST_Y]).val[0];

            // Add to avgs if duck exists
            if (!isDuck) {

                xNum = -1;
                yNum = -1;

            }

            listX.add(xNum);
            listY.add(yNum);

            // Add to map to calculate mode
            Integer freqX = mapX.get(xNum);
            mapX.put(xNum, freqX == null ? 1 : freqX + 1);
            Integer freqY = mapY.get(yNum);
            mapY.put(yNum, freqY == null ? 1 : freqY + 1);

            // Delete the extra
            if (listX.size() > AVG_NUM) {

                freqX = mapX.get(listX.get(0));
                mapX.put(listX.get(0), freqX - 1);
                listX.remove(0);

            }
            if (listY.size() > AVG_NUM) {

                freqY = mapY.get(listY.get(0));
                mapY.put(listY.get(0), freqY - 1);
                listY.remove(0);

            }

            // Draw test rectangle
            Imgproc.rectangle(
                    input,
                    new Point(TEST_X * REGION_WIDTH, TEST_Y * REGION_HEIGHT),
                    new Point((TEST_X + 1) * REGION_WIDTH, (TEST_Y + 1) * REGION_HEIGHT),
                    new Scalar(0, 255, 0),
                    6);

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
