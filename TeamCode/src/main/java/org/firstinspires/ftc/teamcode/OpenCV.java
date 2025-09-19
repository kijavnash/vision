/*
 * Converted FTC EasyOpenCV pipeline to FTC VisionPortal Builder + VisionProcessor
 * Your Imgproc code is preserved – only the camera/pipeline setup changes.
 */

//HOLY IMPORTS
package org.firstinspires.ftc.teamcode;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.Stream;


// TO DO  = go through all the comments
@TeleOp(name="OpenCV")
public class OpenCV extends LinearOpMode
{
    private VisionPortal visionPortal;
    private MyProcessor processor;

    @Override
    public void runOpMode()
    {
        // figure out what a processor is
        processor = new MyProcessor();

        // VisionPortal = this is the new stuff
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG) // Force MJPEG instead of YUY2
                .addProcessor(processor)
                .build();


        telemetry.addLine("Waiting for start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive())
        {
            // Access results from processor
            List<MatOfPoint> polys = processor.getLastPolygons();
            if (!polys.isEmpty()) {
                Rect rect = Imgproc.boundingRect(polys.get(0));
                telemetry.addData("height", rect.height);
                telemetry.addData("Width", rect.width);
            } else {
                telemetry.addLine("No polygons found");
            }
            telemetry.update();

            sleep(100);
        }

        // Cleanup
        visionPortal.close();
    }

    // ===================== VisionProcessor =====================
    static class MyProcessor implements VisionProcessor {
        private Mat hsvMat = new Mat();
        private Mat thresholdedMat = new Mat();
        // filters out all contours below the minArea
        private double minArea = 1000;
        private int kernelSize = 11;

        // ADJUST FOR BRIGHTNESS LEVELS
        private Scalar lowerBound = new Scalar(65, 100, 50);  // Example HSV lower bound
        private Scalar upperBound = new Scalar(100, 255, 255);

        private List<MatOfPoint> lastPolygons = new ArrayList<>();

        @Override
        public void init(int width, int height, CameraCalibration calibration) {
            // it crashes out without this so I gotta figure this out
        }

        @Override
        public Object processFrame(Mat frame, long captureTimeNanos) {
            Imgproc.cvtColor(frame, hsvMat, Imgproc.COLOR_RGB2HSV);

            Core.inRange(hsvMat, lowerBound, upperBound, thresholdedMat);

            List<MatOfPoint> contours = new ArrayList<>();
            Imgproc.findContours(thresholdedMat.clone(), contours, new Mat(),
                    Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
            Imgproc.drawContours(frame, contours, -1, new Scalar(0, 255, 0), 2);

            // MAIN SHIT
            lastPolygons = findCircles(thresholdedMat);

            return null;
        }

        @Override
        public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                                float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
            Paint paint = new Paint();
            paint.setColor(Color.BLUE);
            paint.setStrokeWidth(3);

            for (MatOfPoint poly : lastPolygons) {
                Point[] pts = poly.toArray();
                for (int i = 0; i < pts.length; i++) {
                    Point p1 = pts[i];
                    Point p2 = pts[(i+1) % pts.length];
                    canvas.drawLine(
                            (float) (p1.x * scaleBmpPxToCanvasPx),
                            (float) (p1.y * scaleBmpPxToCanvasPx),
                            (float) (p2.x * scaleBmpPxToCanvasPx),
                            (float) (p2.y * scaleBmpPxToCanvasPx),
                            paint
                    );
                }
            }
        }


        // return all the polygns
        public List<MatOfPoint> getLastPolygons() {
            return lastPolygons;
        }

        // From old code
        // Replaces findUnifiedQuadrilateral, but same signature.
        private List<MatOfPoint> findCircles(Mat binaryMask) {
            // HARD CODED VALUES - subject to change
            /*
            the way the function works is through an accumulator array
            accumulator array = 3d matrix that stores the x value, y value and radius of all possible circles
            dp is the size of the accumulator array relative to image size
            the function then picks the circle with the most tallies accros the pixels

            1.0 = accumulator has same resolution as image (slow but accurate)
            2.0 = accumulator has half the resolution of the image (faster but less accurate)

            should NOT be changed by brightness
            */
            double dp = 1.0;

            // minimum distance two theoretical circles can be next to each other as calculated by the function

            // should NOT be changed by brightness
            double minDist = 50.0;

            /*
            canny param = edge detection
            the way it works is it converts image to grayscale and then checks if there are any brightness gaps
            the higher the canny param, the larger the difference in brightness needs to be
            this means it will eliminate more noise but detect less circles
            upper threshold = cannyParam, lower threshold is 1/2 of upper threshold

            SHOULD be changed by brightness
            */
            double cannyParam = 100.0;

            /*
            accumulator param = voting
            it's the minimum number of votes a circle needs to accumulate to be considered by the funtcion

            SHOULD be changed by brightness
            */
            double accuParam = 30.0;

            // only picks a circle with radius in between these values

            // should NOT be changed by brightness
            int minRadius = 10;
            int maxRadius = 200;

            List<MatOfPoint> circlesAsPolys = new ArrayList<>();

            // === NEW: Morphological cleanup ===
            Mat morphed = new Mat();
            Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, new Size(5, 5));
            Imgproc.erode(binaryMask, morphed, kernel);
            Imgproc.dilate(morphed, morphed, kernel);

            Mat gray = new Mat();
            // HoughCircles works on grayscale — make sure input is grayscale.
            Imgproc.cvtColor(binaryMask, gray, Imgproc.COLOR_GRAY2BGR);
            // Apply erosion and dilation to clean up noise
            // binaryMask may be single-channel
            Imgproc.cvtColor(gray, gray, Imgproc.COLOR_BGR2GRAY);

            // blurred image to reduce noise
            Mat blurred = new Mat();
            Imgproc.GaussianBlur(gray, blurred, new Size(9, 9), 2, 2);

            // output mat
            Mat circles = new Mat();
            Imgproc.HoughCircles(
                    blurred,
                    circles,
                    Imgproc.HOUGH_GRADIENT, // this is the openCV method
                    dp,      // dp (accumulator resolution)
                    minDist,     // minDist between circle centers
                    cannyParam,    // param1 (Canny high threshold)
                    accuParam,     // param2 (accumulator threshold)
                    minRadius,       // minRadius
                    maxRadius       // maxRadius
            );

            if (!circles.empty()) {
                for (int i = 0; i < circles.cols(); i++) {
                    double[] data = circles.get(0, i);
                    if (data == null || data.length < 3) continue;
                    double cx = data[0];
                    double cy = data[1];
                    double r = data[2];

                    // Approximate circle as polygon (20-sided by default)
                    int steps = 20;
                    Point[] pts = new Point[steps];
                    for (int j = 0; j < steps; j++) {
                        double theta = 2 * Math.PI * j / steps;
                        pts[j] = new Point(
                                cx + r * Math.cos(theta),
                                cy + r * Math.sin(theta)
                        );
                    }
                    circlesAsPolys.add(new MatOfPoint(pts));
                }
            }

            return circlesAsPolys;
        }
    }
}
