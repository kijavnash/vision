/* Copyright (c) 2023 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Mat;

import java.util.List;

// put inside NewAprilTag (as a static inner class)
class CroppedAprilTagProcessor implements VisionProcessor {
    private final org.opencv.core.Rect roi;   // region in full-frame pixels
    private final AprilTagProcessor delegate;

    public CroppedAprilTagProcessor(org.opencv.core.Rect roi, AprilTagProcessor delegate) {
        this.roi = roi;
        this.delegate = delegate;
    }

    // init: tell delegate the cropped resolution (so internal buffers match)
    @Override
    public void init(int width, int height, org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration calibration) {
        // initialize delegate using the ROI size and the same calibration object
        // (note: calibration is for full camera; pose will be affected if you don't adjust intrinsics)
        delegate.init(roi.width, roi.height, calibration);
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        // safe ROI inside frame bounds
        int x = Math.max(0, roi.x);
        int y = Math.max(0, roi.y);
        int w = Math.min(roi.width, frame.width() - x);
        int h = Math.min(roi.height, frame.height() - y);

        if (w <= 0 || h <= 0) {
            // fallback: process full frame if ROI invalid
            return delegate.processFrame(frame, captureTimeNanos);
        }

        // submat = view into the original Mat (no copy)
        Mat cropped = frame.submat(new org.opencv.core.Rect(x, y, w, h));

        // Let the AprilTag processor handle the cropped image
        Object userContext = delegate.processFrame(cropped, captureTimeNanos);

        // Adjust detected centers from cropped coords -> full-frame coords
        // (delegate.getDetections() returns detections in cropped image coordinates)
        List<AprilTagDetection> dets = delegate.getDetections();
        for (AprilTagDetection d : dets) {
            d.center.x += x;
            d.center.y += y;
        }

        // return whatever the delegate returned (usually null)
        return userContext;
    }

    @Override
    public void onDrawFrame(android.graphics.Canvas canvas, int onscreenWidth, int onscreenHeight,
                            float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        // Draw ROI box (red) and simple tag markers (green)
        android.graphics.Paint roiPaint = new android.graphics.Paint();
        roiPaint.setColor(android.graphics.Color.RED);
        roiPaint.setStyle(android.graphics.Paint.Style.STROKE);
        roiPaint.setStrokeWidth(5);

        canvas.drawRect(
                roi.x * scaleBmpPxToCanvasPx,
                roi.y * scaleBmpPxToCanvasPx,
                (roi.x + roi.width) * scaleBmpPxToCanvasPx,
                (roi.y + roi.height) * scaleBmpPxToCanvasPx,
                roiPaint
        );

        // draw tiny markers for each detection (centers already shifted to full-frame coords)
        android.graphics.Paint p = new android.graphics.Paint();
        p.setColor(android.graphics.Color.GREEN);
        p.setStyle(android.graphics.Paint.Style.FILL);
        p.setTextSize(18 * scaleCanvasDensity);

        List<AprilTagDetection> dets = delegate.getDetections();
        for (AprilTagDetection d : dets) {
            float cx = (float) (d.center.x * scaleBmpPxToCanvasPx);
            float cy = (float) (d.center.y * scaleBmpPxToCanvasPx);
            canvas.drawCircle(cx, cy, 6, p);
            canvas.drawText(String.valueOf(d.id), cx + 8, cy - 8, p);
        }
    }

    // expose detections if you want to read them (delegates to underlying processor)
    public List<AprilTagDetection> getDetections() {
        return delegate.getDetections();
    }
}



/*
 * This OpMode illustrates the basics of AprilTag recognition and pose estimation,
 * including Java Builder structures for specifying Vision parameters.
 *
 * For an introduction to AprilTags, see the FTC-DOCS link below:
 * https://ftc-docs.firstinspires.org/en/latest/apriltag/vision_portal/apriltag_intro/apriltag-intro.html
 *
 * In this sample, any visible tag ID will be detected and displayed, but only tags that are included in the default
 * "TagLibrary" will have their position and orientation information displayed.  This default TagLibrary contains
 * the current Season's AprilTags and a small set of "test Tags" in the high number range.
 *
 * When an AprilTag in the TagLibrary is detected, the SDK provides location and orientation of the tag, relative to the camera.
 * This information is provided in the "ftcPose" member of the returned "detection", and is explained in the ftc-docs page linked below.
 * https://ftc-docs.firstinspires.org/apriltag-detection-values
 *
 * To experiment with using AprilTags to navigate, try out these two driving samples:
 * RobotAutoDriveToAprilTagOmni and RobotAutoDriveToAprilTagTank
 *
 * There are many "default" VisionPortal and AprilTag configuration parameters that may be overridden if desired.
 * These default parameters are shown as comments in the code below.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 */
@TeleOp(name = "NewAprilTag", group = "TeleOp")

public class NewAprilTag extends LinearOpMode {


    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    /**
     * The variable to store our instance of the AprilTag processor.
     */
    private AprilTagProcessor aprilTag;


    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;

    @Override
    public void runOpMode() {

        initAprilTag();

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch START to start OpMode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {

                telemetryAprilTag();

                // Push telemetry to the Driver Station.
                telemetry.update();

                // Save CPU resources; can resume streaming when needed.
                if (gamepad1.dpad_down) {
                    visionPortal.stopStreaming();
                } else if (gamepad1.dpad_up) {
                    visionPortal.resumeStreaming();
                }

                // Share the CPU.
                sleep(20);
            }
        }

        // Save more CPU resources when camera is no longer needed.
        visionPortal.close();

    }   // end method runOpMode()

    /**
     * Initialize the AprilTag processor.
     */

    private void initAprilTag() {
        // Build AprilTag processor (turn off internal drawing so wrapper draws)
         aprilTag = new AprilTagProcessor.Builder()
                .build();

        // choose ROI (example: center 320x240)
        org.opencv.core.Rect roi = new org.opencv.core.Rect(160, 120, 320, 240);

        // Wrap the raw AprilTag processor with cropping behavior
        //CroppedAprilTagProcessor croppedProcessor = new CroppedAprilTagProcessor(roi, rawAprilTag);

        // Build vision portal
        VisionPortal.Builder builder = new VisionPortal.Builder();
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }
        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);

        // add the cropped wrapper (it calls the rawAprilTag internally)
        //builder.addProcessor(croppedProcessor);

        visionPortal = builder.build();

        // keep a reference to the raw AprilTag so your telemetry code (aprilTag.getDetections()) still works
        // NOTE: we assigned rawAprilTag earlier; keep using aprilTag variable for telemetry:
        //aprilTag = rawAprilTag;
    }

   // end method initAprilTag()


    /**
     * Add telemetry about AprilTag detections.
     */
    private void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");

    }   // end method telemetryAprilTag()

}   // end class
