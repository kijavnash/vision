package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

class ColorPickerPipeline extends OpenCvPipeline
{
    public Scalar avgHSV = new Scalar(0,0,0);
    public Scalar avgYCrCb = new Scalar(0,0,0);
    public Scalar avgLab = new Scalar(0,0,0);

    @Override
    public Mat processFrame(Mat input)
    {
        int width = input.width();
        int height = input.height();

        int roiWidth = (int)(width * 0.1);
        int roiHeight = (int)(height * 0.1);
        int x = (width - roiWidth) / 2;
        int y = (height - roiHeight) / 2;
        Rect roiRect = new Rect(x, y, roiWidth, roiHeight);

        Mat hsv = new Mat();
        Mat ycrcb = new Mat();
        Mat lab = new Mat();
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);
        Imgproc.cvtColor(input, ycrcb, Imgproc.COLOR_RGB2YCrCb);
        Imgproc.cvtColor(input, lab, Imgproc.COLOR_RGB2Lab);

        Mat hsvRoi = hsv.submat(roiRect);
        Mat ycrcbRoi = ycrcb.submat(roiRect);
        Mat labRoi = lab.submat(roiRect);

        avgHSV = Core.mean(hsvRoi);
        avgYCrCb = Core.mean(ycrcbRoi);
        avgLab = Core.mean(labRoi);
        Imgproc.putText(input,
                "HSV: " + (int)avgHSV.val[0] + ", " + (int)avgHSV.val[1] + ", " + (int)avgHSV.val[2],
                new org.opencv.core.Point(10, 30),
                Imgproc.FONT_HERSHEY_SIMPLEX, 0.6, new Scalar(255,255,255), 2);

        Imgproc.putText(input,
                "YCrCb: " + (int)avgYCrCb.val[0] + ", " + (int)avgYCrCb.val[1] + ", " + (int)avgYCrCb.val[2],
                new org.opencv.core.Point(10, 60),
                Imgproc.FONT_HERSHEY_SIMPLEX, 0.6, new Scalar(255,255,255), 2);

        Imgproc.putText(input,
                "Lab: " + (int)avgLab.val[0] + ", " + (int)avgLab.val[1] + ", " + (int)avgLab.val[2],
                new org.opencv.core.Point(10, 90),
                Imgproc.FONT_HERSHEY_SIMPLEX, 0.6, new Scalar(255,255,255), 2);


        Imgproc.rectangle(input, roiRect, new Scalar(0,255,0), 2);

        hsv.release();
        ycrcb.release();
        lab.release();
        hsvRoi.release();
        ycrcbRoi.release();
        labRoi.release();
        return input;
    }
}

@TeleOp
public class ColorDetection extends LinearOpMode
{
    OpenCvWebcam webcam;

    @Override
    public void runOpMode() throws InterruptedException
    {
        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id",
                        hardwareMap.appContext.getPackageName());

        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"),
                cameraMonitorViewId);

        // Keep pipeline reference
        ColorPickerPipeline pipeline = new ColorPickerPipeline();
        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addLine("Camera failed to open!");
            }
        });

        telemetry.addLine("Waiting for start...");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Javi", "Hello");
            telemetry.addData("FPS", webcam.getFps());
            telemetry.update();

            sleep(50);
        }
    }
}
