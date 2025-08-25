package org.firstinspires.ftc.teamcode.vision;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class ColorDetection extends OpenCvPipeline {
    public enum DominantColor {
        RED, GREEN, BLUE, UNKNOWN
    }

    public volatile DominantColor color = DominantColor.UNKNOWN;

    @Override
    public Mat processFrame(Mat input) {
        Mat blurred = new Mat();
        Imgproc.GaussianBlur(input, blurred, new Size(5, 5), 0);

        // Calculate average color
        Scalar avgColor = Core.mean(blurred);

        double red = avgColor.val[0];
        double green = avgColor.val[1];
        double blue = avgColor.val[2];

        if (red > green && red > blue) {
            color = DominantColor.RED;
        } else if (green > red && green > blue) {
            color = DominantColor.GREEN;
        } else if (blue > red && blue > green) {
            color = DominantColor.BLUE;
        } else {
            color = DominantColor.UNKNOWN;
        }

        return input;
    }
}
