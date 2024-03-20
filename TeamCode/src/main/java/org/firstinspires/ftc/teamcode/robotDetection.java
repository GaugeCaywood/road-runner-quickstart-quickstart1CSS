package org.firstinspires.ftc.teamcode;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

public class robotDetection {

    public static boolean detectRobot(Mat frame) {
        // Convert frame to grayscale for adaptive thresholding
        Mat grayFrame = new Mat();
        Imgproc.cvtColor(frame, grayFrame, Imgproc.COLOR_BGR2GRAY);

        // Perform adaptive thresholding to obtain binary image
        Mat binaryFrame = new Mat();
        Imgproc.adaptiveThreshold(grayFrame, binaryFrame, 255, Imgproc.ADAPTIVE_THRESH_MEAN_C, Imgproc.THRESH_BINARY_INV, 11, 10);

        // Find contours in the binary image
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(binaryFrame, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        // Filter contours based on area and aspect ratio
        List<MatOfPoint> filteredContours = new ArrayList<>();
        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            Rect rect = Imgproc.boundingRect(contour);
            double aspectRatio = (double) rect.width / rect.height;

            if (area > 100 && aspectRatio > 0.5 && aspectRatio < 2) {
                filteredContours.add(contour);
            }
        }

        // If any valid contours are found, consider the robot detected
        return !filteredContours.isEmpty();
    }
}
