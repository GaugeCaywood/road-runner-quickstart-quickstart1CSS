package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.opencv.objdetect.CascadeClassifier;
@Disabled
public class ObjectDetectionPipeline {
    private CascadeClassifier objectClassifier;

    public ObjectDetectionPipeline(String classifierPath) {
        // Load the object detection classifier from a trained XML file
        objectClassifier = new CascadeClassifier(classifierPath);
    }

    public Rect detectObject(Mat inputFrame) {
        Mat grayFrame = new Mat();
        MatOfRect objectDetections = new MatOfRect();

        // Convert the input frame to grayscale for object detection
        Imgproc.cvtColor(inputFrame, grayFrame, Imgproc.COLOR_RGBA2GRAY);

        // Detect objects in the grayscale frame
        objectClassifier.detectMultiScale(grayFrame, objectDetections);

        // Return the first detected object (you can modify this logic as needed)
        Rect[] objectsArray = objectDetections.toArray();
        if (objectsArray.length > 0) {
            return objectsArray[0];
        } else {
            return null; // No object detected
        }
    }
}
