package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.videoio.VideoCapture;
import org.opencv.core.Mat;
@TeleOp
public class detection extends LinearOpMode {

    @Override
    public void runOpMode() {
        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .build();
        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640,480))
                .enableLiveView(true)
                .build();

        // OpenCV VideoCapture for retrieving camera frames
        VideoCapture capture = new VideoCapture();
        capture.open(0); // Use 0 for webcam, or provide video file path

        // Initialize robot detection pipeline
        robotDetection robotDetectionPipeline = new robotDetection();

        waitForStart();
        visionPortal.resumeStreaming();
        while (!isStopRequested() && opModeIsActive()) {
            // Get AprilTag detections
            if (tagProcessor.getDetections().size() > 0) {
                AprilTagDetection tag = tagProcessor.getDetections().get(0);

                // Read frame from camera
                Mat frame = new Mat();
                capture.read(frame);

                // Check if robot is detected
                boolean robotDetected = robotDetectionPipeline.detectRobot(frame);

                telemetry.addData("Robot Detected: ", robotDetected);
                telemetry.update();
            }
        }

        capture.release(); // Release VideoCapture resources
    }
}
