package org.firstinspires.ftc.teamcode;
import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionPortalImpl;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Scalar;


@TeleOp
public class testFTC extends LinearOpMode{
    private ColourMassDetectionProcessorRed colourMassDetectionProcessor;
    Scalar lower = new Scalar(0, 60, 0); // the lower hsv threshold for your detection
    Scalar upper = new Scalar(30, 255, 255);
    double minArea = 150;
    @Override
    public void runOpMode(){
        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .build();
        colourMassDetectionProcessor = new ColourMassDetectionProcessorRed(
                lower,
                upper,
                () -> minArea, // these are lambda methods, in case we want to change them while the match is running, for us to tune them or something
                () -> 213, // the left dividing line, in this case the left third of the frame
                () -> 426 // the left dividing line, in this case the right third of the frame
        );
        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .addProcessor(colourMassDetectionProcessor)
                .setCamera(hardwareMap.get(WebcamName.class,"Webcam 1"))
                .setCameraResolution(new Size(640,480))
                .enableLiveView(true)
                .build();

        visionPortal.resumeStreaming();
        while(opModeInInit()){
            visionPortal.setProcessorEnabled(colourMassDetectionProcessor, false);
            if(tagProcessor.getDetections().size()>0){
                AprilTagDetection tag = tagProcessor.getDetections().get(0);
                telemetry.addData("Currently Recorded Position", colourMassDetectionProcessor.getRecordedPropPosition());
                telemetry.addData("Camera State", visionPortal.getCameraState());
                telemetry.addData("Currently Detected Mass Center", "x: " + colourMassDetectionProcessor.getLargestContourX() + ", y: " + colourMassDetectionProcessor.getLargestContourY());
                telemetry.addData("Currently Detected Mass Area", colourMassDetectionProcessor.getLargestContourArea());
                telemetry.addData("ID: ", tag.id);
                telemetry.addData("X: ", (int)tag.ftcPose.x);
                telemetry.addData("Y: ", tag.ftcPose.y);

                telemetry.update();
            }
        }
    }
}