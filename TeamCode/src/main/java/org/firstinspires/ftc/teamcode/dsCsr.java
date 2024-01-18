package org.firstinspires.ftc.teamcode;

import android.util.Size;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.processors.tseSaturationProcessor;

import org.firstinspires.ftc.vision.VisionPortal;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.opencv.core.Scalar;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import java.lang.Math;
import java.util.ArrayList;

@Disabled
@Autonomous(name="Distance Sensor Auton Red", group="Auton")
public class dsCsr extends LinearOpMode {
    int detectedLine = -1;
    private VisionPortal visionPortal;
    private ColourMassDetectionProcessorRed colourMassDetectionProcessor;
    Scalar lower = new Scalar(140, 60, 0); // the lower hsv threshold for your detection
    Scalar upper = new Scalar(180, 255, 255);  // the upper hsv threshold for your detection

    double minArea = 150;
    BotHardwareNew robot = new BotHardwareNew();
    public ElapsedTime runtime = new ElapsedTime();
    public boolean SIDE = true;
    public int slide = 1;
    public boolean position = true;



    @Override
    public void runOpMode() {


            colourMassDetectionProcessor = new ColourMassDetectionProcessorRed(
                    lower,
                    upper,
                    () -> minArea, // these are lambda methods, in case we want to change them while the match is running, for us to tune them or something
                    () -> 213, // the left dividing line, in this case the left third of the frame
                    () -> 426 // the left dividing line, in this case the right third of the frame
            );
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1")) // the camera on your robot is named "Webcam 1" by default
                    .addProcessor(colourMassDetectionProcessor)
                    .build();
        robot.init(hardwareMap);
        while(opModeInInit()){
            telemetry.addData("Currently Recorded Position", colourMassDetectionProcessor.getRecordedPropPosition());
            telemetry.addData("Camera State", visionPortal.getCameraState());
            telemetry.addData("Currently Detected Mass Center", "x: " + colourMassDetectionProcessor.getLargestContourX() + ", y: " + colourMassDetectionProcessor.getLargestContourY());
            telemetry.addData("Currently Detected Mass Area", colourMassDetectionProcessor.getLargestContourArea());
            telemetry.update();
            robot.L1.setPosition(robot.OUTTAKEA_CLOSE);
            robot.L2.setPosition(robot.OUTTAKEB_CLOSE);
        }
        waitForStart();
        telemetry.addData("After wait for start", true);
        telemetry.update();
        // gets the recorded prop position
        ColourMassDetectionProcessorRed.PropPositions recordedPropPosition = colourMassDetectionProcessor.getRecordedPropPosition();

        // now we can use recordedPropPosition to determine where the prop is! if we never saw a prop, your recorded position will be UNFOUND.
        // if it is UNFOUND, you can manually set it to any of the other positions to guess
        if (recordedPropPosition == ColourMassDetectionProcessorRed.PropPositions.UNFOUND) {
            recordedPropPosition = ColourMassDetectionProcessorRed.PropPositions.MIDDLE;
        }
        visionPortal.stopLiveView();
        visionPortal.stopStreaming();
        // now we can use recordedPropPosition in our auto code to modify where we place the purple and yellow pixels
        switch (recordedPropPosition) {
            case LEFT:
                // code to do if we saw the prop on the left

//                robot.UpdateA(500);
//                robot.servoRight(1000);

                robot.LIFT_POWER = -1;
                robot.UpdateA(0);
//                robot.servoRight(500);

                break;
            case UNFOUND: // we can also just add the unfound case here to do fallthrough intstead of the overriding method above, whatever you prefer!

                break;
            case MIDDLE:


                // code to do if we saw the prop on the middle
                break;
            case RIGHT:
                // code to do if we saw the prop on the right
                break;
        }

        }

        ///// TRAJECTORIES /////
        //FORWARD


        //////START STATE MACHINE//////








    public void telemetry(){
        if(slide == 1 && opModeInInit()) {
            telemetry.addData("Side is red = ", SIDE);
            telemetry.update();
        }
        if(slide == 2 && opModeInInit()) {
            telemetry.addData("Are you by the human player area = ", position);
            telemetry.update();
        }
        telemetry.addData("Currently Recorded Position", colourMassDetectionProcessor.getRecordedPropPosition());
        telemetry.addData("Camera State", visionPortal.getCameraState());
        telemetry.addData("Currently Detected Mass Center", "x: " + colourMassDetectionProcessor.getLargestContourX() + ", y: " + colourMassDetectionProcessor.getLargestContourY());
        telemetry.addData("Currently Detected Mass Area", colourMassDetectionProcessor.getLargestContourArea());
    }

}