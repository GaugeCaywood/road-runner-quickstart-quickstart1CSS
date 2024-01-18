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
@Autonomous(name="Distance Sensor Auton Blue", group="Auton")
public class dsCs extends LinearOpMode {
    int detectedLine = -1;
    private VisionPortal visionPortal;
    private ColourMassDetectionProcessor colourMassDetectionProcessor;
    Scalar lower = new Scalar(110, 70, 20); // the lower hsv threshold for your detection
    Scalar upper = new Scalar(140, 255, 255);  // the upper hsv threshold for your detection
    double minArea = 1000;
    BotHardwareNew robot = new BotHardwareNew();

    public ElapsedTime runtime = new ElapsedTime();
    public boolean SIDE = true;
    public int slide = 1;
    public boolean position = true;

    public enum robotState {
        FORWARDTOPIXEL,
        PLACEONBACKDROP,
        IDLE
    }

    robotState currentState = robotState.IDLE;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);


        colourMassDetectionProcessor = new ColourMassDetectionProcessor(
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
        while(!opModeIsActive() && isStopRequested()){
            telemetry.addData("Currently Recorded Position", colourMassDetectionProcessor.getRecordedPropPosition());
            telemetry.addData("Camera State", visionPortal.getCameraState());
            telemetry.addData("Currently Detected Mass Center", "x: " + colourMassDetectionProcessor.getLargestContourX() + ", y: " + colourMassDetectionProcessor.getLargestContourY());
            telemetry.addData("Currently Detected Mass Area", colourMassDetectionProcessor.getLargestContourArea());
        }
        waitForStart();
        starts();
    }

    ///// TRAJECTORIES /////
    //FORWARD


    //////START STATE MACHINE//////







    public void starts() {
        // shuts down the camera once the match starts, we dont need to look any more
        if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
            visionPortal.stopLiveView();
            visionPortal.stopStreaming();
        }

        // gets the recorded prop position
        ColourMassDetectionProcessor.PropPositions recordedPropPosition = colourMassDetectionProcessor.getRecordedPropPosition();

        // now we can use recordedPropPosition to determine where the prop is! if we never saw a prop, your recorded position will be UNFOUND.
        // if it is UNFOUND, you can manually set it to any of the other positions to guess
        if (recordedPropPosition == ColourMassDetectionProcessor.PropPositions.UNFOUND) {
            recordedPropPosition = ColourMassDetectionProcessor.PropPositions.MIDDLE;
        }

        // now we can use recordedPropPosition in our auto code to modify where we place the purple and yellow pixels
        switch (recordedPropPosition) {
            case LEFT:
                // code to do if we saw the prop on the left

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
    public void stops() {
        // this closes down the portal when we stop the code, its good practice!
        colourMassDetectionProcessor.close();
        visionPortal.close();
    }
    public void telemetry(){
        telemetry.addData("Currently Recorded Position", colourMassDetectionProcessor.getRecordedPropPosition());
        telemetry.addData("Camera State", visionPortal.getCameraState());
        telemetry.addData("Currently Detected Mass Center", "x: " + colourMassDetectionProcessor.getLargestContourX() + ", y: " + colourMassDetectionProcessor.getLargestContourY());
        telemetry.addData("Currently Detected Mass Area", colourMassDetectionProcessor.getLargestContourArea());
    }
    public void forwardDs(int distance, double speed, int liftValue){
        while(distance > robot.ds.getDistance(DistanceUnit.INCH)){
            robot.forward(speed);
            robot.UpdateLifts(liftValue);
        }
        robot.stop();
    }
    public void turnDs(int distance, double speed, int liftValue){
        while(distance > robot.ds.getDistance(DistanceUnit.INCH)){
            robot.turnLeft(speed);
            robot.UpdateLifts(liftValue);
        }
        robot.stop();
    }
    public void strafeDs(int distance, double speed, int liftValue){
        while(distance > robot.ds.getDistance(DistanceUnit.INCH)){
            robot.left(speed);
            robot.UpdateLifts(liftValue);
        }
        robot.stop();
    }
    public void forwardTime(double time, double speed, int liftValue){
        while(time < runtime.milliseconds()){
            robot.forward(speed);
            robot.UpdateLifts(liftValue);
        }
        robot.stop();
    }
    public void strafeTime(double time, double speed, int liftValue){
        while(time < runtime.milliseconds()){
            robot.left(speed);
            robot.UpdateLifts(liftValue);
        }
        robot.stop();
    }
    public void turnTime(int time, double speed, int liftValue){
        while(time < runtime.milliseconds()){
            robot.turnLeft(speed);
            robot.UpdateLifts(liftValue);
        }
        robot.stop();
    }
    public void openServo(int liftValue){
        while(liftValue < robot.liftA.getCurrentPosition()){
            robot.UpdateLifts(liftValue);
        }
        robot.L1.setPosition(robot.OUTTAKEB_OPEN);
    }

}