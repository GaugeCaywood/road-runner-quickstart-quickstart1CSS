package org.firstinspires.ftc.teamcode;

import android.util.Size;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.controller.PIDController;
import org.firstinspires.ftc.vision.VisionPortal;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Scalar;
import java.lang.Math;
import java.util.ArrayList;


@Autonomous(name="Human Player Side Red RR", group="Auton")
public class HPRedRR extends LinearOpMode {

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
    public String propPoz = "UNFOUND";

    int liftPos = 0;
    int target = 0;
    final double ticks_in_degrees = 751.8/180;
    PIDController controller;

    public static double p = 0.01, i = 0, d = 0.0002;
    public static double f = 0.45;
    public enum Stage{
        placePurplePixelright,
        placePurplePixelleft,
        placePurplePixelMiddle,
        idle
    }
    Stage stage = Stage.placePurplePixelright;
    @Override
    public void runOpMode() {


        controller = new PIDController(p, i, d);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Trajectory purplePixelRIGHT = drive.trajectoryBuilder(new Pose2d(-37.5, -61.2, Math.toRadians(-90)))
                .lineToLinearHeading(new Pose2d(-37.5, -23, Math.toRadians(180)))
                .waitSeconds(0.5)

                .splineToSplineHeading(new Pose2d(-57, -11, Math.toRadians(180)), Math.toRadians(180))
                .waitSeconds(1)

                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(48, -35, Math.toRadians(180)), Math.toRadians(-60))
                .waitSeconds(1)
                .build();
       /////////////////
        robot.init(hardwareMap);
        visionProcessor();
        while(!opModeIsActive()){

            telemetry.addData("Currently Recorded Position", colourMassDetectionProcessor.getRecordedPropPosition());
            telemetry.addData("Camera State", visionPortal.getCameraState());
            telemetry.addData("Currently Detected Mass Center", "x: " + colourMassDetectionProcessor.getLargestContourX() + ", y: " + colourMassDetectionProcessor.getLargestContourY());
            telemetry.addData("Currently Detected Mass Area", colourMassDetectionProcessor.getLargestContourArea());
            telemetry.update();
        }
        waitForStart();
        ColourMassDetectionProcessorRed.PropPositions recordedPropPosition = colourMassDetectionProcessor.getRecordedPropPosition();

        // now we can use recordedPropPosition to determine where the prop is! if we never saw a prop, your recorded position will be UNFOUND.
        // if it is UNFOUND, you can manually set it to any of the other positions to guess
        if (recordedPropPosition == ColourMassDetectionProcessorRed.PropPositions.UNFOUND) {
            recordedPropPosition = ColourMassDetectionProcessorRed.PropPositions.MIDDLE;
        }
        visionPortal.stopLiveView();
        visionPortal.stopStreaming();

        while(opModeIsActive() && !isStopRequested()) {
            // gets the recorded prop position
            drive.update();
            controller.setPID(p, i, d);
            int armPos = robot.liftA.getCurrentPosition();
            double pid = controller.calculate(armPos, target);
            double ff = Math.cos(Math.toRadians(target / ticks_in_degrees)) * f;

            double liftPower = pid + ff;

            robot.liftA.setPower(liftPower);
            robot.liftB.setPower(liftPower);
            if (liftPos == 0) {
                target = 0;

            } else if (liftPos == 1) {
                target = 2000;


            } else if (liftPos == 2) {
                target = 3000;

            }
            else if (liftPos == 2) {
                target = 4000;

            }

            // now we can use recordedPropPosition in our auto code to modify where we place the purple and yellow pixels
            switch (recordedPropPosition) {
                case LEFT:
                    // code to do if we saw the prop on the left
                    switch (stage){
                        case placePurplePixelleft:
                            target = 100;
                            drive.followTrajectoryAsync(purplePixelRIGHT);
                            stage = Stage.idle;
                            break;
                        case idle:
                            drive.update();

                    }

                    break;
                case UNFOUND: // we can also just add the unfound case here to do fallthrough intstead of the overriding method above, whatever you prefer!

                    break;
                case MIDDLE:
                    switch (stage){
                        case placePurplePixelMiddle:
                            drive.followTrajectoryAsync(purplePixelRIGHT);
                            stage = Stage.idle;
                            break;
                        case idle:
                            drive.update();

                    }

                    // code to do if we saw the prop on the middle
                    break;
                case RIGHT:
                    // code to do if we saw the prop on the right
                    switch (stage){
                        case placePurplePixelright:
                            drive.followTrajectoryAsync(purplePixelRIGHT);
                            stage = Stage.idle;
                            break;
                        case idle:
                            drive.update();

                    }
                    break;
            }
        }
    }

    ///// TRAJECTORIES /////
    //FORWARD


    //////START STATE MACHINE//////




    public void visionProcessor(){
        //////////VISION///////////////////////
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
        ////////VISION END/
    }




}