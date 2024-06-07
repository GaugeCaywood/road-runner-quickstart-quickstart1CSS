package org.firstinspires.ftc.teamcode;


import android.content.SharedPreferences;
import android.preference.PreferenceManager;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.vision.VisionPortal;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Scalar;
import java.lang.Math;
import java.util.ArrayList;


@Autonomous(name="Background side Red 2+0 ", group="Auton")
public class BBCred extends LinearOpMode {
    //////////////////VISION//////////////////////
    private VisionPortal visionPortal;
    private ColourMassDetectionProcessorRed colourMassDetectionProcessor;
    Scalar lower = new Scalar(140, 60, 0); // the lower hsv threshold for your detection
    Scalar upper = new Scalar(180, 255, 255);  // the upper hsv threshold for your detection
    boolean servoUp = false;
    double minArea = 150;
    /////////////////////HARDWARE
    BotHardwareNew robot = new BotHardwareNew();
    public ElapsedTime runtime = new ElapsedTime();

    public boolean position = true;

    private int preloadpos = 0;

    enum Stage {firststage, preLoadTravel,scorepreload, drivetostack, placePixel, liftUp, park, liftDown, end}
    Stage stage = Stage.firststage;


    int liftPos = -1;
    int target = -45;
    final double ticks_in_degrees = 751.8/180;
    PIDController controller;

    public static double p = 0.01, i = 0, d = 0.000;
    public static double f = 0.2;
    @Override
    public void runOpMode() {



        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        /////////////////
        robot.init(hardwareMap);


        //TRAJECTORIES FOR ROADRUNNER//
        //
        //
        ElapsedTime servo = new ElapsedTime();
        drive.setPoseEstimate(new Pose2d(12, -61.2, Math.toRadians(-90)));
        TrajectorySequence DriveToPreloadR =  drive.trajectorySequenceBuilder(new Pose2d(12, -61.2, Math.toRadians(-90)))
                .setTangent(Math.toRadians(40))
                .splineToLinearHeading(new Pose2d(29.5, -29, Math.toRadians(0)), Math.toRadians(70))
                .addDisplacementMarker(()-> {robot.L1.setPosition(robot.OUTTAKEA_OPEN);})
                .setTangent(Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(53.5, -39, Math.toRadians(180)), Math.toRadians(0))

                        .build();
        TrajectorySequence DriveToPreloadM = drive.trajectorySequenceBuilder(new Pose2d(12, -61.2, Math.toRadians(-90)))
                .setTangent(Math.toRadians(40))
                .splineToLinearHeading(new Pose2d(17, -23, Math.toRadians(0)), Math.toRadians(70))
                .addDisplacementMarker(()-> {robot.L1.setPosition(robot.OUTTAKEA_OPEN);})
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(23,-23,Math.toRadians(0)),Math.toRadians(0))
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(53.5, -30, Math.toRadians(180)), Math.toRadians(0))
                .build();
        TrajectorySequence DriveToPreloadL = drive.trajectorySequenceBuilder(new Pose2d(12, -61.2, Math.toRadians(-90)))
                .setTangent(Math.toRadians(0)).splineToLinearHeading(new Pose2d(6, -32, Math.toRadians(0)), Math.toRadians(180))
                .addDisplacementMarker(()-> {robot.L1.setPosition(robot.OUTTAKEA_OPEN);})
                .waitSeconds(.2)
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(52.5, -25, Math.toRadians(180)), Math.toRadians(0))

                .build();

        TrajectorySequence DriveToParkR = drive.trajectorySequenceBuilder(DriveToPreloadR.end())
                .splineToLinearHeading(new Pose2d(52, -59, Math.toRadians(180)), Math.toRadians(0))
                        .build();
        TrajectorySequence DriveToParkM = drive.trajectorySequenceBuilder(DriveToPreloadM.end())
                .splineToLinearHeading(new Pose2d(52, -59, Math.toRadians(180)), Math.toRadians(0))
                        .build();
        TrajectorySequence DriveToParkL = drive.trajectorySequenceBuilder(DriveToPreloadL.end())
                .splineToLinearHeading(new Pose2d(52, -59, Math.toRadians(180)), Math.toRadians(0))
                        .build();
        robot.L1.setPosition(robot.OUTTAKEA_CLOSE);
        robot.L2.setPosition(robot.OUTTAKEB_CLOSE);
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
        robot.L1.setPosition(robot.OUTTAKEA_CLOSE);
        while(opModeIsActive() && !isStopRequested()) {
            // gets the recorded prop position
            drive.update();

            controller = new PIDController(p, i, d);
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
            if(robot.liftA.getCurrentPosition() < 1800){
                robot.wristDown();
            }

            switch (stage) {
                case firststage:

//                    put your vision processor in here
                    switch (recordedPropPosition) {
                        case LEFT:
                            // code to do if we saw the prop on the left
                            preloadpos = 1;
                            stage = Stage.preLoadTravel;

                            break;
                        case UNFOUND: // we can also just add the unfound case here to do fallthrough intstead of the overriding method above, whatever you prefer!

                            break;
                        case MIDDLE:
                            preloadpos = 2;
                            stage = Stage.preLoadTravel;
                            // code to do if we saw the prop on the middle
                            break;
                        case RIGHT:
                            // code to do if we saw the prop on the right
                            preloadpos = 3;

                            stage = Stage.preLoadTravel;

                            break;
                    }

                    break;
                case preLoadTravel:

                    if (preloadpos == 3) {
                        drive.followTrajectorySequenceAsync(DriveToPreloadR);
                        target = 300;
                    }
                    if (preloadpos == 2) {
                        drive.followTrajectorySequenceAsync(DriveToPreloadM);
                        target = 600;
                    }
                    if (preloadpos == 1) {
                        drive.followTrajectorySequenceAsync(DriveToPreloadL);
                        target = 300;
                    }

                    stage = Stage.scorepreload;
                    break;
                case scorepreload:
                    if (!drive.isBusy()) {
                        telemetry.addData("Lift Is: ", robot.liftA.getCurrentPosition());
                        telemetry.addData("Target: ", target);
                        telemetry.update();


                        stage = Stage.liftUp;
                    }
                    break;

                case liftUp:
                    if(!drive.isBusy()){
                        servo.reset();
                        stage = Stage.placePixel;
                    }


                    break;
                case placePixel:


                    if(!drive.isBusy()) {
                        target = 2000;
                        if (!servoUp && robot.liftA.getCurrentPosition() > 1800) {
                            robot.wristUp();
                            servoUp = true;
                        }
                    }
                    if(servoUp&& servo.seconds() > 1){
                        robot.L2.setPosition(robot.OUTTAKEB_OPEN);
                    }
                    if(servo.seconds() > 2) {
                        if (!drive.isBusy() && servoUp) {
                            stage = Stage.park;
                        }
                    }
                    break;
                case park:
                    if(!drive.isBusy()) {
                        if (preloadpos == 1) {
                            drive.followTrajectorySequenceAsync(DriveToParkL);
                            servo.reset();
                            stage = Stage.liftDown;
                        } else if (preloadpos == 2) {
                            drive.followTrajectorySequenceAsync(DriveToParkM);

                            servo.reset();
                            stage = Stage.liftDown;
                        } else if (preloadpos == 3) {
                            drive.followTrajectorySequenceAsync(DriveToParkR);
                            servo.reset();
                            stage = Stage.liftDown;

                        }
                    }
                    break;
                case liftDown:
                    if(!drive.isBusy()){
                        target = -100;
                        stage = Stage.end;
                    }
                    break;
                case end:
                    telemetry.addData("Lift Is: ", robot.liftA.getCurrentPosition());
                    telemetry.addData("Target: ", target);
                    telemetry.update();
                    Pose2d endPose = drive.getPoseEstimate();
                    SharedPreferences prefs = PreferenceManager.getDefaultSharedPreferences(hardwareMap.appContext);
                    SharedPreferences.Editor editor = prefs.edit();
                    editor.putFloat("endPositionX",(float) endPose.getX());
                    editor.putFloat("endPositionY",(float) endPose.getY());
                    editor.putFloat("endPosition_Heading",(float) endPose.getHeading());
                    editor.putFloat("color",  1);
                    editor.apply();
                    break;



            }

            // now we can use recordedPropPosition in our auto code to modify where we place the purple and yellow pixels


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