package org.firstinspires.ftc.teamcode;


import android.content.SharedPreferences;
import android.preference.PreferenceManager;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import com.acmerobotics.dashboard.config.Config;
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

@Config
@Autonomous(name="Background side Blue 2+4 ", group="Auton")
public class BBCBlue24 extends LinearOpMode {
    //////////////////VISION//////////////////////
    private VisionPortal visionPortal;
    private ColourMassDetectionProcessorRed colourMassDetectionProcessor;
    Scalar lower = new Scalar(110, 70, 20); // the lower hsv threshold for your detection
    Scalar upper = new Scalar(140, 255, 255);  // the upper hsv threshold for your detection
    boolean servoUp = false;
    double minArea = 150;
    /////////////////////HARDWARE
    BotHardwareNew robot = new BotHardwareNew();
    public ElapsedTime runtime = new ElapsedTime();
    public boolean backUp = false;
    public boolean position = true;

    private int preloadpos = 0;

    enum Stage {firststage, preLoadTravel,scorepreload, drivetostack, placePixel, liftUp, park, liftDown, liftDownCollect,DriveToCollect,collect,collecting,drivetoplace, reachedtarge,place,liftDownCollectSecond,collectSecond,collectingSecond,drivetoplaceSecond,placeSecond, end}
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
        ElapsedTime collect = new ElapsedTime();
        drive.setPoseEstimate(new Pose2d(14, 61.2, Math.toRadians(90)));
        TrajectorySequence DriveToPreloadR =  drive.trajectorySequenceBuilder(new Pose2d(14, 61.2, Math.toRadians(90)))
                .setTangent(Math.toRadians(-40))
                .splineToLinearHeading(new Pose2d(8, 32, Math.toRadians(0)), Math.toRadians(-100))
                .addDisplacementMarker(()->{robot.L1.setPosition(robot.OUTTAKEA_CLOSE);
                })
                .build();
        TrajectorySequence DriveToPreloadM = drive.trajectorySequenceBuilder(new Pose2d(14, 61.2, Math.toRadians(90)))

                .setTangent(Math.toRadians(-40))
                .splineToLinearHeading(new Pose2d(20, 25, Math.toRadians(0)), Math.toRadians(-100))
                .build();
        TrajectorySequence DriveToPreloadL = drive.trajectorySequenceBuilder(new Pose2d(14, 61.2, Math.toRadians(90)))

                .setTangent(Math.toRadians(-40))
                .splineToLinearHeading(new Pose2d(30, 28, Math.toRadians(0)), Math.toRadians(-100))
                .build();
        TrajectorySequence DriveToBackBoardR = drive.trajectorySequenceBuilder(DriveToPreloadR.end())

                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(53, 31, Math.toRadians(180)), Math.toRadians(0))
                .build();
        TrajectorySequence DriveToBackBoardM = drive.trajectorySequenceBuilder(DriveToPreloadM.end())
                .splineToLinearHeading(new Pose2d(25,25,Math.toRadians(0)),Math.toRadians(180))
                .setTangent(Math.toRadians(20))
                .splineToLinearHeading(new Pose2d(53, 37, Math.toRadians(180)), Math.toRadians(90))
                .build();
        TrajectorySequence DriveToBackBoardL = drive.trajectorySequenceBuilder(DriveToPreloadL.end())
                .splineToLinearHeading(new Pose2d(36.5,28,Math.toRadians(0)),Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(54.5, 40, Math.toRadians(180)), Math.toRadians(0))
                .build();
        //collect
        TrajectorySequence DriveToCollectFirstR = drive.trajectorySequenceBuilder(DriveToBackBoardR.end())


                .splineToLinearHeading(new Pose2d(20, 62, Math.toRadians(180)), Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-40,62,Math.toRadians(180)), Math.toRadians(180))

                .splineToLinearHeading(new Pose2d(-57,32, Math.toRadians(180)),Math.toRadians(-90))
                .build();
        TrajectorySequence DriveToCollectFirstM = drive.trajectorySequenceBuilder(DriveToBackBoardM.end())


                .splineToLinearHeading(new Pose2d(20, 62, Math.toRadians(180)), Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-40,62,Math.toRadians(180)), Math.toRadians(180))

                .splineToLinearHeading(new Pose2d(-57,32, Math.toRadians(180)),Math.toRadians(-90))
                .build();
        TrajectorySequence DriveToCollectFirstL = drive.trajectorySequenceBuilder(DriveToBackBoardL.end())

                .splineToLinearHeading(new Pose2d(20, 62, Math.toRadians(180)), Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-40,62,Math.toRadians(180)), Math.toRadians(180))

                .splineToLinearHeading(new Pose2d(-57,32, Math.toRadians(180)),Math.toRadians(-90))
                .build();
        TrajectorySequence DriveToPlaceFirst = drive.trajectorySequenceBuilder(DriveToCollectFirstL.end())
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(-35,66,Math.toRadians(180)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(20,58,Math.toRadians(180)), Math.toRadians(0))
                .addDisplacementMarker(()->{target= 2500;})
                .splineToLinearHeading(new Pose2d(53.5, 41, Math.toRadians(180)), Math.toRadians(0))
                .addDisplacementMarker(()->{robot.intake.setPower(0);})
                .build();
        TrajectorySequence DriveToCollectSecond = drive.trajectorySequenceBuilder(DriveToPlaceFirst.end())
                .addDisplacementMarker(()->{target= -100;})
                .splineToLinearHeading(new Pose2d(20, 63, Math.toRadians(180)), Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-50,63,Math.toRadians(180)), Math.toRadians(180))

                .splineToLinearHeading(new Pose2d(-57,34, Math.toRadians(180)),Math.toRadians(-90))
                .addDisplacementMarker(()->{collect.reset();})
                .build();
        TrajectorySequence DrivetToPlaceSecond = drive.trajectorySequenceBuilder(DriveToPlaceFirst.end())
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(-35,66,Math.toRadians(180)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(20,64,Math.toRadians(180)), Math.toRadians(0))
                .addDisplacementMarker(()->{target= 2500;})
                .splineToLinearHeading(new Pose2d(53, 41, Math.toRadians(180)), Math.toRadians(0))
                .addDisplacementMarker(()->{robot.intake.setPower(0);})
                .build();
        //park



        robot.L1.setPosition(robot.OUTTAKEA_CLOSE);
        robot.L2.setPosition(robot.OUTTAKEB_CLOSE);
        ElapsedTime servo = new ElapsedTime();
        ElapsedTime collecting = new ElapsedTime();
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
            recordedPropPosition = ColourMassDetectionProcessorRed.PropPositions.LEFT;
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
            if(robot.liftA.getCurrentPosition() < robot.LIFTENCODERTRIGGER){
                robot.wristDown();
            }
            if(robot.liftA.getCurrentPosition() > robot.LIFTENCODERTRIGGER){
                robot.wristUp();
            }
            telemetry.addData("Collecting seconds: ", collecting.seconds());
            telemetry.update();
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
                    }
                    if (preloadpos == 2) {
                        drive.followTrajectorySequenceAsync(DriveToPreloadM);
                    }
                    if (preloadpos == 1) {
                        drive.followTrajectorySequenceAsync(DriveToPreloadL);
                    }
                    target = 300;
                    stage = Stage.scorepreload;
                    break;
                case scorepreload:
                    if (!drive.isBusy()) {
                        telemetry.addData("Lift Is: ", robot.liftA.getCurrentPosition());
                        telemetry.addData("Target: ", target);
                        telemetry.update();
                        robot.L1.setPosition(robot.OUTTAKEA_OPEN);

                        stage = Stage.liftUp;
                    }
                    break;

                case liftUp:

                    if(!drive.isBusy()) {
                        if (preloadpos == 1) {

                            drive.followTrajectorySequenceAsync(DriveToBackBoardL);

                            servo.reset();
                            stage = Stage.placePixel;
                        } else if (preloadpos == 2) {
                            drive.followTrajectorySequenceAsync(DriveToBackBoardM);

                            servo.reset();
                            stage = Stage.placePixel;
                        } else if (preloadpos == 3) {
                            drive.followTrajectorySequenceAsync(DriveToBackBoardR);
                            servo.reset();
                            stage = Stage.placePixel;

                        }
                    }

                    break;
                case placePixel:


                    if(!drive.isBusy()) {
                        target = 2000;
                        if (robot.liftA.getCurrentPosition() > 1800) {

                            servoUp = true;
                        }

                        if (servoUp && servo.seconds() > 5) {
                            robot.L2.setPosition(robot.OUTTAKEB_OPEN);
                        }
                        if (servo.seconds() > 5.5) {
                            if (!drive.isBusy() && servoUp) {
                                collect.reset();
                                stage = Stage.liftDownCollect;

                            }
                        }
                    }
                    break;
                case liftDownCollect:
                    if(!drive.isBusy() && !backUp){
                        if(preloadpos ==1){
                            drive.followTrajectorySequenceAsync(DriveToCollectFirstL);
                        } else if (preloadpos == 2) {
                            drive.followTrajectorySequenceAsync(DriveToCollectFirstM);
                        }
                        else{
                            drive.followTrajectorySequenceAsync(DriveToCollectFirstR);
                        }
                        backUp = true;
                    }
                    if(collect.milliseconds() > 500 && backUp){
                        target = -100;
                        stage = Stage.collect;
                    }
                    break;
                case collect:
                    if(!drive.isBusy()){
                        robot.intake.setPower(1);
                        robot.firstPixel();
                        collecting.reset();
                        stage = Stage.collecting;
                    }
                    break;
                case collecting:
                    if(collecting.milliseconds()>1250){
                        robot.secondPixel();
                    }
                    if (collecting.milliseconds()> 2500){
                        robot.servo(true,2,true);
                        robot.intake.setPower(-1);
                        robot.high();
                        stage = Stage.drivetoplace;
                    }
                    break;
                case drivetoplace:
                    if(!drive.isBusy()){
                        drive.followTrajectorySequenceAsync(DriveToPlaceFirst);

                        stage = Stage.reachedtarge;
                    }
                    break;
                case reachedtarge:
                    if(!drive.isBusy()){
                        servo.reset();
                        target = 2500;
                        stage = Stage.place;
                    }
                    break;
                case place:
                    if (robot.liftA.getCurrentPosition()> 2300){
                        robot.servo(false,2,true);
                        backUp=false;
                        collect.reset();
                        stage = Stage.liftDownCollectSecond;

                    }
                    break;
                case liftDownCollectSecond:
                    if(!drive.isBusy()){
                        if(!drive.isBusy() && !backUp){
                            drive.followTrajectorySequenceAsync(DriveToCollectSecond);
                            backUp = true;
                        }
                        if(collect.milliseconds() > 500 && backUp){
                            target = -100;
                            robot.low();
                            robot.intake.setPower(1);
                            stage = Stage.collectSecond;
                        }
                    }
                    break;
                case collectSecond:
                    if(!drive.isBusy()){


                        collecting.reset();
                        stage = Stage.collecting;
                    }
                    break;
                case collectingSecond:
                    if (collecting.milliseconds()> 2500){
                        robot.servo(true,2,true);
                        robot.intake.setPower(-1);
                        robot.high();
                        stage = Stage.drivetoplace;
                    }

                    break;
                case drivetoplaceSecond:
                    if(!drive.isBusy()){
                        drive.followTrajectorySequenceAsync(DrivetToPlaceSecond);
                        stage =Stage.placeSecond;
                    }
                    break;
                case placeSecond:
                    if (robot.liftA.getCurrentPosition()> 2300&& !drive.isBusy()){
                        robot.servo(false,2,true);
                        backUp=false;
                        collect.reset();
                        stage = Stage.end;

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
                    editor.putFloat("color",  0);
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