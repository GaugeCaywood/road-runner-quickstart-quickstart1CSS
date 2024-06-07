package org.firstinspires.ftc.teamcode;


import android.content.SharedPreferences;
import android.preference.PreferenceManager;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
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

import org.opencv.core.Scalar;
import java.lang.Math;
import java.util.ArrayList;



@Autonomous(name="Human Player Side Red RR 2+2", group="Auton")
public class HPRedRR22 extends LinearOpMode {
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
    public boolean liftUp = false;

    public boolean backUp = false;
    private int preloadpos = 0;

    enum Stage {firststage, preLoadTravel,scorepreload, drivetostack, placePixel,
        liftUp, park, liftDown, backUp, backCollect, collect,grab, liftUpPlaceTwo, end, collecting}
    Stage stage = Stage.firststage;


    int liftPos = -1;
    int target = -45;
    final double ticks_in_degrees = 751.8/180;
    PIDController controller;

    public static double p = 0.01, i = 0, d = 0.000;
    public static double f = 0.2;
    public boolean drop = false;
    ElapsedTime initTime = new ElapsedTime();
    @Override
    public void runOpMode() {



        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        /////////////////
        robot.init(hardwareMap);
        robot.high();

        //TRAJECTORIES FOR ROADRUNNER//
        //
        //

        drive.setPoseEstimate(new Pose2d(-35.8, -61.2, Math.toRadians(-90)));
        TrajectorySequence PPreloadRight = drive.trajectorySequenceBuilder(new Pose2d(-36.8, -61.2, Math.toRadians(-90)))

                .setTangent(Math.toRadians(130))
                .splineToLinearHeading(new Pose2d(-29.5, -32, Math.toRadians(180)), Math.toRadians(50))

                .build();

        TrajectorySequence DriveToStackRight = drive.trajectorySequenceBuilder(PPreloadRight.end())
                .setTangent(Math.toRadians(200))
                .splineToLinearHeading(new Pose2d(-56, -11, Math.toRadians(180)), Math.toRadians(180))
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(45, -14, Math.toRadians(180)), Math.toRadians(-10))
                .build();


        TrajectorySequence PPreloadMiddle = drive.trajectorySequenceBuilder(new Pose2d(-36.8, -61.2, Math.toRadians(-90)))
                .lineToLinearHeading(new Pose2d(-40, -23, Math.toRadians(180)))

                .build();
        TrajectorySequence DriveToStackMiddle = drive.trajectorySequenceBuilder(PPreloadMiddle.end())
                .setTangent(Math.toRadians(190))
                .splineToLinearHeading(new Pose2d(-57, -12, Math.toRadians(180)), Math.toRadians(180))
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(40, -14, Math.toRadians(180)), Math.toRadians(-10))

                .build();
        TrajectorySequence PPreloadLeft = drive.trajectorySequenceBuilder(new Pose2d(-36.8, -61.2, Math.toRadians(-90)))

                .lineToLinearHeading(new Pose2d(-48, -22, Math.toRadians(90)) )

                .build();
        TrajectorySequence DriveToStackLeft = drive.trajectorySequenceBuilder(PPreloadLeft.end())
                .setTangent(Math.toRadians(120))
                .splineToLinearHeading(new Pose2d(-60, -11, Math.toRadians(180)), Math.toRadians(180))
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(45, -14, Math.toRadians(180)), Math.toRadians(-10))

                .build();
        TrajectorySequence DriveToBackBoardMiddle = drive.trajectorySequenceBuilder(DriveToStackMiddle.end())
                .splineToLinearHeading(new Pose2d(50, -37, Math.toRadians(180)), Math.toRadians(-30))
                .build();
        TrajectorySequence DriveToBackBoardLeft = drive.trajectorySequenceBuilder(DriveToStackLeft.end())
                .splineToLinearHeading(new Pose2d(51, -21, Math.toRadians(180)), Math.toRadians(-30))


                .build();
        TrajectorySequence DriveToBackBoardRight = drive.trajectorySequenceBuilder(DriveToStackRight.end())
                .splineToLinearHeading(new Pose2d(50.5, -41, Math.toRadians(180)), Math.toRadians(-30))
                .build();
        ///park
        TrajectorySequence DriveToBackParkM = drive.trajectorySequenceBuilder(DriveToStackMiddle.end())
                .splineToLinearHeading(new Pose2d(49, -38, Math.toRadians(180)), Math.toRadians(-30))
                .build();
        TrajectorySequence DriveToBackBoardL = drive.trajectorySequenceBuilder(DriveToStackLeft.end())
                .splineToLinearHeading(new Pose2d(49, -21, Math.toRadians(180)), Math.toRadians(-30))


                .build();
        TrajectorySequence DriveToBackBoardR = drive.trajectorySequenceBuilder(DriveToStackRight.end())
                .splineToLinearHeading(new Pose2d(51.5, -46, Math.toRadians(180)), Math.toRadians(-30))
                .build();
        ///Collect and Score white first
        TrajectorySequence DriveToCollectFirstR = drive.trajectorySequenceBuilder(DriveToBackBoardR.end())
                .splineToLinearHeading(new Pose2d(43,-4,Math.toRadians(180)), Math.toRadians(0))
                .setTangent(Math.toRadians(180))

                .splineToLinearHeading(new Pose2d(-57, -9.25                                                                                                                                                                                                                                               , Math.toRadians(180)), Math.toRadians(180))
                .build();
        TrajectorySequence DriveToCollectFirstM = drive.trajectorySequenceBuilder(DriveToBackBoardMiddle.end())
                .setTangent(Math.toRadians(120))
                .splineToLinearHeading(new Pose2d(-57, -7, Math.toRadians(180)), Math.toRadians(180))
                .build();
        TrajectorySequence DriveToCollectFirstL = drive.trajectorySequenceBuilder(DriveToBackBoardL.end())
                .setTangent(Math.toRadians(120))
                .splineToLinearHeading(new Pose2d(-59, -8.5, Math.toRadians(180)), Math.toRadians(180))
                .build();
        TrajectorySequence DriveToPlaceBeforeLift = drive.trajectorySequenceBuilder(DriveToCollectFirstL.end())
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(53, -30, Math.toRadians(180)), Math.toRadians(-70))
                        .build();
        TrajectorySequence DriveToPlaceBeforeLiftM = drive.trajectorySequenceBuilder(DriveToCollectFirstM.end())
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(53, -30, Math.toRadians(180)), Math.toRadians(-70))
                .build();
        TrajectorySequence DriveToPlaceBeforeLiftR = drive.trajectorySequenceBuilder(DriveToCollectFirstR.end())
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(53, -30, Math.toRadians(180)), Math.toRadians(-70))
                .build();

        robot.L1.setPosition(robot.OUTTAKEA_CLOSE);
        robot.L2.setPosition(robot.OUTTAKEB_CLOSE);
        ElapsedTime servo = new ElapsedTime();
        ElapsedTime collect = new ElapsedTime();
        ElapsedTime lift = new ElapsedTime();

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
            if(robot.liftA.getCurrentPosition() > robot.LIFTENCODERTRIGGER){
                robot.wristUp();

            }
            if(robot.liftA.getCurrentPosition() < robot.LIFTENCODERTRIGGER) {
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
                            stage =Stage.preLoadTravel;
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
                        drive.followTrajectorySequenceAsync(PPreloadRight);
                    }
                    if (preloadpos == 2) {
                        drive.followTrajectorySequenceAsync(PPreloadMiddle);
                    }
                    if (preloadpos == 1) {
                        drive.followTrajectorySequenceAsync(PPreloadLeft);
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

                        stage = Stage.drivetostack;
                    }


                    break;
                case drivetostack:
                    if(!drive.isBusy()) {
                        if (preloadpos == 1) {
                            drive.followTrajectorySequenceAsync(DriveToStackLeft);
                        } else if (preloadpos == 2) {
                            drive.followTrajectorySequenceAsync(DriveToStackMiddle);
                        } else if (preloadpos == 3) {
                            drive.followTrajectorySequenceAsync(DriveToStackRight);
                        }
                    }

                    stage = Stage.liftUp;
                    break;
                case liftUp:

                    if(!drive.isBusy()) {
                        if (preloadpos == 1) {
                            target = 2300;
                            drive.followTrajectorySequenceAsync(DriveToBackBoardLeft);
                            target = 2300;
                            servo.reset();
                            stage = Stage.placePixel;
                        } else if (preloadpos == 2) {
                            drive.followTrajectorySequenceAsync(DriveToBackBoardMiddle);
                            target = 2300;
                            servo.reset();
                            stage = Stage.placePixel;
                        } else if (preloadpos == 3) {
                            drive.followTrajectorySequenceAsync(DriveToBackBoardRight);
                            target = 2300;                            servo.reset();
                            stage = Stage.placePixel;

                        }
                    }

                    break;
                case placePixel:


                    if(!drive.isBusy() && robot.liftA.getCurrentPosition() > 2200) {
                        servoUp = true;


                        if (servoUp) {
                            robot.L2.setPosition(robot.OUTTAKEB_OPEN);
                        }
                        if (servo.seconds() > 1) {
                            if (!drive.isBusy() && servoUp) {
                                stage = Stage.collect;
                            }
                        }
                    }
                    break;
                case collect:

                    if(!drive.isBusy() && !backUp) {
                        if(preloadpos == 1) {
                            drive.followTrajectorySequenceAsync(DriveToCollectFirstL);
                            robot.intake.setPower(1);
                            robot.firstPixel();
                        } else if (preloadpos == 2) {
                            drive.followTrajectorySequenceAsync(DriveToCollectFirstM);
                            robot.intake.setPower(1);
                            robot.firstPixel();
                        }
                        else{
                            drive.followTrajectorySequenceAsync(DriveToCollectFirstR);
                            robot.intake.setPower(1);
                            robot.firstPixel();
                        }
                        backUp = true;
                        collect.reset();
                    }
                    if(backUp && collect.milliseconds() > 500){
                        target = -100;
                        stage = Stage.backCollect;
                    }
                    break;
                case backCollect:
                    if(!drive.isBusy()){
                        collect.reset();


                        stage = Stage.collecting;
                    }
                    break;
                case collecting:
                    if(collect.milliseconds()> 1000){
                        robot.secondPixel();
                    }
                    if(collect.milliseconds() > 2500) {
                        stage = Stage.grab;
                    }
                    break;
                case grab:

                         robot.intake.setPower(0);
                        robot.servo(true, 2, true);
                    if (preloadpos== 1) {


                        drive.followTrajectorySequenceAsync(DriveToPlaceBeforeLift);
                    }
                    else if(preloadpos ==2){
                        drive.followTrajectorySequenceAsync(DriveToPlaceBeforeLiftM);
                    }
                    else {
                        drive.followTrajectorySequenceAsync(DriveToPlaceBeforeLiftR);
                    }
                        lift.reset();
                        stage = Stage.liftUpPlaceTwo;

                    break;
                case liftUpPlaceTwo:
                    robot.intake.setPower(-1);
                    if(!drive.isBusy() && !liftUp){

                        robot.intake.setPower(0);
                        target = 3000;
                        liftUp = true;
                    }
                    if(robot.liftA.getCurrentPosition() > 2900&& !drop){
                        robot.servo(false, 2, false);
                        robot.L1.setPosition(robot.OUTTAKEA_OPEN);
                        robot.L2.setPosition(robot.OUTTAKEB_OPEN);
                        lift.reset();
                        drop = true;
                    }
                    if(lift.milliseconds() >500 && drop){
                        stage = Stage.park;
                    }
                    break;
                case park:
                    if(!drive.isBusy()) {
                        robot.intake.setPower(0);
                        if (preloadpos == 1) {
                            drive.followTrajectorySequenceAsync(DriveToBackBoardL);
                            stage = Stage.liftDown;
                                         }
                        if(preloadpos == 2){
                            drive.followTrajectorySequenceAsync(DriveToBackParkM);
                            stage = Stage.liftDown;
                        }
                        else if (preloadpos == 3) {
                            drive.followTrajectorySequenceAsync(DriveToBackBoardR);
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