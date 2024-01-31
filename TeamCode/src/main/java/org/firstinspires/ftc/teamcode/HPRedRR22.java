package org.firstinspires.ftc.teamcode;


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

import org.firstinspires.ftc.vision.VisionProcessor;
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

    private int preloadpos = 0;

    enum Stage {firststage, preLoadTravel,scorepreload, drivetostack, placePixel,
        liftUp, park, liftDown, backUp, backCollect, collect, end}
    Stage stage = Stage.firststage;
    int pixels = -1;

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

        drive.setPoseEstimate(new Pose2d(-35.8, -61.2, Math.toRadians(-90)));

        TrajectorySequence PPreloadRight = drive.trajectorySequenceBuilder(new Pose2d(-36.8, -61.2, Math.toRadians(-90)))
                .setTangent(Math.toRadians(120))
                .splineToLinearHeading(new Pose2d(-29, -34, Math.toRadians(180)), Math.toRadians(50))
                .build();

        TrajectorySequence DriveToStackRight = drive.trajectorySequenceBuilder(PPreloadRight.end())
                .waitSeconds(10)
                .setTangent(Math.toRadians(200))
                .splineToLinearHeading(new Pose2d(-50, -11, Math.toRadians(180)), Math.toRadians(180))
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(34, -11, Math.toRadians(180)), Math.toRadians(-10))
                .build();


        TrajectorySequence PPreloadMiddle = drive.trajectorySequenceBuilder(new Pose2d(-36.8, -61.2, Math.toRadians(-90)))
                .lineToLinearHeading(new Pose2d(-40, -23, Math.toRadians(180)))
                .build();
        TrajectorySequence DriveToStackMiddle = drive.trajectorySequenceBuilder(PPreloadMiddle.end())
                .waitSeconds(10)
                .setTangent(Math.toRadians(160))
                .splineToLinearHeading(new Pose2d(-57, -11, Math.toRadians(180)), Math.toRadians(180))
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(34, -11, Math.toRadians(180)), Math.toRadians(-10))

                .build();
        TrajectorySequence PPreloadLeft = drive.trajectorySequenceBuilder(new Pose2d(-36.8, -61.2, Math.toRadians(-90)))
                .lineToLinearHeading(new Pose2d(-48, -22, Math.toRadians(90)) )
                .build();
        TrajectorySequence DriveToStackLeft = drive.trajectorySequenceBuilder(PPreloadLeft.end())
                .waitSeconds(10)
                .setTangent(Math.toRadians(120))
                .splineToLinearHeading(new Pose2d(-55, -11, Math.toRadians(180)), Math.toRadians(180))
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(34, -11, Math.toRadians(180)), Math.toRadians(-10))

                .build();
        TrajectorySequence DriveToBackBoardMiddle = drive.trajectorySequenceBuilder(DriveToStackMiddle.end())
                .splineToLinearHeading(new Pose2d(54.5, -39, Math.toRadians(180)), Math.toRadians(-30))
                .build();
        TrajectorySequence DriveToBackBoardLeft = drive.trajectorySequenceBuilder(DriveToStackLeft.end())
                .splineToLinearHeading(new Pose2d(54.5, -20, Math.toRadians(180)), Math.toRadians(-30))


                .build();
        TrajectorySequence DriveToBackBoardRight = drive.trajectorySequenceBuilder(DriveToStackRight.end())
                .splineToLinearHeading(new Pose2d(54.5, -46, Math.toRadians(180)), Math.toRadians(-30))
                .build();

        TrajectorySequence DriveBackFromBBR = drive.trajectorySequenceBuilder(DriveToBackBoardRight.end())
                .lineToLinearHeading(new Pose2d(-37.5, -23, Math.toRadians(180)))
                .build();
        TrajectorySequence DriveBackFromBBL = drive.trajectorySequenceBuilder(DriveToBackBoardLeft.end())
                .lineToLinearHeading(new Pose2d(-37.5, -23, Math.toRadians(180)))
                .build();
        TrajectorySequence DriveBackFromBBM = drive.trajectorySequenceBuilder(DriveToBackBoardMiddle.end())
                .lineToLinearHeading(new Pose2d(-37.5, -23, Math.toRadians(180)))
                .build();
        ///park
        TrajectorySequence DriveToBackParkM = drive.trajectorySequenceBuilder(DriveToStackMiddle.end())
                .splineToLinearHeading(new Pose2d(51.5, -38, Math.toRadians(180)), Math.toRadians(-30))
                .build();
        TrajectorySequence DriveToBackBoardL = drive.trajectorySequenceBuilder(DriveToStackLeft.end())
                .splineToLinearHeading(new Pose2d(51.5, -21, Math.toRadians(180)), Math.toRadians(-30))


                .build();
        TrajectorySequence DriveToBackBoardR = drive.trajectorySequenceBuilder(DriveToStackRight.end())
                .splineToLinearHeading(new Pose2d(51.5, -46, Math.toRadians(180)), Math.toRadians(-30))
                .build();
        ///Collect and Score white first
        TrajectorySequence DriveToCollectFirst = drive.trajectorySequenceBuilder(DriveBackFromBBL.end())
                .splineToSplineHeading(new Pose2d(-57, -11, Math.toRadians(180)), Math.toRadians(180))
                .build();
        TrajectorySequence DriveToPlaceBeforeLift = drive.trajectorySequenceBuilder(DriveToCollectFirst.end())
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(48, -35, Math.toRadians(180)), Math.toRadians(-60))
                        .build();

        robot.L1.setPosition(robot.OUTTAKEA_CLOSE);
        robot.L2.setPosition(robot.OUTTAKEB_CLOSE);
        ElapsedTime servo = new ElapsedTime();
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
            if(robot.liftA.getCurrentPosition() < 1500){
                robot.wristDown();
            }

            switch (stage) {
                case firststage:
                    pixels = 0;
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
                        robot.L2.setPosition(robot.OUTTAKEB_OPEN);

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
                        if(pixels == 0) {
                            if (preloadpos == 1) {
                                target = 1800;
                                drive.followTrajectorySequenceAsync(DriveToBackBoardLeft);
                                target = 1800;
                                servo.reset();
                                stage = Stage.placePixel;
                            } else if (preloadpos == 2) {
                                drive.followTrajectorySequenceAsync(DriveToBackBoardMiddle);
                                target = 1800;
                                servo.reset();
                                stage = Stage.placePixel;
                            } else if (preloadpos == 3) {
                                drive.followTrajectorySequenceAsync(DriveToBackBoardRight);
                                target = 1800;
                                servo.reset();
                                stage = Stage.placePixel;

                            }
                        }
                        else if(pixels == 1){
                            if(!drive.isBusy()) {
                                target = 2500;
                            }
                            if(robot.liftA.getCurrentPosition() > 2400 && !servoUp){
                                servo.reset();
                                servoUp = true;
                            }
                            if(servo.seconds() > 2){
                                robot.servo(false, 2, true);
                            }
                            if(servo.seconds() >4){
                                stage = Stage.liftDown;
                            }
                        }
                    }

                    break;
                case placePixel:


                    if(!drive.isBusy()) {
                        if (!servoUp && robot.liftA.getCurrentPosition() > 1500) {
                            robot.wristUp();
                            servoUp = true;
                        }
                    }
                    if(servoUp&& servo.seconds() > .5){
                        robot.L1.setPosition(robot.OUTTAKEA_OPEN);
                    }
                    if(servo.seconds() > 1) {
                        if (!drive.isBusy() && servoUp) {
                            stage = Stage.liftDown;
                        }
                    }
                    break;
                case backUp:
                    robot.autonIntake.setPower(-1);
                    robot.autonHeightControlS.setPosition(.8);
                    if(!drive.isBusy()){
                        if(preloadpos == 1) {
                            drive.followTrajectorySequenceAsync(DriveBackFromBBL);
                            stage = Stage.backCollect;
                        }
                        else if(preloadpos == 2){
                            drive.followTrajectorySequenceAsync(DriveBackFromBBM);
                            stage = Stage.backCollect;
                        }
                        else if(preloadpos == 3){
                            drive.followTrajectorySequenceAsync(DriveBackFromBBR);
                            stage = Stage.backCollect;
                        }
                    }

                    break;
                case park:
                    if(!drive.isBusy()) {
                        if (preloadpos == 1) {
                            drive.followTrajectorySequenceAsync(DriveToBackBoardL);
                            servo.reset();
                            stage = Stage.liftDown;
                        } else if (preloadpos == 2) {
                            drive.followTrajectorySequenceAsync(DriveToBackParkM);

                            servo.reset();
                            stage = Stage.liftDown;
                        } else if (preloadpos == 3) {
                            drive.followTrajectorySequenceAsync(DriveToBackBoardR);
                            servo.reset();
                            stage = Stage.liftDown;

                        }
                    }
                    break;
                case liftDown:
                    if(!drive.isBusy()){
                        servoUp = false;
                        target = -30;
                        if(pixels == 0){
                            //drive.followTrajectorySequenceAsync();
                            stage = Stage.backUp;
                        }
                        if(pixels == 1) {
                            stage = Stage.park;
                        }
                    }
                    break;

                case end:
                    telemetry.addData("Lift Is: ", robot.liftA.getCurrentPosition());
                    telemetry.addData("Target: ", target);
                    telemetry.update();
                    break;
                case backCollect:
                    robot.autonIntake.setPower(-1);
                    robot.autonHeightControlS.setPosition(.8);
                    robot.intake.setPower(1);
                    if(!drive.isBusy()){
                        drive.followTrajectorySequenceAsync(DriveToCollectFirst);
                        servo.reset();
                        stage = Stage.collect;
                    }
                    break;
                case collect:
                    if(servo.seconds() > 2){
                        robot.servo(true, 2, true);
                        pixels = 1;
                        drive.followTrajectorySequenceAsync(DriveToPlaceBeforeLift);
                        stage = Stage.liftUp;
                    }

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