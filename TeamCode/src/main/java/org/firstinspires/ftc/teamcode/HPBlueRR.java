package org.firstinspires.ftc.teamcode;


import android.content.SharedPreferences;
import android.preference.PreferenceManager;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.core.Scalar;

@Autonomous(name="Human Player Side Blue RR", group="Auton")
public class HPBlueRR extends LinearOpMode {
    //////////////////VISION//////////////////////
    private VisionPortal visionPortal;
    private ColourMassDetectionProcessor colourMassDetectionProcessor;
    Scalar lower = new Scalar(110, 70, 20); // the lower hsv threshold for your detection
    Scalar upper = new Scalar(140, 255, 255);  // the upper hsv threshold for your detection
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

        drive.setPoseEstimate(new Pose2d(-34, 61.2, Math.toRadians(90)));

        TrajectorySequence PPreloadRight = drive.trajectorySequenceBuilder(new Pose2d(-34, 61.2, Math.toRadians(90)))
                .setTangent(Math.toRadians(-120))
                .splineToLinearHeading(new Pose2d(-44.5, 21, Math.toRadians(-90)), Math.toRadians(-90))
                .build();

        TrajectorySequence DriveToStackRight = drive.trajectorySequenceBuilder(PPreloadRight.end())

                .setTangent(Math.toRadians(-100))
                .splineToLinearHeading(new Pose2d(-57, 7, Math.toRadians(180)), Math.toRadians(-180))




                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(32, 13, Math.toRadians(180)), Math.toRadians(0))

                .build();


        TrajectorySequence PPreloadMiddle = drive.trajectorySequenceBuilder(new Pose2d(-34, 61.2, Math.toRadians(90)))
                .setTangent(Math.toRadians(-120))
                .splineToLinearHeading(new Pose2d(-46, 24, Math.toRadians(-180)), Math.toRadians(-180))
                .build();
        TrajectorySequence DriveToStackMiddle = drive.trajectorySequenceBuilder(PPreloadMiddle.end())

                .setTangent(Math.toRadians(-160))
                .splineToLinearHeading(new Pose2d(-59, 7, Math.toRadians(-180)), Math.toRadians(-180))
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(25, 12, Math.toRadians(-180)), Math.toRadians(0))

                .build();
        TrajectorySequence PPreloadLeft = drive.trajectorySequenceBuilder(new Pose2d(-34, 61.2, Math.toRadians(90)))
                .setTangent(Math.toRadians(-130))
                .splineToLinearHeading(new Pose2d(-28, 34, Math.toRadians(180)), Math.toRadians(-20))
                .build();
        TrajectorySequence DriveToStackLeft = drive.trajectorySequenceBuilder(PPreloadLeft.end())

                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-54, 11, Math.toRadians(180)), Math.toRadians(-140))
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(30, 14, Math.toRadians(180)), Math.toRadians(0))
                .build();
        TrajectorySequence DriveToBackBoardMiddle = drive.trajectorySequenceBuilder(DriveToStackMiddle.end())
                .setTangent(Math.toRadians(-30))
                .splineToLinearHeading(new Pose2d(53.5, 43.5, Math.toRadians(180)), Math.toRadians(0))
                .build();
        TrajectorySequence DriveToBackBoardLeft = drive.trajectorySequenceBuilder(DriveToStackLeft.end())
                .setTangent(Math.toRadians(70))
                .splineToLinearHeading(new Pose2d(53, 50, Math.toRadians(180)), Math.toRadians(0))
                .build();
        TrajectorySequence DriveToBackBoardRight = drive.trajectorySequenceBuilder(DriveToStackRight.end())
                .setTangent(Math.toRadians(-30))
                .splineToLinearHeading(new Pose2d(54.5, 35.5, Math.toRadians(180)), Math.toRadians(0))
                .build();
        TrajectorySequence DriveToBackParkM = drive.trajectorySequenceBuilder(DriveToStackMiddle.end())
                .setTangent(Math.toRadians(0))
                .lineToLinearHeading(new Pose2d(44, 14, Math.toRadians(180)))
                .build();
        TrajectorySequence DriveToBackBoardL = drive.trajectorySequenceBuilder(DriveToStackLeft.end())
                .setTangent(Math.toRadians(180))
                .lineToLinearHeading(new Pose2d(46, 47, Math.toRadians(180)))


                .build();
        TrajectorySequence DriveToBackBoardR = drive.trajectorySequenceBuilder(DriveToStackRight.end())
                .setTangent(Math.toRadians(180))
                .lineToLinearHeading(new Pose2d(44, 33.5, Math.toRadians(180)))
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
        ColourMassDetectionProcessor.PropPositions recordedPropPosition = colourMassDetectionProcessor.getRecordedPropPosition();

        // now we can use recordedPropPosition to determine where the prop is! if we never saw a prop, your recorded position will be UNFOUND.
        // if it is UNFOUND, you can manually set it to any of the other positions to guess
        if (recordedPropPosition == ColourMassDetectionProcessor.PropPositions.UNFOUND) {
            recordedPropPosition = ColourMassDetectionProcessor.PropPositions.MIDDLE;
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
            if(robot.liftA.getCurrentPosition() < 2200){
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
                        drive.followTrajectorySequenceAsync(PPreloadRight);
                    }
                    if (preloadpos == 2) {
                        drive.followTrajectorySequenceAsync(PPreloadMiddle);
                    }
                    if (preloadpos == 1) {
                        drive.followTrajectorySequenceAsync(PPreloadLeft);
                    }
                    target = 200;
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
                            target = 2300;
                            drive.followTrajectorySequenceAsync(DriveToBackBoardRight);
                            servo.reset();
                            stage = Stage.placePixel;

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
                    if(servoUp&& servo.seconds() > 3){
                        robot.L2.setPosition(robot.OUTTAKEB_OPEN);
                    }
                    if(servo.seconds() > 4) {
                        if (!drive.isBusy() && servoUp) {
                            robot.wristDown();
                            stage = Stage.park;
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
                        target = -50;
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
        ////////VISION END/
    }




}