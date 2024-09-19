package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.lang.Math;
public class DriveSubsystem {
    private DcMotor fr, br, fl, bl;
    private double speedModifier = 1.0;
    private SampleMecanumDrive drive;
    public boolean running = false;
    public boolean trajectory = false;

    public DriveSubsystem(HardwareMap hwMap) {
        drive = new SampleMecanumDrive(hwMap);
    }
    public void setSpeedModifier(double modifier) {
        this.speedModifier = modifier;
    }
    public void drive(double y, double lx, double rx) {
        drive.setWeightedDrivePower(new Pose2d(y, lx, rx));
        drive.update();
    }

    public void turn90Degrees() {
        drive.turn(Math.toRadians(92));
    }
    public void turnneg90Degrees() {
        drive.turn(Math.toRadians(-92));
    }
    public void moveToPosition(Pose2d targetPosition,double headings) {
        Trajectory trajectory = drive.trajectoryBuilder(drive.getPoseEstimate())
                .splineToLinearHeading(targetPosition,Math.toRadians(headings))
                .build();
        drive.followTrajectory(trajectory);
    }
    public Pose2d getCurrentPoz(){
        return drive.getPoseEstimate();
    }
    public void driveUpdate(Pose2d currentPoz){
        drive.setPoseEstimate(currentPoz);
    }
    public void goToPlaceRR(int color) {
        if(color == 1) {
            if (getCurrentPoz().getX() < 0 && getCurrentPoz().getY() > 0) {
                TrajectorySequence quadrant2ToPlace = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .splineToLinearHeading(new Pose2d(-15, 0, Math.toRadians(-45)), Math.toRadians(0))
                        .setTangent(Math.toRadians(0))
                        .splineToLinearHeading(new Pose2d(40, -45, Math.toRadians(180)), Math.toRadians(0))
                        .build();
                drive.followTrajectorySequenceAsync(quadrant2ToPlace);

            } else if (getCurrentPoz().getX() > 0 && getCurrentPoz().getY() > 0) {
                TrajectorySequence quadrant1ToPlace = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .splineToLinearHeading(new Pose2d(40, -45, Math.toRadians(180)), Math.toRadians(0))
                        .build();
                drive.followTrajectorySequenceAsync(quadrant1ToPlace);

            } else if (getCurrentPoz().getX() < 0 && getCurrentPoz().getY() < 0) {
                TrajectorySequence quadrant3ToPlace = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .setTangent(Math.toRadians(0))
                        .splineToLinearHeading(new Pose2d(-20, 0, Math.toRadians(180)), Math.toRadians(0))
                        .splineToLinearHeading(new Pose2d(20, 0, Math.toRadians(180)), Math.toRadians(0))
                        .splineToLinearHeading(new Pose2d(45, -46, Math.toRadians(180)), Math.toRadians(0))
                        .build();
                drive.followTrajectorySequenceAsync(quadrant3ToPlace);

            } else {
                TrajectorySequence quadrant4ToPlace = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .splineToLinearHeading(new Pose2d(45, -46, Math.toRadians(180)), Math.toRadians(0))
                        .build();
                drive.followTrajectorySequenceAsync(quadrant4ToPlace);
            }
        }
        else if(color==0) {
            if (getCurrentPoz().getX() < 0 && getCurrentPoz().getY() > 0) {
                TrajectorySequence quadrant2ToPlace = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .setTangent(Math.toRadians(0))
                        .splineToLinearHeading(new Pose2d(-20,0,Math.toRadians(180)),Math.toRadians(0))
                        .splineToLinearHeading(new Pose2d(20,10,Math.toRadians(180)),Math.toRadians(0))
                        .splineToLinearHeading(new Pose2d(45,35,Math.toRadians(180)),Math.toRadians(0))
                        .build();
                drive.followTrajectorySequenceAsync(quadrant2ToPlace);

            } else if (getCurrentPoz().getX() > 0 && getCurrentPoz().getY() > 0) {
                TrajectorySequence quadrant1ToPlace = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .splineToLinearHeading(new Pose2d(45,35,Math.toRadians(180)),Math.toRadians(0))
                        .build();
                drive.followTrajectorySequenceAsync(quadrant1ToPlace);

            } else if (getCurrentPoz().getX() < 0 && getCurrentPoz().getY() < 0) {
                TrajectorySequence quadrant3ToPlace = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .splineToLinearHeading(new Pose2d(-20,0,Math.toRadians(180)),Math.toRadians(0))
                        .splineToLinearHeading(new Pose2d(20,10,Math.toRadians(180)),Math.toRadians(0))
                        .splineToLinearHeading(new Pose2d(45,35,Math.toRadians(180)),Math.toRadians(0))
                        .build();
                drive.followTrajectorySequenceAsync(quadrant3ToPlace);

            } else {
                TrajectorySequence quadrant4ToPlace = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .splineToLinearHeading(new Pose2d(45,35,Math.toRadians(180)),Math.toRadians(0))
                        .build();
                drive.followTrajectorySequenceAsync(quadrant4ToPlace);
            }
        }
    }

    public void toCollect(int color){
        if(color == 1) {
            running = true;
            if (getCurrentPoz().getX() < 0 && getCurrentPoz().getY() > 0) {

                TrajectorySequence quadrant2ToCollect = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .splineToLinearHeading(new Pose2d(-45,45,Math.toRadians(90)),Math.toRadians(90))
                        .build();
                drive.followTrajectorySequence(quadrant2ToCollect);
                trajectory = true;

            } else if (getCurrentPoz().getX() > 0 && getCurrentPoz().getY() > 0) {

                TrajectorySequence quadrant1ToCollect = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .splineToLinearHeading(new Pose2d(10,0,Math.toRadians(90)),Math.toRadians(180))
                        .splineToLinearHeading(new Pose2d(-20,0,Math.toRadians(90)),Math.toRadians(180))
                        .splineToLinearHeading(new Pose2d(-45,45,Math.toRadians(90)),Math.toRadians(90))
                        .build();
                drive.followTrajectorySequence(quadrant1ToCollect);


            } else if (getCurrentPoz().getX() < 0 && getCurrentPoz().getY() < 0) {

                TrajectorySequence quadrant3ToCollect = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .splineToLinearHeading(new Pose2d(-45,45,Math.toRadians(90)),Math.toRadians(90))
                        .build();
                drive.followTrajectorySequence(quadrant3ToCollect);

            } else {
                TrajectorySequence quadrant4ToCollect = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .setTangent(Math.toRadians(180))
                        .splineToLinearHeading(new Pose2d(10,0,Math.toRadians(90)),Math.toRadians(180))
                        .splineToLinearHeading(new Pose2d(-20,0,Math.toRadians(90)),Math.toRadians(180))
                        .splineToLinearHeading(new Pose2d(-45,45,Math.toRadians(90)),Math.toRadians(90))
                        .build();
                drive.followTrajectorySequence(quadrant4ToCollect);
            }
            running = false;
        }
        else if(color == 0) {

            if (getCurrentPoz().getX() < 0 && getCurrentPoz().getY() > 0) {
                TrajectorySequence quadrant2ToCollect = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .splineToLinearHeading(new Pose2d(-45,-45,Math.toRadians(-90)),Math.toRadians(180))
                        .build();
                drive.followTrajectorySequence(quadrant2ToCollect);

            } else if (getCurrentPoz().getX() > 0 && getCurrentPoz().getY() > 0) {
                TrajectorySequence quadrant1ToCollect = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .setTangent(Math.toRadians(180))
                        .splineToLinearHeading(new Pose2d(10,0,Math.toRadians(-90)),Math.toRadians(180))
                        .splineToLinearHeading(new Pose2d(-30,0,Math.toRadians(-90)),Math.toRadians(180))
                        .splineToLinearHeading(new Pose2d(-50,-45,Math.toRadians(-90)),Math.toRadians(180))
                        .build();
                drive.followTrajectorySequence(quadrant1ToCollect);

            } else if (getCurrentPoz().getX() < 0 && getCurrentPoz().getY() < 0) {
                TrajectorySequence quadrant3ToCollect = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .splineToLinearHeading(new Pose2d(-45,-45,Math.toRadians(-90)),Math.toRadians(180))
                        .build();
                drive.followTrajectorySequence(quadrant3ToCollect);

            } else {
                TrajectorySequence quadrant4ToCollect = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .setTangent(Math.toRadians(180))
                        .splineToLinearHeading(new Pose2d(10,0,Math.toRadians(-90)),Math.toRadians(180))
                        .splineToLinearHeading(new Pose2d(-20,0,Math.toRadians(-90)),Math.toRadians(180))
                        .splineToLinearHeading(new Pose2d(-45,-45,Math.toRadians(-90)),Math.toRadians(180))
                        .build();
                drive.followTrajectorySequence(quadrant4ToCollect);
            }
        }
    }
    public static double worldXPosition = 50;
    public static double worldYPosition = 50;
    public static double movement_X = 0;
    public static double movement_Y = 0;
    public static double movement_turn = 0;
    public static double worldAngle_rad = 0;

    public boolean busy(){

        return drive.isBusy();
    }
}
