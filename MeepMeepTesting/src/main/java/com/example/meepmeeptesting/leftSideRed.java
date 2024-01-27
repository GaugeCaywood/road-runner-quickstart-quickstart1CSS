package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueLight;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;



public class leftSideRed {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        // Declare our first bot
        RoadRunnerBotEntity middle = new DefaultBotBuilder(meepMeep)
                // We set this bot to be blue
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(80, 80, Math.toRadians(180), Math.toRadians(180), 18)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-37, -61.2, Math.toRadians(90)))
                                .lineToLinearHeading(new Pose2d(-40, -23, Math.toRadians(180)))
                                .setTangent(Math.toRadians(160))
                                .splineToLinearHeading(new Pose2d(-57, -11, Math.toRadians(180)), Math.toRadians(180))
                                .setTangent(Math.toRadians(0))
                                .splineToLinearHeading(new Pose2d(34, -11, Math.toRadians(180)), Math.toRadians(-10))
                                .splineToLinearHeading(new Pose2d(54.5, -39, Math.toRadians(180)), Math.toRadians(-30))
                                .splineToLinearHeading(new Pose2d(51.5, -38, Math.toRadians(180)), Math.toRadians(-30))
                                .build()
                );
        RoadRunnerBotEntity left = new DefaultBotBuilder(meepMeep)
                // We set this bot to be blue
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(50, 50, Math.toRadians(180), Math.toRadians(180), 18)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-35, -60, Math.toRadians(-90)))
                                .lineToLinearHeading(new Pose2d(-48, -22, Math.toRadians(90)) )
                                .setTangent(Math.toRadians(120))
                                .splineToLinearHeading(new Pose2d(-55, -11, Math.toRadians(180)), Math.toRadians(180))
                                .setTangent(Math.toRadians(0))
                                .splineToLinearHeading(new Pose2d(34, -11, Math.toRadians(180)), Math.toRadians(-10))
                                .splineToLinearHeading(new Pose2d(54.5, -20, Math.toRadians(180)), Math.toRadians(-30))
                                .splineToLinearHeading(new Pose2d(51.5, -21, Math.toRadians(180)), Math.toRadians(-30))

                                .build()
                );
        RoadRunnerBotEntity right = new DefaultBotBuilder(meepMeep)
                // We set this bot to be blue
                .setColorScheme(new ColorSchemeBlueLight())
                .setConstraints(50, 50, Math.toRadians(180), Math.toRadians(180), 18)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-35, -60, Math.toRadians(-90)))
                                .setTangent(Math.toRadians(120))
                                .splineToLinearHeading(new Pose2d(-29, -34, Math.toRadians(180)), Math.toRadians(50))
                                .waitSeconds(10)
                                .setTangent(Math.toRadians(200))
                                .splineToLinearHeading(new Pose2d(-50, -11, Math.toRadians(180)), Math.toRadians(180))
                                .setTangent(Math.toRadians(0))
                                .splineToLinearHeading(new Pose2d(34, -11, Math.toRadians(180)), Math.toRadians(-10))
                                .splineToLinearHeading(new Pose2d(54.5, -46, Math.toRadians(180)), Math.toRadians(-30))
                                .splineToLinearHeading(new Pose2d(51.5, -46, Math.toRadians(180)), Math.toRadians(-30))
                                .build()
                );

        // Declare out second bot


        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(1.00f)

                // Add both of our declared bot entities
                .addEntity(middle)
                //.addEntity(left)
               // .addEntity(right)
                .start();
    }
}