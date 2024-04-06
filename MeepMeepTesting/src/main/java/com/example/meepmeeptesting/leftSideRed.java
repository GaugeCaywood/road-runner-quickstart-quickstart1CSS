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
                .setConstraints(50, 50, Math.toRadians(180), Math.toRadians(180), 17)
                .followTrajectorySequence(drive ->
drive.trajectorySequenceBuilder(new Pose2d(-57, -38,Math.toRadians(180)))
        .setTangent(Math.toRadians(-90))
        .splineToLinearHeading(new Pose2d(-50,-68,Math.toRadians(180)), Math.toRadians(0))
        .build()
                );
        RoadRunnerBotEntity left = new DefaultBotBuilder(meepMeep)
                // We set this bot to be blue
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(50, 50, Math.toRadians(180), Math.toRadians(180), 18)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(52.5, -25, Math.toRadians(180)))

                                .splineToLinearHeading(new Pose2d(30, -57,Math.toRadians(180)),Math.toRadians(180))
                                .build()
                );
        RoadRunnerBotEntity right = new DefaultBotBuilder(meepMeep)
                // We set this bot to be blue
                .setColorScheme(new ColorSchemeBlueLight())
                .setConstraints(50, 50, Math.toRadians(180), Math.toRadians(180), 18)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-35, -60, Math.toRadians(-90)))
                                .setTangent(Math.toRadians(0))
                                .splineToLinearHeading(new Pose2d(48, -35, Math.toRadians(180)), Math.toRadians(-60))
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