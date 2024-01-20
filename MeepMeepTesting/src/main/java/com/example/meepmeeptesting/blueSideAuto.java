package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueLight;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;



public class blueSideAuto {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        // Declare our first bot
        RoadRunnerBotEntity middle = new DefaultBotBuilder(meepMeep)
                // We set this bot to be blue
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 18)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-37, 61.2, Math.toRadians(90)))
                                .setTangent(Math.toRadians(-120))
                                .splineToLinearHeading(new Pose2d(-45, 23, Math.toRadians(-90)), Math.toRadians(-90))
                                .waitSeconds(0.5)

                                .setTangent(Math.toRadians(-100))
                                .splineToLinearHeading(new Pose2d(-54, 11, Math.toRadians(180)), Math.toRadians(-180))




                                .setTangent(Math.toRadians(0))
                                .splineToLinearHeading(new Pose2d(34, 11, Math.toRadians(180)), Math.toRadians(0))

                                .setTangent(Math.toRadians(-30))
                                .splineToLinearHeading(new Pose2d(55, 33.5, Math.toRadians(180)), Math.toRadians(0))

                                .waitSeconds(0.5)

                                .build()
                );
        RoadRunnerBotEntity left = new DefaultBotBuilder(meepMeep)
                // We set this bot to be blue
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-35, -60, Math.toRadians(-90)))
                                .lineToLinearHeading(new Pose2d(-35, -34, Math.toRadians(90)))

                                .waitSeconds(1)
                                .lineToLinearHeading(new Pose2d(33,-34,Math.toRadians(90)))
                                .lineToLinearHeading(new Pose2d(46,-34,Math.toRadians(0)))
                                .splineToLinearHeading(new Pose2d(43.15, -15.92, Math.toRadians(0.00)), Math.toRadians(52.70))
                                .lineToLinearHeading(new Pose2d(62.08, -11.15, Math.toRadians(0.00)))
                                .build()
                );
        RoadRunnerBotEntity right = new DefaultBotBuilder(meepMeep)
                // We set this bot to be blue
                .setColorScheme(new ColorSchemeBlueLight())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-35, -60, Math.toRadians(-90)))
                                .lineToLinearHeading(new Pose2d(-35, -34, Math.toRadians(0)))
                                .waitSeconds(1)
                                .lineToLinearHeading(new Pose2d(33,-34,Math.toRadians(0)))
                                .lineToLinearHeading(new Pose2d(46,-34,Math.toRadians(0)))
                                .splineToLinearHeading(new Pose2d(43.15, -15.92, Math.toRadians(0.00)), Math.toRadians(52.70))
                                .lineToLinearHeading(new Pose2d(62.08, -11.15, Math.toRadians(0.00)))
                                .build()
                );

        // Declare out second bot


        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)

                // Add both of our declared bot entities
                .addEntity(middle)
                //             .addEntity(left)
                //               .addEntity(right)
                .start();
    }
}