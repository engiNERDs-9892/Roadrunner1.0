package com.example.lib;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);


        RoadRunnerBotEntity BlueRight = new DefaultBotBuilder(meepMeep)

                .setColorScheme(new ColorSchemeBlueDark())
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 16.5)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-36, 60, Math.toRadians(270.00)))

                                .splineTo(new Vector2d(-36, 32.00), Math.toRadians(270.00))
                                .setReversed(true)
                                .splineTo(new Vector2d(-31.18, 34.15), Math.toRadians(8.95))
                                .splineToLinearHeading(new Pose2d(55.00, 35.00, Math.toRadians(-180.00)), Math.toRadians(0.00))
                                .setReversed(false)
                                .lineToConstantHeading(new Vector2d(-18.02, 35.95))
                                .splineToConstantHeading(new Vector2d(-61.81, 23.50), Math.toRadians(180.00))
                                .setReversed(true)
                                .lineToConstantHeading(new Vector2d(-61.70, 35.70))
                                .lineToConstantHeading(new Vector2d(55.00, 34.00))
                                .setReversed(false)
                                .lineToConstantHeading(new Vector2d(-18.02, 35.95))
                                .splineToConstantHeading(new Vector2d(-61.81, 23.50), Math.toRadians(180.00))
                                .setReversed(true)
                                .lineToConstantHeading(new Vector2d(-61.70, 35.70))
                                .lineToConstantHeading(new Vector2d(55.00, 34.00))
                                .setReversed(false)
                                .lineToConstantHeading(new Vector2d(-18.02, 35.95))
                                .splineToConstantHeading(new Vector2d(-61.81, 23.50), Math.toRadians(180.00))
                                .setReversed(true)
                                .lineToConstantHeading(new Vector2d(-61.70, 35.70))
                                .lineToConstantHeading(new Vector2d(55.00, 34.00))
                                .setReversed(false)


                                .build()
                );


        RoadRunnerBotEntity BlueLeft = new DefaultBotBuilder(meepMeep)

                .setColorScheme(new ColorSchemeBlueDark())
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 16.5)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(12, 60, Math.toRadians(-90.00)))

                                .splineTo(new Vector2d(12.00, 32.00), Math.toRadians(270.00))
                                .setReversed(true)
                                .splineToLinearHeading(new Pose2d(55.00, 34.00, Math.toRadians(540.00)), Math.toRadians(360.00))
                                .setReversed(false)
                                .lineToConstantHeading(new Vector2d(-18.02, 35.95))
                                .splineToConstantHeading(new Vector2d(-61.81, 23.50), Math.toRadians(180.00))
                                .setReversed(true)
                                .lineToConstantHeading(new Vector2d(-61.70, 35.70))
                                .lineToConstantHeading(new Vector2d(55.00, 34.00))
                                .setReversed(false)
                                .lineToConstantHeading(new Vector2d(-18.02, 35.95))
                                .splineToConstantHeading(new Vector2d(-61.81, 23.50), Math.toRadians(180.00))
                                .setReversed(true)
                                .lineToConstantHeading(new Vector2d(-61.70, 35.70))
                                .lineToConstantHeading(new Vector2d(55.00, 34.00))
                                .setReversed(false)
                                .lineToConstantHeading(new Vector2d(-18.02, 35.95))
                                .splineToConstantHeading(new Vector2d(-61.81, 23.50), Math.toRadians(180.00))
                                .setReversed(true)
                                .lineToConstantHeading(new Vector2d(-61.70, 35.70))
                                .lineToConstantHeading(new Vector2d(55.00, 34.00))
                                .setReversed(false)


                                .build()
                );

        RoadRunnerBotEntity RedRight = new DefaultBotBuilder(meepMeep)

                .setColorScheme(new ColorSchemeRedDark())
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(60), Math.toRadians(60), 16.5)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(12, -60, Math.toRadians(90)))

                                .splineTo(new Vector2d(12.00, -32.00), Math.toRadians(90.00))
                                .setReversed(true)
                                .splineToLinearHeading(new Pose2d(55.00, -34.00, Math.toRadians(-180.00)), Math.toRadians(0.00))
                                .setReversed(false)
                                .lineToConstantHeading(new Vector2d(-18.02, -35.95))
                                .splineToConstantHeading(new Vector2d(-61.81, -23.50), Math.toRadians(180.00))
                                .setReversed(true)
                                .lineToConstantHeading(new Vector2d(-61.70, -35.70))
                                .lineToConstantHeading(new Vector2d(55.00, -34.00))
                                .setReversed(false)
                                .lineToConstantHeading(new Vector2d(-18.02, -35.95))
                                .splineToConstantHeading(new Vector2d(-61.81, -23.50), Math.toRadians(180.00))
                                .setReversed(true)
                                .lineToConstantHeading(new Vector2d(-61.70, -35.70))
                                .lineToConstantHeading(new Vector2d(55.00, -34.00))
                                .setReversed(false).lineToConstantHeading(new Vector2d(-18.02, -35.95))
                                .splineToConstantHeading(new Vector2d(-61.81, -23.50), Math.toRadians(180.00))
                                .setReversed(true)
                                .lineToConstantHeading(new Vector2d(-61.70, -35.70))
                                .lineToConstantHeading(new Vector2d(55.00, -34.00))
                                .setReversed(false)









                                .build()
                );

        RoadRunnerBotEntity RedLeft = new DefaultBotBuilder(meepMeep)

                .setColorScheme(new ColorSchemeRedDark())
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 16.5)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-36, -60, Math.toRadians(90)))

                                .splineTo(new Vector2d(-36.00, -32.00), Math.toRadians(90.00))
                                .setReversed(true)
                                .splineTo(new Vector2d(-31.18, -36.15), Math.toRadians(8.95))
                                .splineToLinearHeading(new Pose2d(55.00, -35.00, Math.toRadians(-180.00)), Math.toRadians(0.00))
                                .setReversed(false)
                                .lineToConstantHeading(new Vector2d(-18.02, -35.95))
                                .splineToConstantHeading(new Vector2d(-61.81, -23.50), Math.toRadians(180.00))
                                .setReversed(true)
                                .lineToConstantHeading(new Vector2d(-61.70, -35.70))
                                .lineToConstantHeading(new Vector2d(55.00, -34.00))
                                .setReversed(false)
                                .lineToConstantHeading(new Vector2d(-18.02, -35.95))
                                .splineToConstantHeading(new Vector2d(-61.81, -23.50), Math.toRadians(180.00))
                                .setReversed(true)
                                .lineToConstantHeading(new Vector2d(-61.70, -35.70))
                                .lineToConstantHeading(new Vector2d(55.00, -34.00))
                                .setReversed(false).lineToConstantHeading(new Vector2d(-18.02, -35.95))
                                .splineToConstantHeading(new Vector2d(-61.81, -23.50), Math.toRadians(180.00))
                                .setReversed(true)
                                .lineToConstantHeading(new Vector2d(-61.70, -35.70))
                                .lineToConstantHeading(new Vector2d(55.00, -34.00))
                                .setReversed(false)

                                .build()
                );



        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_LIGHT)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                //.addEntity(RedLeft)
                //.addEntity(RedRight)
                //.addEntity(BlueLeft)
                //.addEntity(BlueRight)
                .start();
    }
}