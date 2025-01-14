package org.firstinspires.ftc.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class AutoBasketPathTest {
    private static final Pose2d INITIAL_POSE = new Pose2d(36, 63, 3 * Math.PI / 2);

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(400);

        RoadRunnerBotEntity bot = new DefaultBotBuilder(meepMeep)
                .setConstraints(50, 50, Math.PI, Math.PI, 12)
                .setStartPose(INITIAL_POSE)
                .build();

        bot.runAction(bot.getDrive().actionBuilder(INITIAL_POSE)
                // Move to basket and deposit the preload sample
                .splineTo(new Vector2d(36, 36), 5 * Math.PI / 4)
                .splineToConstantHeading(new Vector2d(59, 59), Math.PI / 4)
                // Move to first sample while resetting output box and retracting slides
                .setTangent(5 * Math.PI / 4)
                .splineToSplineHeading(new Pose2d(36, 25, 0), 3 * Math.PI / 2)
                // Move to basket the second time and deposit
                .setTangent(Math.PI / 2)
                .splineToSplineHeading(new Pose2d(59, 59, 5 * Math.PI / 4), Math.PI / 4)
                // Move to second sample while resetting output box and retracting slides
                .setTangent(5 * Math.PI / 4)
                .splineToSplineHeading(new Pose2d(46, 25, 0), 3 * Math.PI / 2)
                // Move to basket the third time and deposit
                .setTangent(Math.PI / 2)
                .splineToSplineHeading(new Pose2d(59, 59, 5 * Math.PI / 4), Math.PI / 4)
                // Move to third sample while resetting output box and retracting slides
                .setTangent(5 * Math.PI / 4)
                .splineToSplineHeading(new Pose2d(56, 25, 0), 3 * Math.PI / 2)
                // Move to basket the fourth time and deposit
                .setTangent(Math.PI / 2)
                .splineToSplineHeading(new Pose2d(59, 59, 5 * Math.PI / 4), Math.PI / 4)
                .build()
        );

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .addEntity(bot)
                .start();
    }
}
