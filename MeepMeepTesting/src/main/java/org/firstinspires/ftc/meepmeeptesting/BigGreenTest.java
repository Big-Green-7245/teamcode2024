package org.firstinspires.ftc.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class BigGreenTest {
    private static final Pose2d INITIAL_POSE = new Pose2d(36, 63, 3 * Math.PI / 2);

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(400);

        RoadRunnerBotEntity bot = new DefaultBotBuilder(meepMeep)
                .setConstraints(50, 50, Math.PI, Math.PI, 12)
                .setStartPose(INITIAL_POSE)
                .build();

        bot.runAction(bot.getDrive().actionBuilder(INITIAL_POSE)
                .splineTo(new Vector2d(36, 36), 5 * Math.PI / 4)
                .build()
        );
        // Move to basket and deposit the preload sample
        bot.runAction(bot.getDrive().actionBuilder(new Pose2d(36, 36, 5 * Math.PI / 4))
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(59, 59, 5 * Math.PI / 4), 0)
                .build()
        );
        // Move to first sample while resetting output box and retracting slides
        bot.runAction(bot.getDrive().actionBuilder(new Pose2d(59, 59, 5 * Math.PI / 4))
                .splineTo(new Vector2d(36, 25), 3 * Math.PI / 2)
                .turnTo(0)
                .build()
        );
        // Move to basket the second time and deposit
        bot.runAction(bot.getDrive().actionBuilder(new Pose2d(36, 25, 0))
                .turnTo(3 * Math.PI / 2)
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(59, 59, 5 * Math.PI / 4), 0)
                .build()
        );
        // Move to second sample while resetting output box and retracting slides
        bot.runAction(bot.getDrive().actionBuilder(new Pose2d(59, 59, 5 * Math.PI / 4))
                .splineTo(new Vector2d(36, 25), 3 * Math.PI / 4)
                .turnTo(0)
                .build()
        );
    }
}
