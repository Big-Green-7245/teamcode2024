package org.firstinspires.ftc.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class AutoBasketPathTest {
    private static final Pose2d BASKET_INITIAL_POSE = new Pose2d(36, 61, 3 * Math.PI / 2);
    private static final Pose2d BASKET_POSE = new Pose2d(54, 54, 5 * Math.PI / 4);
    private static final Pose2d SAMPLE_1_POSE = new Pose2d(34, 26, 0);
    private static final Pose2d SAMPLE_2_POSE = new Pose2d(44, 26, 0);
    private static final Pose2d SAMPLE_3_POSE = new Pose2d(54, 26, 0);
    private static final Pose2d ASCENT_ZONE_POSE = new Pose2d(24, 12, Math.PI);

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity bot = new DefaultBotBuilder(meepMeep)
                .setConstraints(50, 50, Math.PI, Math.PI, 12)
                .setStartPose(BASKET_INITIAL_POSE)
                .build();

        bot.runAction(bot.getDrive().actionBuilder(BASKET_INITIAL_POSE)
                // Move to basket and deposit the preload sample
                .setTangent(3 * Math.PI / 2)
                .splineToLinearHeading(BASKET_POSE, Math.PI / 4)
                // Move to first sample while resetting output box and retracting slides
                .setTangent(5 * Math.PI / 4)
                .splineToLinearHeading(SAMPLE_1_POSE, 3 * Math.PI / 2)
                // Move to basket the second time and deposit
                .setTangent(Math.PI / 2)
                .splineToLinearHeading(BASKET_POSE, Math.PI / 4)
                // Move to second sample while resetting output box and retracting slides
                .setTangent(5 * Math.PI / 4)
                .splineToLinearHeading(SAMPLE_2_POSE, 3 * Math.PI / 2)
                // Move to basket the third time and deposit
                .setTangent(Math.PI / 2)
                .splineToLinearHeading(BASKET_POSE, Math.PI / 4)
                // Move to third sample while resetting output box and retracting slides
                .setTangent(5 * Math.PI / 4)
                .splineToLinearHeading(SAMPLE_3_POSE, 3 * Math.PI / 2)
                // Move to basket the fourth time and deposit
                .setTangent(Math.PI / 2)
                .splineToLinearHeading(BASKET_POSE, Math.PI / 4)
                // Move to ascent zone while resetting output box and retracting slides
                .setTangent(5 * Math.PI / 4)
                .splineTo(ASCENT_ZONE_POSE.position, Math.PI)
                .build()
        );

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .addEntity(bot)
                .start();
    }
}
