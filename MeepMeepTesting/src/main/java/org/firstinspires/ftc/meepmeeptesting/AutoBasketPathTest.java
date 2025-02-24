package org.firstinspires.ftc.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class AutoBasketPathTest {
    private static final Pose2d BASKET_INITIAL_POSE = new Pose2d(36, 61, 3 * Math.PI / 2);
    private static final Pose2d BASKET_POSE = new Pose2d(55, 55, 5 * Math.PI / 4);
    private static final Pose2d SAMPLE_1_POSE = new Pose2d(38, 35.5, 7 * Math.PI / 4); // 48, 25.5
    private static final Pose2d SAMPLE_2_POSE = new Pose2d(48, 35.5, 7 * Math.PI / 4); // 58, 25.5
    private static final Pose2d SAMPLE_3_POSE = new Pose2d(55, 25.5, 0); // 68, 25.5
    private static final Pose2d ASCENT_ZONE_POSE = new Pose2d(24, 12, Math.PI);
    private static final Pose2d ASCENT_ZONE_POSE_2 = new Pose2d(22, 10, 0);

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity bot = new DefaultBotBuilder(meepMeep)
                .setConstraints(60, 60, Math.PI, Math.PI, 14)
                .setStartPose(BASKET_INITIAL_POSE)
                .build();

        bot.runAction(bot.getDrive().actionBuilder(BASKET_INITIAL_POSE)
                // Move to basket and deposit the preload sample
                .setTangent(3 * Math.PI / 2)
                .splineToLinearHeading(BASKET_POSE, Math.PI / 4)
                // Move to first sample while resetting output box and retracting slides
                .setTangent(5 * Math.PI / 4)
                .splineToSplineHeading(SAMPLE_1_POSE, SAMPLE_1_POSE.heading)
                // Move to basket the second time and deposit
                .lineToX(SAMPLE_1_POSE.position.x + 4 * SAMPLE_1_POSE.heading.real)
                .splineToSplineHeading(BASKET_POSE, Math.PI / 4)
                // Move to second sample while resetting output box and retracting slides
                .setTangent(5 * Math.PI / 4)
                .splineToSplineHeading(SAMPLE_2_POSE, SAMPLE_2_POSE.heading)
                // Move to basket the third time and deposit
                .lineToX(SAMPLE_2_POSE.position.x + 4 * SAMPLE_2_POSE.heading.real)
                .splineToSplineHeading(BASKET_POSE, Math.PI / 4)
                // Move to third sample while resetting output box and retracting slides
                .setTangent(5 * Math.PI / 4)
                .splineToSplineHeading(SAMPLE_3_POSE, SAMPLE_3_POSE.heading)
                // Move to basket the fourth time and deposit
                .lineToX(SAMPLE_3_POSE.position.x + 4 * SAMPLE_3_POSE.heading.real)
                .splineToSplineHeading(BASKET_POSE, Math.PI / 4)
                // Move to ascent zone while resetting output box and retracting slides
                .setTangent(5 * Math.PI / 4)
                .splineTo(ASCENT_ZONE_POSE.position, Math.PI)
                // Intake a sample and move to basket
                .setTangent(3 * Math.PI / 2)
                .lineToYConstantHeading(ASCENT_ZONE_POSE.position.y - 4)
                .lineToYConstantHeading(ASCENT_ZONE_POSE.position.y)
                // Move to basket the fifth time and deposit
                .setTangent(0)
                .splineToLinearHeading(BASKET_POSE, Math.PI / 4)
                // Move to ascent zone while resetting output box and retracting slides
                .setTangent(5 * Math.PI / 4)
                .splineToLinearHeading(ASCENT_ZONE_POSE_2, Math.PI)
                .build()
        );

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .addEntity(bot)
                .start();
    }
}
