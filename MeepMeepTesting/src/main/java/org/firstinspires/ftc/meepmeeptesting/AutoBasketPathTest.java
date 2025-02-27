package org.firstinspires.ftc.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.util.List;

public class AutoBasketPathTest {
    private static final Pose2d BASKET_INITIAL_POSE = new Pose2d(36, 61, 3 * Math.PI / 2);
    private static final Pose2d BASKET_POSE = new Pose2d(55, 55, 5 * Math.PI / 4);
    private static final Pose2d SAMPLE_1_POSE = new Pose2d(38, 35.5, 7 * Math.PI / 4); // 48, 25.5
    private static final Pose2d SAMPLE_2_POSE = new Pose2d(48, 35.5, 7 * Math.PI / 4); // 58, 25.5
    private static final Pose2d SAMPLE_3_POSE = new Pose2d(55, 25.5, 0); // 68, 25.5
    private static final List<Pose2d> SAMPLE_POSES = List.of(SAMPLE_1_POSE, SAMPLE_2_POSE, SAMPLE_3_POSE);
    private static final Pose2d SAMPLE_SUBMERSIBLE_POSE_1 = new Pose2d(24, 10, Math.PI);
    private static final Pose2d SAMPLE_SUBMERSIBLE_POSE_2 = new Pose2d(24, 8, Math.PI);
    private static final List<Pose2d> SAMPLE_SUBMERSIBLE_POSES = List.of(SAMPLE_SUBMERSIBLE_POSE_1, SAMPLE_SUBMERSIBLE_POSE_2);
    private static final Pose2d ASCENT_ZONE_POSE_PARKING = new Pose2d(22, 8, 0);

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity bot = new DefaultBotBuilder(meepMeep)
                .setConstraints(60, 60, Math.PI, Math.PI, 14)
                .setStartPose(BASKET_INITIAL_POSE)
                .build();

        // Move to basket and deposit the preload sample
        TrajectoryActionBuilder builder = bot.getDrive().actionBuilder(BASKET_INITIAL_POSE)
                .setTangent(3 * Math.PI / 2)
                .splineToLinearHeading(BASKET_POSE, Math.PI / 4);

        for (Pose2d samplePose : SAMPLE_POSES) {
            // Move to sample while resetting output box and retracting slides
            builder = builder
                    .setTangent(5 * Math.PI / 4)
                    .splineToSplineHeading(samplePose, samplePose.heading);

            // Move to basket and deposit
            builder = builder
                    .endTrajectory()
                    .lineToX(samplePose.position.x + 4 * samplePose.heading.real)
                    .splineToSplineHeading(BASKET_POSE, Math.PI / 4);
        }

        for (Pose2d submersiblePose : SAMPLE_SUBMERSIBLE_POSES) {
            builder = builder
                    // Move to ascent zone while resetting output box and retracting slides
                    .setTangent(5 * Math.PI / 4)
                    .splineTo(submersiblePose.position, Math.PI)
                    // Intake a sample and move to basket
                    .setTangent(3 * Math.PI / 2)
                    .lineToYConstantHeading(submersiblePose.position.y - 4)
                    .lineToYConstantHeading(submersiblePose.position.y)
                    // Move to basket and deposit
                    .setTangent(0)
                    .splineToLinearHeading(BASKET_POSE, Math.PI / 4);
        }

        // Move to ascent zone while resetting output box and retracting slides
        builder = builder
                .setTangent(5 * Math.PI / 4)
                .splineToLinearHeading(ASCENT_ZONE_POSE_PARKING, Math.PI);

        bot.runAction(builder.build());
        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .addEntity(bot)
                .start();
    }
}
