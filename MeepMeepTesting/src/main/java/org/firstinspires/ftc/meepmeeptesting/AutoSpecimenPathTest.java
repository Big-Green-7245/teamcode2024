package org.firstinspires.ftc.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.util.List;

public class AutoSpecimenPathTest {
    private static final Pose2d SPECIMEN_INITIAL_POSE = new Pose2d(-12, 63, Math.PI / 2);
    private static final double SUBMERSIBLE_Y = 34.5;
    private static final Pose2d INITIAL_SUBMERSIBLE_POSE = new Pose2d(-4, SUBMERSIBLE_Y, Math.PI / 2);
    private static final int OBSERVATION_ZONE_Y = 48;
    private static final Pose2d OBSERVATION_ZONE_POSE = new Pose2d(-36, 60, 3 * Math.PI / 2);
    private static final Pose2d SPECIMEN_SAMPLE_1_INTERMEDIATE_1 = new Pose2d(-35, 32, 3 * Math.PI / 2);
    private static final Pose2d SPECIMEN_SAMPLE_1_INTERMEDIATE_2 = new Pose2d(-35, 28, 3 * Math.PI / 2);
    private static final Pose2d SPECIMEN_SAMPLE_1_POSE = new Pose2d(-42, 14, 3 * Math.PI / 2);
    private static final Pose2d SPECIMEN_SAMPLE_1_DEPOSIT_POSE = new Pose2d(-44, OBSERVATION_ZONE_Y, 3 * Math.PI / 2);
    private static final Pose2d SPECIMEN_SAMPLE_2_POSE = new Pose2d(-52, 14, 3 * Math.PI / 2);
    private static final Pose2d SPECIMEN_SAMPLE_2_DEPOSIT_POSE = new Pose2d(-54, OBSERVATION_ZONE_Y, 3 * Math.PI / 2);
    private static final Pose2d SPECIMEN_SAMPLE_3_POSE = new Pose2d(-62, 14, 3 * Math.PI / 2);
    private static final Pose2d SPECIMEN_SAMPLE_3_DEPOSIT_POSE = new Pose2d(-62, OBSERVATION_ZONE_Y, 3 * Math.PI / 2);
    private static final Pose2d SUBMERSIBLE_1_POSE = new Pose2d(-6, SUBMERSIBLE_Y, Math.PI / 2);
    private static final Pose2d SUBMERSIBLE_2_POSE = new Pose2d(-8, SUBMERSIBLE_Y, Math.PI / 2);
    private static final Pose2d SUBMERSIBLE_3_POSE = new Pose2d(-10, SUBMERSIBLE_Y, Math.PI / 2);
    private static final Pose2d SUBMERSIBLE_4_POSE = new Pose2d(-12, SUBMERSIBLE_Y, Math.PI / 2);
    private static final List<Pose2d> SUBMERSIBLE_POSES = List.of(SUBMERSIBLE_1_POSE, SUBMERSIBLE_2_POSE, SUBMERSIBLE_3_POSE, SUBMERSIBLE_4_POSE);

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity bot = new DefaultBotBuilder(meepMeep)
                .setConstraints(50, 50, Math.PI, Math.PI, 12)
                .setStartPose(SPECIMEN_INITIAL_POSE)
                .build();

        TrajectoryActionBuilder builder = bot.getDrive().actionBuilder(SPECIMEN_INITIAL_POSE)
                // Move to submersible and deposit the preload specimen
                .setTangent(3 * Math.PI / 2)
                .splineToConstantHeading(INITIAL_SUBMERSIBLE_POSE.position, 3 * Math.PI / 2)
                // Move to first sample while resetting specimen claw and retracting slides
                .setTangent(Math.PI / 2)
                .splineTo(SPECIMEN_SAMPLE_1_INTERMEDIATE_1.position, 3 * Math.PI / 2)
                .splineToSplineHeading(SPECIMEN_SAMPLE_1_INTERMEDIATE_2, 3 * Math.PI / 2)
                .splineToConstantHeading(SPECIMEN_SAMPLE_1_POSE.position, Math.PI / 2)
                // Push first sample to observation zone
                .splineToConstantHeading(SPECIMEN_SAMPLE_1_DEPOSIT_POSE.position, Math.PI / 2)
                // Move to second sample
                .setTangent(3 * Math.PI / 2)
                .splineToConstantHeading(SPECIMEN_SAMPLE_2_POSE.position, Math.PI / 2)
                // Push second sample to observation zone
                .splineToConstantHeading(SPECIMEN_SAMPLE_2_DEPOSIT_POSE.position, Math.PI / 2)
                // Move to third sample
                .setTangent(3 * Math.PI / 2)
                .splineToConstantHeading(SPECIMEN_SAMPLE_3_POSE.position, Math.PI / 2)
                // Push third sample to observation zone
                .splineToConstantHeading(SPECIMEN_SAMPLE_3_DEPOSIT_POSE.position, Math.PI / 2)
                // Move to observation zone and pick up second specimen
                .splineToConstantHeading(OBSERVATION_ZONE_POSE.position, Math.PI / 2);

        for (Pose2d submersiblePose : SUBMERSIBLE_POSES) {
            // Pick up specimen and move to submersible
            builder = builder
                    .setTangent(7 * Math.PI / 4)
                    .splineToLinearHeading(submersiblePose, 7 * Math.PI / 4);

            // Deposit the specimen and move to observation zone
            builder = builder
                    .setTangent(3 * Math.PI / 4)
                    .splineToLinearHeading(OBSERVATION_ZONE_POSE, Math.PI / 2);
        }

        bot.runAction(builder.build());
        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .addEntity(bot)
                .start();
    }
}
