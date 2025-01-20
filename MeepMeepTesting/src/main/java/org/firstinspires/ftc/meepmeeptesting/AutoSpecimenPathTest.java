package org.firstinspires.ftc.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.util.List;

public class AutoSpecimenPathTest {
    private static final Pose2d SPECIMEN_INITIAL_POSE = new Pose2d(-12, 63, Math.PI / 2);
    private static final double SUBMERSIBLE_Y = 34;
    private static final Pose2d INITIAL_SUBMERSIBLE_POSE = new Pose2d(-4, SUBMERSIBLE_Y, Math.PI / 2);
    private static final int SPECIMEN_SAMPLE_PICKUP_Y = 42;
    private static final Pose2d SPECIMEN_SAMPLE_1_POSE = new Pose2d(-32, SPECIMEN_SAMPLE_PICKUP_Y, 5 * Math.PI / 4);
    private static final Pose2d SPECIMEN_SAMPLE_2_POSE = new Pose2d(-42, SPECIMEN_SAMPLE_PICKUP_Y, 5 * Math.PI / 4);
    private static final Pose2d SPECIMEN_SAMPLE_3_POSE = new Pose2d(-52, SPECIMEN_SAMPLE_PICKUP_Y, 5 * Math.PI / 4);
    private static final List<Pose2d> SPECIMEN_SAMPLE_POSES = List.of(SPECIMEN_SAMPLE_1_POSE, SPECIMEN_SAMPLE_2_POSE, SPECIMEN_SAMPLE_3_POSE);
    private static final Pose2d OBSERVATION_ZONE_POSE = new Pose2d(-36, 60, 3 * Math.PI / 2);
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
                .setTangent(3 * Math.PI / 4)
                // Move to sample while starting intake
                .splineToLinearHeading(SPECIMEN_SAMPLE_1_POSE, Math.PI);

        for (int i = 1; i < SPECIMEN_SAMPLE_POSES.size(); i++) {
            builder = builder
                    // Intake and turn to observation zone
                    .turnTo(3 * Math.PI / 4)
                    // Spit out the sample and move to the next sample while starting intake
                    .setTangent(Math.PI)
                    .splineToLinearHeading(SPECIMEN_SAMPLE_POSES.get(i), Math.PI);
        }

        builder = builder
                // Intake and turn to observation zone
                .turnTo(3 * Math.PI / 4)
                // Move to observation zone to pick up specimen
                .setTangent(Math.PI / 4)
                .splineToLinearHeading(OBSERVATION_ZONE_POSE, Math.PI / 2);

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
