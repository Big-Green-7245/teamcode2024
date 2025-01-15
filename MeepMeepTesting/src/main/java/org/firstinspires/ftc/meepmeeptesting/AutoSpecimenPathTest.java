package org.firstinspires.ftc.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class AutoSpecimenPathTest {
    private static final Pose2d SPECIMEN_INITIAL_POSE = new Pose2d(-12, 61, Math.PI / 2);
    private static final Pose2d INITIAL_SUBMERSIBLE_POSE = new Pose2d(-6, 36, Math.PI / 2);
    private static final int OBSERVATION_ZONE_Y = 48;
    private static final Pose2d OBSERVATION_ZONE_POSE = new Pose2d(-36, 60, 3 * Math.PI / 2);
    private static final Pose2d SUBMERSIBLE_POSE = new Pose2d(-8, 36, Math.PI / 2);
    private static final Pose2d SUBMERSIBLE_SIDE_POSE = new Pose2d(-4, 36, Math.PI / 2);
    private static final Pose2d SPECIMEN_SAMPLE_1_POSE = new Pose2d(-41, 14, 3 * Math.PI / 2);
    private static final Pose2d SPECIMEN_SAMPLE_1_DEPOSIT_POSE = new Pose2d(-42, OBSERVATION_ZONE_Y, 3 * Math.PI / 2);
    private static final Pose2d SPECIMEN_SAMPLE_2_POSE = new Pose2d(-51, 14, 3 * Math.PI / 2);
    private static final Pose2d SPECIMEN_SAMPLE_2_DEPOSIT_POSE = new Pose2d(-52, OBSERVATION_ZONE_Y, 3 * Math.PI / 2);
    private static final Pose2d SPECIMEN_SAMPLE_3_POSE = new Pose2d(-61, 14, 3 * Math.PI / 2);
    private static final Pose2d SPECIMEN_SAMPLE_3_DEPOSIT_POSE = new Pose2d(-61, OBSERVATION_ZONE_Y, 3 * Math.PI / 2);

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity bot = new DefaultBotBuilder(meepMeep)
                .setConstraints(50, 50, Math.PI, Math.PI, 12)
                .setStartPose(SPECIMEN_INITIAL_POSE)
                .build();

        bot.runAction(bot.getDrive().actionBuilder(SPECIMEN_INITIAL_POSE)
                // Move to submersible and deposit the preload specimen
                .setTangent(3 * Math.PI / 2)
                .splineToConstantHeading(INITIAL_SUBMERSIBLE_POSE.position, 3 * Math.PI / 2)
                // Move to first sample while resetting specimen claw and retracting slides
                .setTangent(Math.PI / 2)
                .splineTo(new Vector2d(-36, 32), 3 * Math.PI / 2)
                .splineToSplineHeading(SPECIMEN_SAMPLE_1_POSE, Math.PI / 2)
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
                .splineToConstantHeading(OBSERVATION_ZONE_POSE.position, Math.PI / 2)
                // Move to submersible to deposit second specimen
                .setTangent(3 * Math.PI / 2)
                .splineToLinearHeading(SUBMERSIBLE_POSE, 3 * Math.PI / 2)
                // Move to the side while depositing the second specimen
                .setTangent(0)
//                .splineToConstantHeading(SUBMERSIBLE_SIDE_POSE.position, 0)
                // Move to observation zone and pick up third specimen
//                .setTangent(Math.PI / 2)
                .splineToLinearHeading(OBSERVATION_ZONE_POSE, Math.PI / 2)
                // Move to submersible to deposit third specimen
                .setTangent(3 * Math.PI / 2)
                .splineToLinearHeading(SUBMERSIBLE_POSE, 3 * Math.PI / 2)
                // Move to the side while depositing the third specimen
                .setTangent(0)
//                .splineToConstantHeading(SUBMERSIBLE_SIDE_POSE.position, 0)
                // Move to observation zone and pick up fourth specimen
//                .setTangent(Math.PI / 2)
                .splineToLinearHeading(OBSERVATION_ZONE_POSE, Math.PI / 2)
                // Move to submersible to deposit fourth specimen
                .setTangent(3 * Math.PI / 2)
                .splineToLinearHeading(SUBMERSIBLE_POSE, 3 * Math.PI / 2)
                // Move to the side while depositing the fourth specimen
                .setTangent(0)
//                .splineToConstantHeading(SUBMERSIBLE_SIDE_POSE.position, 0)
                // Move to observation zone and pick up fifth specimen
//                .setTangent(Math.PI / 2)
                .splineToLinearHeading(OBSERVATION_ZONE_POSE, Math.PI / 2)
                // Move to submersible to deposit fifth specimen
                .setTangent(3 * Math.PI / 2)
                .splineToLinearHeading(SUBMERSIBLE_POSE, 3 * Math.PI / 2)
                // Move to the side while depositing the fifth specimen
//                .setTangent(0)
//                .splineToConstantHeading(SUBMERSIBLE_SIDE_POSE.position, 0)
                .build()
        );

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .addEntity(bot)
                .start();
    }
}
