package com.example.meepmeep;

import static java.lang.Math.PI;
import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.acmerobotics.roadrunner.Pose2d;

public class MeepMeep {

    static int randomization = 2;

    static boolean
            isRed = true,
            isParkedMiddle = true,
            isUnderTruss = false,
            doAprilTag = false,
            runScore = false;

    public static double
            START_X = 12;

    public static double
            BACKBOARD_X = 50.4,
            ANGLE_1 = 52,
            ANGLE_2 = 46,
            ANGLE_3 = 32.5,
            ANGLE_4 = 20,
            LEFT = toRadians(180),
            FORWARD = toRadians(90),
            RIGHT = toRadians(0),
            BACKWARD = toRadians(270);
    public static Pose2d
            start = new Pose2d(START_X, -61.788975, BACKWARD),
            spikeLeft = new Pose2d((START_X - 5.45), -34.5, toRadians(135)),
            spikeCenter = new Pose2d((START_X + 6), -34.25, toRadians(0)),
            spikeRight = new Pose2d((START_X + 7.75), -35, toRadians(45)),
            backboardLeft = new Pose2d(BACKBOARD_X, -30.5, LEFT),
            backboardCenter = new Pose2d(BACKBOARD_X, -35, LEFT),
            backboardRight = new Pose2d(BACKBOARD_X, -41, LEFT),
            parkingLeft = new Pose2d(48.5, -10, toRadians(165)),
            parkingRight = new Pose2d(48.5, -56, toRadians(200)),
            spikeDodgeStageDoor = new Pose2d(23, -10, LEFT),
            stageDoor = new Pose2d(16, -10, LEFT),
            outerTruss = new Pose2d(23.5, -58, LEFT),
            pixelStack1 = new Pose2d(-56.5, -12, LEFT),
            pixelStack3 = new Pose2d(-56.5, -35, LEFT),
            mainSpike = null,
            yellowScoring = null,
            transition = null,
            whiteScoring = null,
            pixelStack = null;

    public static void main(String[] args) {
        com.noahbres.meepmeep.MeepMeep meepMeep = new com.noahbres.meepmeep.MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();


        TrajectoryActionBuilder builder = myBot.getDrive().actionBuilder(start);

        setLogic();

        builder = scorePurplePixel(builder);
        builder = scoreYellowPixel(builder);
        builder = builder.strafeTo(parkingRight.position);

        myBot.runAction(builder.build());




        meepMeep.setBackground(com.noahbres.meepmeep.MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }

    public static void setLogic() {
        switch (randomization) {
            case 0:
                mainSpike = spikeLeft;
                yellowScoring = backboardLeft;
                transition = isUnderTruss ? outerTruss : stageDoor;
                whiteScoring = isUnderTruss ? backboardRight : backboardCenter;
                break;
            case 1:
                mainSpike = spikeCenter;
                yellowScoring = backboardCenter;
                transition = isUnderTruss ? outerTruss : spikeDodgeStageDoor;
                whiteScoring = isUnderTruss ? backboardRight : backboardLeft;
                break;
            case 2:
                mainSpike = spikeRight;
                yellowScoring = backboardRight;
                transition = isUnderTruss ? outerTruss : spikeDodgeStageDoor;
                whiteScoring = isUnderTruss ? backboardCenter : backboardLeft;
                break;
        }
        pixelStack = isUnderTruss ? pixelStack3 : pixelStack1;
    }

    private static TrajectoryActionBuilder scorePurplePixel(TrajectoryActionBuilder builder) {
        if (randomization == 1) {
            builder = builder
                    .lineToY(mainSpike.position.y)
//                    .stopAndAdd(robot.purplePixel::toggle)
                    .waitSeconds(0.3);
        } else if (isRed ? randomization == 2 : randomization == 0) {
            builder = builder
                    .setReversed(true)
                    .strafeTo(mainSpike.position)
                    .strafeTo(new Vector2d((mainSpike.position.x + 6), (mainSpike.position.y - 16)));
        } else {
            builder = builder
                    .setReversed(true)
                    .splineTo(mainSpike.position, toRadians(135))
//                    .stopAndAdd(robot.purplePixel::toggle)
                    .waitSeconds(0.3);

            builder = builder
                    .setReversed(false)
                    .strafeTo(new Vector2d((randomization == 0 ? mainSpike.position.x + 6: mainSpike.position.x - 6), (mainSpike.position.y - 6)));
        }

        return builder;
    }

    private static TrajectoryActionBuilder scoreYellowPixel(TrajectoryActionBuilder builder) {
        if (randomization == 1) {
            builder = builder
                    .lineToY(mainSpike.position.y - 4)
                    .splineToLinearHeading(backboardCenter, Math.PI / 2);
        }
        builder = builder
                .setReversed(true)
                .strafeToLinearHeading(yellowScoring.position, LEFT)
                /*.stopAndAdd(new SequentialAction(
                        new InstantAction(() -> robot.arm.setFlap(true)),
                        new InstantAction(() -> robot.lift.setToAutonHeight(200)),
                        new SleepAction(0.6),
                        new InstantAction(robot.arm::toggleArm),
                        new SleepAction(0.5),
                        new InstantAction(() -> robot.arm.setFlap(false)),
                        new SleepAction(0.45),
                        new InstantAction(() -> {
                            robot.arm.setFlap(true);
                            robot.arm.toggleArm();
                        }),
                        new SleepAction(0.5),
                        new InstantAction(robot.lift::retract)
                ))*/;

        return builder;
    }
}