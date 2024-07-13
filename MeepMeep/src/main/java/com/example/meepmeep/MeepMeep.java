package com.example.meepmeep;

import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.acmerobotics.roadrunner.Pose2d;

public class MeepMeep {

    static boolean
            isRed = false,
            isParkedMiddle = true,
            isUnderTruss = false,
            doAprilTag = false,
            runScore = false;

    public static double
            START_X = -35;

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
            spikeLeft = new Pose2d((START_X - 6), -34.5, toRadians(0)),
            spikeCenter = new Pose2d((START_X + 6), -33.5, toRadians(0)),
            spikeRight = new Pose2d((START_X + 6), -36, toRadians(0)),
            backboardLeft = new Pose2d(BACKBOARD_X, isRed ? -30.5 : 30.5, LEFT),
            backboardCenter = new Pose2d(BACKBOARD_X, isRed ? -35 : 35, LEFT),
            backboardRight = new Pose2d(BACKBOARD_X, isRed ? -41 : 41, LEFT),
            parkingLeft = new Pose2d(48.5, -10, toRadians(165)),
            parkingRight = new Pose2d(48.5, -56, toRadians(200)),
            spikeDodgeStageDoor = new Pose2d(23, -10, LEFT),
            stageDoor = new Pose2d(16, -10, LEFT),
            outerTruss = new Pose2d(23.5, -58, LEFT),
            pixelStack1 = new Pose2d(-59.25, isRed ? -12 : 12, LEFT),
            pixelStack3 = new Pose2d(-59.25, isRed ? -35 : 35, LEFT),
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

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-35, -66.775, BACKWARD))
                    .setReversed(true)
                    .splineTo(spikeRight.position, -Math.PI)
                    .setReversed(false)
                    .lineToX(spikeLeft.position.x - 3)
                .build());



        meepMeep.setBackground(com.noahbres.meepmeep.MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}