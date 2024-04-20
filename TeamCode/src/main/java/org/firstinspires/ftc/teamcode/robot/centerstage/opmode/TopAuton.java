package org.firstinspires.ftc.teamcode.robot.centerstage.opmode;

import static java.lang.Math.toRadians;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


import static org.firstinspires.ftc.teamcode.robot.centerstage.Robot.mTelemetry;
import static org.firstinspires.ftc.teamcode.robot.centerstage.opmode.MainAuton.getAllianceSideData;
import static org.firstinspires.ftc.teamcode.robot.centerstage.opmode.MainAuton.getPropSensorData;
import org.firstinspires.ftc.teamcode.robot.centerstage.Robot;

import org.firstinspires.ftc.teamcode.robot.drivetrain.MecanumDrive;
import org.firstinspires.ftc.teamcode.util.LoopUtil;

@Config
@Autonomous(name = "Top 2+4", group = "24064 Main", preselectTeleOp = "MainTeleOp")
public final class TopAuton extends LinearOpMode {
    static Robot robot;
    static boolean
            isRed = false,
            isParkedMiddle = true,
            isUnderTruss = false,
            doAprilTag = false;

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
            spikeLeft = new Pose2d((START_X - 6), -34.5, toRadians(135)),
            spikeCenter = new Pose2d((START_X + 6), -33.5, toRadians(315)),
            spikeRight = new Pose2d(30, -36, toRadians(315)),
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

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, telemetry, new Pose2d(0, 0, 0));

        getAllianceSideData(this);

        TrajectoryActionBuilder[] trajectories = {getTrajectory(0), getTrajectory(1), getTrajectory(2)};
        TrajectoryActionBuilder trajectory;

        int randomization = getPropSensorData(this);

        trajectory = trajectories[randomization];

        while (opModeIsActive()) {
            if (!opModeIsActive()) {
                return;
            }

            robot.readSensors();

            robot.drivetrain.updatePoseEstimate();
            robot.run();

            mTelemetry.addData("Loop time (hertz)", LoopUtil.getLoopTimeInHertz());
            mTelemetry.update();
        }

        trajectory.build();
    }

    private TrajectoryActionBuilder getTrajectory(int randomization) {
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

        TrajectoryActionBuilder builder = isRed ? robot.drivetrain.actionBuilder(start) : robot.drivetrain.mirroredActionBuilder(start);

        scorePurplePixel(builder, randomization);
        scoreYellowPixel(builder, randomization);
        getWhitePixels(builder);
        scoreWhitePixels(builder);
        getWhitePixels(builder);
        scoreWhitePixels(builder);

        return builder;
    }

    private void scorePurplePixel(TrajectoryActionBuilder builder, int randomization) {
        if (randomization == 1) {
            builder
                .lineToY(mainSpike.position.y)
                .lineToY(mainSpike.position.y + 3);
        }

        builder
                .setReversed(true)
                .splineTo(mainSpike.position, -Math.PI)
                .setReversed(false);
    }

    private void scoreYellowPixel(TrajectoryActionBuilder builder, int randomization) {
        if (randomization == 1) {
            builder
                    .splineToLinearHeading(backboardCenter, Math.PI / 2);
        }

        builder
                .setReversed(true)
                .splineTo(yellowScoring.position, -Math.PI)
                .setReversed(false)
                .setTangent(pixelStack.heading);
    }

    private void getWhitePixels(TrajectoryActionBuilder builder) {
        builder
            .splineTo(transition.position, Math.PI)
            .splineTo(pixelStack.position, Math.PI);
    }

    private void scoreWhitePixels(TrajectoryActionBuilder builder) {
        builder
            .setReversed(true)
            .splineTo(transition.position, RIGHT)
            .splineTo(whiteScoring.position, RIGHT)
            .setReversed(false);
    }
}
