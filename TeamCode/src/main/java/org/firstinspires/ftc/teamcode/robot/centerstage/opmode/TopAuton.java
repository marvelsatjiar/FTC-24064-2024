package org.firstinspires.ftc.teamcode.robot.centerstage.opmode;

import static java.lang.Math.toRadians;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TimeTrajectory;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


import static org.firstinspires.ftc.teamcode.robot.centerstage.Robot.mTelemetry;
import static org.firstinspires.ftc.teamcode.robot.centerstage.opmode.AutonMechanisms.AutonMechanics.DodgeObjects;
import static org.firstinspires.ftc.teamcode.robot.centerstage.opmode.MainAuton.autonEndPose;
import static org.firstinspires.ftc.teamcode.robot.centerstage.opmode.MainAuton.backboardCenter;
import static org.firstinspires.ftc.teamcode.robot.centerstage.opmode.MainAuton.getAllianceSideData;
import static org.firstinspires.ftc.teamcode.robot.centerstage.opmode.MainAuton.getPropSensorData;
import static org.firstinspires.ftc.teamcode.robot.centerstage.opmode.MainAuton.mainSpike;
import static org.firstinspires.ftc.teamcode.robot.centerstage.opmode.MainAuton.pixelStack;
import static org.firstinspires.ftc.teamcode.robot.centerstage.opmode.MainAuton.start;
import static org.firstinspires.ftc.teamcode.robot.centerstage.opmode.MainAuton.whiteScoring;
import static org.firstinspires.ftc.teamcode.robot.centerstage.opmode.MainAuton.yellowScoring;
import static org.firstinspires.ftc.teamcode.robot.centerstage.opmode.MainAuton.transition;

import org.firstinspires.ftc.teamcode.robot.centerstage.Robot;

import org.firstinspires.ftc.teamcode.robot.centerstage.opmode.AutonMechanisms.AutonMechanics;
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

    public enum TrajStates {
        RANDOMIZATION,
        CYCLING,
        IDLE
    }

    TrajStates currentTraj;

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

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, telemetry, new Pose2d(0, 0, 0));

        getAllianceSideData(this);

        TrajectoryActionBuilder[] trajectories = {getTrajectory(0), getTrajectory(1), getTrajectory(2)};
        TrajectoryActionBuilder trajectory;

        int randomization = getPropSensorData(this);

        trajectory = trajectories[randomization];

        trajectory.build();

        robot.drivetrain.pose = start;

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
    }

    private TrajectoryActionBuilder getTrajectory(int randomization) {
        MainAuton.setLogic(randomization, isUnderTruss);

        TrajectoryActionBuilder builder = isRed ? robot.drivetrain.actionBuilder(start) : robot.drivetrain.mirroredActionBuilder(start);

        currentTraj = TrajStates.RANDOMIZATION;
        Actions.runBlocking(DodgeObjects(builder, currentTraj));
        scorePurplePixel(builder, randomization);
        scoreYellowPixel(builder, randomization);
        getWhitePixels(builder);
        scoreWhitePixels(builder);
        getWhitePixels(builder);
        scoreWhitePixels(builder);
        currentTraj = TrajStates.IDLE;

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
            .splineTo(transition.position, Math.PI);

        currentTraj = TrajStates.CYCLING;

        builder
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
