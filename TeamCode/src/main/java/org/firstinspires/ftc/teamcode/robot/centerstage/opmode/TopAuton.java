package org.firstinspires.ftc.teamcode.robot.centerstage.opmode;

import static java.lang.Math.toRadians;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
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

        Action traj = trajectory.build();


        robot.drivetrain.pose = start;
        // you no longer need a loop here, as runBlocking does the loop instead
        /*
        while (opModeIsActive()) {
            if (!opModeIsActive()) {
                return;
            }
        */
            // removed robot.readSensors() and moving it into the pausing action


            Actions.runBlocking(traj);
            // todo: this runs *blocking* meaning nothing after it will happen until the entire trajectory finishes
            // removing robot.run() here as that will be updated inside your pausing action
            // same thing for robot.drivetrain.updatePoseEstimate();

            /* this part needs to be moved inside the pausing action if you want it to still work every loop
            mTelemetry.addData("Loop time (hertz)", LoopUtil.getLoopTimeInHertz());
            mTelemetry.update();

             */
        // used to be end of loop }
    }

    private TrajectoryActionBuilder getTrajectory(int randomization) {
        MainAuton.setLogic(randomization, isUnderTruss);

        TrajectoryActionBuilder builder = isRed ? robot.drivetrain.actionBuilder(start) : robot.drivetrain.mirroredActionBuilder(start);
        // any code that you write here happens when the trajectory is *built*, not when it runs
        // so you can't access current traj directly like you had before (commented out)
        // currentTraj = TrajStates.RANDOMIZATION;
        // instead do it in actions which run when the trajectory is being run
        builder.afterTime(0, // after 0 time so at the start of the trajectory
                new InstantAction(() -> // () -> is shorthand for creating an unnamed zero-argument function
                                        // so this is equivalent to run() {
                        AutonMechanics.currentTraj = TrajStates.RANDOMIZATION)); // and then this code will be run at the start of the trajectory
        // the code from DodgeObjects should all happen in the run function in the pause action
        // so i'm removing it here
        // Actions.runBlocking(DodgeObjects(builder, currentTraj));
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
            .splineTo(transition.position, Math.PI)
                // see line 111
            .afterTime(0, new InstantAction(() -> AutonMechanics.currentTraj = TrajStates.CYCLING))
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
