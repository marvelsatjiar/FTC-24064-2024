package org.firstinspires.ftc.teamcode.robot.centerstage.opmode;

import static org.firstinspires.ftc.teamcode.robot.centerstage.opmode.AutonMechanisms.AutonMechanics.AsyncTrajectoryObjectDodgeAction;
import static org.firstinspires.ftc.teamcode.robot.centerstage.opmode.AutonMechanisms.AutonMechanics.intakeAction;
import static org.firstinspires.ftc.teamcode.robot.centerstage.opmode.AutonMechanisms.AutonMechanics.scoreAction;
import static org.firstinspires.ftc.teamcode.robot.centerstage.opmode.MainAuton.backboardCenter;
import static org.firstinspires.ftc.teamcode.robot.centerstage.opmode.MainAuton.getAllianceSideData;
import static org.firstinspires.ftc.teamcode.robot.centerstage.opmode.MainAuton.getPropSensorData;
import static org.firstinspires.ftc.teamcode.robot.centerstage.opmode.MainAuton.mainSpike;
import static org.firstinspires.ftc.teamcode.robot.centerstage.opmode.MainAuton.pixelStack;
import static org.firstinspires.ftc.teamcode.robot.centerstage.opmode.MainAuton.start;
import static org.firstinspires.ftc.teamcode.robot.centerstage.opmode.MainAuton.transition;
import static org.firstinspires.ftc.teamcode.robot.centerstage.opmode.MainAuton.whiteScoring;
import static org.firstinspires.ftc.teamcode.robot.centerstage.opmode.MainAuton.yellowScoring;
import static org.firstinspires.ftc.teamcode.robot.centerstage.opmode.MainAuton.TrajStates;
import static java.lang.Math.PI;
import static java.lang.Math.toRadians;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.centerstage.Robot;
import org.firstinspires.ftc.teamcode.robot.centerstage.opmode.AutonMechanisms.AutonMechanics;

@Config
@Autonomous(name = "Top 2+4", group = "24064 Main", preselectTeleOp = "MainTeleOp")
public final class BottomAuton extends LinearOpMode {
    static Robot robot;

    static Action dodgeObjectsAction;
    static boolean
            isRed = false,
            isParkedMiddle = true,
            isUnderTruss = false,
            doAprilTag = false,
            runScore = false;

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


        // TODO: This runs *blocking* meaning nothing after it will happen until the entire trajectory finishes
        Actions.runBlocking(traj);
        Actions.runBlocking(AsyncTrajectoryObjectDodgeAction(traj, robot, currentTraj));

    }

    private TrajectoryActionBuilder getTrajectory(int randomization) {
        MainAuton.setLogic(randomization, isUnderTruss);

        TrajectoryActionBuilder builder = isRed ? robot.drivetrain.actionBuilder(start) : robot.drivetrain.mirroredActionBuilder(start);
        /* any code that you write here happens when the trajectory is *built*, not when it runs
         so you can't access current traj directly like you had before (commented out)
         currentTraj = TrajStates.RANDOMIZATION;
         instead do it in actions which run when the trajectory is being run */

        builder.afterTime(0, new InstantAction(() -> currentTraj = TrajStates.RANDOMIZATION));
        scorePurplePixel(builder, randomization);
        scoreYellowPixel(builder);
        getWhitePixels(builder, 1);
        scoreWhitePixels(builder);
        getWhitePixels(builder, 2);
        scoreWhitePixels(builder);
        builder.afterTime(0, new InstantAction(() -> currentTraj = TrajStates.IDLE));

        return builder;
    }
    private void scorePurplePixel(TrajectoryActionBuilder builder, int randomization) {
        if (randomization == 1) {
            builder
                .lineToY(mainSpike.position.y);
            robot.purplePixel.toggle();
            new SleepAction(0.15);
            builder
                .lineToY(isRed ? mainSpike.position.y - 3 : mainSpike.position.y + 3);
        } else {
            builder
                    .setReversed(true)
                    .splineTo(mainSpike.position, -Math.PI);
            robot.purplePixel.toggle();
            new SleepAction(0.15);
            builder
                    .setReversed(false)
                    .strafeTo(new Vector2d((mainSpike.position.x - 3), (mainSpike.position.y - 3)));
        }
    }

    private void scoreYellowPixel(TrajectoryActionBuilder builder) {
        builder
                .splineToLinearHeading(pixelStack, PI)
                .splineToLinearHeading(transition, PI)
                .setReversed(true)
                .splineTo(yellowScoring.position, -Math.PI);

         scoreAction(robot, false);

        builder
                .setReversed(false)
                .setTangent(pixelStack.heading);
    }

    private void getWhitePixels(TrajectoryActionBuilder builder, int cycle) {
        builder
            .splineTo(transition.position, Math.PI)
            .afterTime(0, new InstantAction(() -> currentTraj = MainAuton.TrajStates.CYCLING))
            .splineTo(pixelStack.position, Math.PI);

        intakeAction(robot, cycle);
    }

    private void scoreWhitePixels(TrajectoryActionBuilder builder) {
        builder
            .setReversed(true)
            .splineTo(transition.position, RIGHT)
            .splineTo(whiteScoring.position, RIGHT)
            .setReversed(false);

        scoreAction(robot, true);
    }
}
