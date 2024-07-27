package org.firstinspires.ftc.teamcode.robot.centerstage.opmode;

import static org.firstinspires.ftc.teamcode.robot.centerstage.opmode.AutonMechanisms.AutonMechanics.UpdateAction;
import static org.firstinspires.ftc.teamcode.robot.centerstage.opmode.MainAuton.getAllianceSideData;
import static org.firstinspires.ftc.teamcode.robot.centerstage.opmode.MainAuton.getPropSensorData;
import static org.firstinspires.ftc.teamcode.robot.centerstage.opmode.MainAuton.mainSpike;
import static org.firstinspires.ftc.teamcode.robot.centerstage.opmode.MainAuton.pixelStack;
import static org.firstinspires.ftc.teamcode.robot.centerstage.opmode.MainAuton.startRed;
import static org.firstinspires.ftc.teamcode.robot.centerstage.opmode.MainAuton.transition;
import static org.firstinspires.ftc.teamcode.robot.centerstage.opmode.MainAuton.whiteScoring;
import static org.firstinspires.ftc.teamcode.robot.centerstage.opmode.MainAuton.yellowScoring;
import static org.firstinspires.ftc.teamcode.robot.centerstage.opmode.MainAuton.TrajStates;
import static java.lang.Math.PI;
import static java.lang.Math.toRadians;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.centerstage.subsystem.Robot;

@Config
@Autonomous(name = "BottomAuton", group = "24064 Main", preselectTeleOp = "MainTeleOp")
public final class Bottom2_5 extends LinearOpMode {
    static Robot robot;

    static Action dodgeObjectsAction;
    static boolean
            isRed = false,
            isParkedMiddle = true,
            isUnderTruss = false,
            doAprilTag = false;

    TrajStates currentTraj;

    public static double
            START_X = -35;

    public static double
            BACKBOARD_X = 50.4,
            LEFT = toRadians(180),
            FORWARD = toRadians(90),
            RIGHT = toRadians(0),
            BACKWARD = toRadians(270);

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, telemetry, startRed);

        getAllianceSideData(this);

        int randomization = getPropSensorData(this);

        TrajectoryActionBuilder[] trajectories = {getTrajectory(0), getTrajectory(1), getTrajectory(2)};
        TrajectoryActionBuilder trajectory;

        trajectory = trajectories[randomization];

        Action traj = trajectory.build();

        // TODO: This runs *blocking* meaning nothing after it will happen until the entire trajectory finishes
        Actions.runBlocking(new ParallelAction(
                traj,
                UpdateAction(robot)
        ));

    }

    private TrajectoryActionBuilder getTrajectory(int randomization) {

        MainAuton.setLogic(randomization, isUnderTruss);

        TrajectoryActionBuilder builder = isRed ? robot.drivetrain.mirroredActionBuilder(startRed) : robot.drivetrain.actionBuilder(startRed);
        /* any code that you write here happens when the trajectory is *built*, not when it runs
         so you can't access current traj directly like you had before (commented out)
         currentTraj = TrajStates.RANDOMIZATION;
         instead do it in actions which run when the trajectory is being run */

//        builder.afterTime(0, new InstantAction(() -> currentTraj = TrajStates.RANDOMIZATION));
        builder = scorePurplePixel(builder, randomization);
        builder = scoreYellowPixel(builder);
        builder = getWhitePixels(builder, 1);
        builder = scoreWhitePixels(builder);
        builder = getWhitePixels(builder, 2);
        builder = scoreWhitePixels(builder);
//        builder.afterTime(0, new InstantAction(() -> currentTraj = TrajStates.IDLE));

        return builder;
    }
    private TrajectoryActionBuilder scorePurplePixel(TrajectoryActionBuilder builder, int randomization) {
        if (randomization == 1) {
            builder = builder
                .lineToY(mainSpike.position.y);
//            builder = builder.afterTime(0.3, new InstantAction(() -> robot.purplePixel.toggle()));
            builder = builder
                .lineToY(isRed ? mainSpike.position.y - 3 : mainSpike.position.y + 3);
        } else {
            builder = builder
                    .setReversed(true)
                    .splineTo(mainSpike.position, toRadians(randomization == 2 ? 45 : 135));
//            builder = builder.afterTime(0.3, new InstantAction(() -> robot.purplePixel.toggle()));
            builder = builder
                    .setReversed(false)
                    .strafeTo(new Vector2d((mainSpike.position.x - 3), (mainSpike.position.y - 3)));
        }

        return builder;
    }

    private TrajectoryActionBuilder scoreYellowPixel(TrajectoryActionBuilder builder) {
        builder = builder
                .splineToLinearHeading(pixelStack, -PI)
                .lineToX(transition.position.x)
                .setReversed(true)
                .splineTo(yellowScoring.position, toRadians(0));

//        builder.afterTime(0.45, scoreAction(robot, false));

        return builder;
    }

    private TrajectoryActionBuilder getWhitePixels(TrajectoryActionBuilder builder, int cycle) {
        builder = builder
            .setReversed(false)
            .setTangent(pixelStack.heading)
            .splineTo(transition.position, Math.PI)
//            .afterTime(0, new InstantAction(() -> currentTraj = MainAuton.TrajStates.CYCLING))
            .splineTo(pixelStack.position, Math.PI);

//        builder = builder.afterTime(0.3, intakeAction(robot, cycle));

        return builder;
    }

    private TrajectoryActionBuilder scoreWhitePixels(TrajectoryActionBuilder builder) {
        builder = builder
            .setReversed(true)
            .splineTo(transition.position, RIGHT)
            .splineTo(whiteScoring.position, RIGHT);

//        builder = builder.afterTime(0.45, scoreAction(robot, false));

        return builder;
    }
}
