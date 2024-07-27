package org.firstinspires.ftc.teamcode.robot.centerstage.opmode;

import static java.lang.Math.toRadians;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


import static org.firstinspires.ftc.teamcode.robot.centerstage.opmode.AutonMechanisms.AutonMechanics.UpdateAction;
import static org.firstinspires.ftc.teamcode.robot.centerstage.opmode.MainAuton.backboardCenter;
import static org.firstinspires.ftc.teamcode.robot.centerstage.opmode.MainAuton.getAllianceSideData;
import static org.firstinspires.ftc.teamcode.robot.centerstage.opmode.MainAuton.getPropSensorData;
import static org.firstinspires.ftc.teamcode.robot.centerstage.opmode.MainAuton.mainSpike;
import static org.firstinspires.ftc.teamcode.robot.centerstage.opmode.MainAuton.parkingRight;
import static org.firstinspires.ftc.teamcode.robot.centerstage.opmode.MainAuton.pixelStack;
import static org.firstinspires.ftc.teamcode.robot.centerstage.opmode.MainAuton.startBlue;
import static org.firstinspires.ftc.teamcode.robot.centerstage.opmode.MainAuton.startRed;
import static org.firstinspires.ftc.teamcode.robot.centerstage.opmode.MainAuton.whiteScoring;
import static org.firstinspires.ftc.teamcode.robot.centerstage.opmode.MainAuton.yellowScoring;
import static org.firstinspires.ftc.teamcode.robot.centerstage.opmode.MainAuton.transition;


import org.firstinspires.ftc.teamcode.robot.centerstage.subsystem.Robot;

@Config
@Autonomous(name = "TopAuton", group = "24064 Main", preselectTeleOp = "MainTeleOp")
public final class Top2_4 extends LinearOpMode {
    static Robot robot;

    static Action dodgeObjectsAction;
    static boolean
            isRed = false,
            isParkedMiddle = true,
            isUnderTruss = false,
            doAprilTag = false;

    MainAuton.TrajStates currentTraj;

    public static double
            START_X = 12;

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

        TrajectoryActionBuilder builder = isRed ? robot.drivetrain.mirroredActionBuilder(startRed) : robot.drivetrain.actionBuilder(startBlue);
        /* any code that you write here happens when the trajectory is *built*, not when it runs
         so you can't access current traj directly like you had before (commented out)
         currentTraj = TrajStates.RANDOMIZATION;
         instead do it in actions which run when the trajectory is being run */

//        builder.afterTime(0, new InstantAction(() -> currentTraj = TrajStates.RANDOMIZATION));
        builder = scorePurplePixel(builder, randomization);
        builder = scoreYellowPixel(builder, randomization);
//        builder = getWhitePixels(builder, 1);
//        builder = scoreWhitePixels(builder);
//        builder = getWhitePixels(builder, 2);
//        builder = scoreWhitePixels(builder);
        builder = builder.strafeTo(parkingRight.position);
//        builder.afterTime(0, new InstantAction(() -> currentTraj = TrajStates.IDLE));

        return builder;
    }
    private TrajectoryActionBuilder scorePurplePixel(TrajectoryActionBuilder builder, int randomization) {
        if (randomization == 1) {
            builder = builder
                    .lineToY(mainSpike.position.y)
                    .stopAndAdd(robot.purplePixel::toggle)
                    .waitSeconds(0.3);
        } else if (isRed ? randomization == 2 : randomization == 0) {
            builder = builder
                    .setReversed(true)
                    .strafeTo(mainSpike.position)
                    .stopAndAdd(robot.purplePixel::toggle)
                    .waitSeconds(0.3)
                    .strafeTo(new Vector2d((mainSpike.position.x + 6), (mainSpike.position.y - 16)));
        } else {
            builder = builder
                    .setReversed(true)
                    .splineTo(mainSpike.position, toRadians(135))
                    .stopAndAdd(robot.purplePixel::toggle)
                    .waitSeconds(0.3);

            builder = builder
                    .setReversed(false)
                    .strafeTo(new Vector2d((randomization == 0 ? mainSpike.position.x + 6: mainSpike.position.x - 6), (mainSpike.position.y - 6)));
        }

        return builder;
    }

    private TrajectoryActionBuilder scoreYellowPixel(TrajectoryActionBuilder builder, int randomization) {
        if (randomization == 1) {
            builder = builder
                    .lineToY(mainSpike.position.y - 4)
                    .splineToLinearHeading(backboardCenter, Math.PI / 2);
        }
        builder = builder
                .setReversed(true)
                .strafeToLinearHeading(yellowScoring.position, LEFT)
                .stopAndAdd(new SequentialAction(
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
        ));

        return builder;
    }

    private TrajectoryActionBuilder getWhitePixels(TrajectoryActionBuilder builder, int cycle) {
        builder = builder
            .setReversed(false)
            .setTangent(pixelStack.heading)
            .splineTo(transition.position, Math.PI)
//            .afterTime(0, new InstantAction(() -> currentTraj = TrajStates.CYCLING))
            .splineTo(pixelStack.position, Math.PI);

//        intakeAction(robot, cycle);

        return builder;
    }

    private TrajectoryActionBuilder scoreWhitePixels(TrajectoryActionBuilder builder) {
        builder = builder
            .setReversed(true)
            .splineTo(transition.position, RIGHT)
            .splineTo(whiteScoring.position, RIGHT);

//        scoreAction(robot, true);

        return builder;
    }
}
