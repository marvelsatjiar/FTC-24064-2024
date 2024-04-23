package org.firstinspires.ftc.teamcode.robot.centerstage.opmode.AutonMechanisms;

import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TimeTrajectory;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.control.controller.PIDController;
import org.firstinspires.ftc.teamcode.control.gainmatrices.PIDGains;
import org.firstinspires.ftc.teamcode.control.motion.State;
import org.firstinspires.ftc.teamcode.robot.centerstage.Robot;
import org.firstinspires.ftc.teamcode.robot.centerstage.opmode.TopAuton;
import org.firstinspires.ftc.teamcode.robot.drivetrain.MecanumDrive;

public class AutonMechanics {
    static Robot robot;

    private static TopAuton.TrajStates currentTraj;

    AutonMechanics(TopAuton.TrajStates currentTraj) {
        this.currentTraj = currentTraj;
    }

    public static Action PausingTrajectoryAction(Action traj, boolean hasDodged) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!hasDodged) {
                    if (currentTraj == TopAuton.TrajStates.RANDOMIZATION) {
                        robot.drivetrain.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
                    } else if (currentTraj == TopAuton.TrajStates.CYCLING) {
                        robot.drivetrain.setDrivePowers(
                                new PoseVelocity2d(
                                        new Vector2d(
                                                0,
                                                targetPower
                                        ),
                                        0
                                )
                        );
                    }
                    return true;
                } else {
                    return traj.run(telemetryPacket);
                }
            }
        };
    }


    public static class DodgeObjects implements Action {

        private TrajectoryActionBuilder privateTrajectory;

        private boolean hasDodged = true;
        private double
                leftDistance,
                rightDistance,
                targetPower;

        // TODO: PLEASE TUNE!!!!!!!!!!!!!!!!!!!
        public PIDGains pidGains = new PIDGains(
                0.005,
                0.002,
                0.0001,
                Double.POSITIVE_INFINITY
        );

        private final PIDController controller = new PIDController();
        private State targetState;

        private DodgeObjects(TrajectoryActionBuilder privateTrajectory, TopAuton.TrajStates currentTraj) {
            this.privateTrajectory = privateTrajectory;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            new ParallelAction(
                    privateTrajectory.build(),
                    PausingTrajectoryAction(privateTrajectory.build(), hasDodged),
                    telemetryPacket1 -> {
                        controller.setGains(pidGains);
                        return false;
                    },

                    new SequentialAction(
                            telemetryPacket1 -> {
                                leftDistance = robot.leftDistanceSensor.getDistance(INCH);
                                rightDistance = robot.rightDistanceSensor.getDistance(INCH);
                                return true;
                                },

                            telemetryPacket1 -> {
                                if (leftDistance + rightDistance < 4) {
                                    hasDodged = false;
                                    if (leftDistance < rightDistance) {
                                        targetState = new State(-2);
                                        targetPower = controller.calculate(new State(leftDistance));
                                    } else {
                                        targetState = new State(2);
                                        targetPower = controller.calculate(new State(rightDistance));
                                    }
                                }

                                hasDodged = true;
                                return currentTraj != TopAuton.TrajStates.RANDOMIZATION;
                            }
                    ),

                    telemetryPacket1 -> {
                        controller.setTarget(targetState);
                        if (currentTraj == TopAuton.TrajStates.RANDOMIZATION && !hasDodged) {
                            new InstantAction(() -> robot.drivetrain.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0)));
                        } else if (!hasDodged) {
                            robot.drivetrain.setDrivePowers(
                                    new PoseVelocity2d(
                                            new Vector2d(
                                                    0,
                                                    targetPower
                                            ),
                                            0
                                    )
                            );
                        }
                        return hasDodged;
                    }
            );
            return currentTraj != TopAuton.TrajStates.IDLE;
        }
    }

    public static Action DodgeObjects(TrajectoryActionBuilder privateTrajectory, TopAuton.TrajStates currentTraj) {
        return new DodgeObjects(privateTrajectory, currentTraj);
    }
}
