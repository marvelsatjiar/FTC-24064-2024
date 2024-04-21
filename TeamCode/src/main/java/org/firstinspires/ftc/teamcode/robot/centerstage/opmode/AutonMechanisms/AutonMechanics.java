package org.firstinspires.ftc.teamcode.robot.centerstage.opmode.AutonMechanisms;

import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.control.controllers.PIDController;
import org.firstinspires.ftc.teamcode.control.gainmatrices.PIDGains;
import org.firstinspires.ftc.teamcode.control.motion.State;
import org.firstinspires.ftc.teamcode.robot.centerstage.Robot;
import org.firstinspires.ftc.teamcode.robot.centerstage.opmode.TopAuton;

public class AutonMechanics {

    static Robot robot;
    private static boolean hasDodged = true;
    private static double
            leftDistance,
            rightDistance,
            targetPower;

    // TODO: PLEASE TUNE!!!!!!!!!!!!!!!!!!!
    public static PIDGains pidGains = new PIDGains(
            0.005,
            0.002,
            0.0001,
            Double.POSITIVE_INFINITY
    );

    private static final PIDController controller = new PIDController();
    private static State targetState;

    public static class DodgeObjects implements Action {

        private final TrajectoryActionBuilder privateTrajectory;
        private final TopAuton.TrajStates currentTraj;

        private final Action cancelTraj = new FailoverAction(
            new FollowTrajectoryAction(traj),
            new InstantAction(() -> setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0)))
        );

        private DodgeObjects(TrajectoryActionBuilder privateTrajectory, TopAuton.TrajStates currentTraj) {
            this.privateTrajectory = privateTrajectory;
            this.currentTraj = currentTraj;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            new ParallelAction(
                    privateTrajectory.build(),
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
                        if (currentTraj == TopAuton.TrajStates.RANDOMIZATION) {
                            cancelTraj;
                        } else {
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
