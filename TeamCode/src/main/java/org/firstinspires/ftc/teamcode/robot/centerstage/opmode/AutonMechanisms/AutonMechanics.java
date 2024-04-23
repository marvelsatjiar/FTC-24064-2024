package org.firstinspires.ftc.teamcode.robot.centerstage.opmode.AutonMechanisms;

import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH;
import static org.firstinspires.ftc.teamcode.robot.centerstage.Robot.mTelemetry;

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

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.control.controller.PIDController;
import org.firstinspires.ftc.teamcode.control.gainmatrices.PIDGains;
import org.firstinspires.ftc.teamcode.control.motion.State;
import org.firstinspires.ftc.teamcode.robot.centerstage.Robot;
import org.firstinspires.ftc.teamcode.robot.centerstage.opmode.TopAuton;
import org.firstinspires.ftc.teamcode.robot.drivetrain.MecanumDrive;
import org.firstinspires.ftc.teamcode.util.LoopUtil;

public class AutonMechanics {
    public static TopAuton.TrajStates currentTraj;
    static Double targetPower;
    private static boolean hasDodged = true;

    public static Action AsyncTrajectoryObjectDodgeAction(Action traj, Robot robot) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                // Action to update all of the needed constants
                UpdateAction(robot);

                // Action to detect objects for the below actions
                objectDetection(robot);

                // Logic for whether moving aside or stopping depending if an object is there
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
                    // Notably, returning true to run again without changing traj
                    return true;
                } else {
                    // Returning back to normal traj; everything is normal
                    return traj.run(telemetryPacket);
                }
            }
        };
    }

    public static Action objectDetection(Robot robot) {

        // Setting all constants below for PID and distance sensors
        Double leftDistance = robot.leftDistanceSensor.getDistance(INCH);
        Double rightDistance = robot.rightDistanceSensor.getDistance(INCH);

        // TODO: PLEASEEE TUNEE!!!!
        PIDGains pidGains = new PIDGains(
                0.005,
                0.002,
                0.0001,
                Double.POSITIVE_INFINITY
        );

        final PIDController controller = new PIDController();
        return new Action() {
            State targetState;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                // Logic for checking if it is too far & then setting necessary bools
                if (leftDistance + rightDistance < 6) {
                    hasDodged = false;
                    controller.setGains(pidGains);
                    // If left side has more distance; run that first when it is NOT in randomization
                    if (leftDistance > rightDistance & currentTraj != TopAuton.TrajStates.RANDOMIZATION) {
                        targetState = new State(-2);
                        controller.setTarget(targetState);
                        targetPower = controller.calculate(new State(leftDistance));
                        // Otherwise, run right side first when it is NOT in randomization
                    } else if (currentTraj != TopAuton.TrajStates.RANDOMIZATION) {
                        targetState = new State(2);
                        controller.setTarget(targetState);
                        targetPower = controller.calculate(new State(rightDistance));
                    }
                }
                // Returning true to always check this during traj
                return true;
            }
        };
    }

    public static Action UpdateAction(Robot robot) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                // Necessary reading to drive robot and read sensors, as well as updating pose estimate
                robot.readSensors();
                robot.drivetrain.updatePoseEstimate();
                robot.run();

                // Telemetry for debugging if needed
                mTelemetry.addData("Loop time (hertz)", LoopUtil.getLoopTimeInHertz());
                mTelemetry.update();

                // No need to keep on running the action; the loop in the above action will run it inside the loop
                return false;
            }
        };
    }
}
