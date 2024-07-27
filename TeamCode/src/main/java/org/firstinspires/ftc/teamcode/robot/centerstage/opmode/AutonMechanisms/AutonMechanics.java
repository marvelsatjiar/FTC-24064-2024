package org.firstinspires.ftc.teamcode.robot.centerstage.opmode.AutonMechanisms;

import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH;
import static org.firstinspires.ftc.teamcode.robot.centerstage.subsystem.Robot.mTelemetry;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;

import org.firstinspires.ftc.teamcode.control.controller.PIDController;
import org.firstinspires.ftc.teamcode.control.gainmatrices.PIDGains;
import org.firstinspires.ftc.teamcode.control.motion.State;
import org.firstinspires.ftc.teamcode.robot.centerstage.subsystem.Robot;
import org.firstinspires.ftc.teamcode.robot.centerstage.opmode.MainAuton;
import org.firstinspires.ftc.teamcode.util.LoopUtil;

public class AutonMechanics {
    static Double targetPower;
    private static boolean hasDodged = true;

    public static Action AsyncTrajectoryObjectDodgeAction(Action traj, Robot robot, MainAuton.TrajStates currentTraj) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                // Action to update all of the needed constants
                UpdateAction(robot);

                // Action to detect objects for the below actions
//                objectDetection(robot);

                // Logic for whether moving aside or stopping depending if an object is there
//                if (!hasDodged) {
//                    if (currentTraj == TopAuton.TrajStates.RANDOMIZATION) {
//                        robot.drivetrain.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
//                    } else if (currentTraj == TopAuton.TrajStates.CYCLING) {
//                        robot.drivetrain.setDrivePowers(
//                                new PoseVelocity2d(
//                                        new Vector2d(
//                                                0,
//                                                targetPower
//                                        ),
//                                        0
//                                )
//                        );
//                    }
                    // Notably, returning true to run again without changing traj
//                    return true;
//                } else {
                    // Returning back to normal traj; everything is normal
                    return traj.run(telemetryPacket);
//                }
            }
        };
    }

    public static Action UpdateAction(Robot robot) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                // Necessary reading to drive robot and read sensors, as well as updating pose estimate
                robot.readSensors();
                robot.run();

                // Telemetry for debugging if needed
                mTelemetry.addData("Loop time (hertz)", LoopUtil.getLoopTimeInHertz());

                // No need to keep on running the action; the loop in the above action will run it inside the loop
                return true;
            }
        };
    }

    public static Action scoreAction(Robot robot, boolean isWhite) {
        return new SequentialAction(
                new InstantAction(() -> robot.arm.setFlap(true)),
                new InstantAction(() -> robot.lift.setToAutonHeight(isWhite ? 200 : 0)),
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
        );
    }

    public static Action intakeAction(Robot robot, int cycle) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {

                robot.rollers.setDeployable(cycle == 1 ? 52 : 32.5);
                robot.rollers.setIntake(0.8);
                new SleepAction(0.5);
                robot.rollers.setDeployable(cycle == 1 ? 46 : 20);
                new SleepAction(0.5);
                robot.rollers.resetDeployable();
                robot.rollers.setIntake(0);

                return false;
            }
        };
    }

}
