package org.firstinspires.ftc.teamcode.robot.centerstage.opmode.test;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.auto.Actions;
import org.firstinspires.ftc.teamcode.robot.centerstage.opmode.AbstractAuto;
import org.firstinspires.ftc.teamcode.robot.centerstage.opmode.MainTeleOp;

// CenterStage
@Autonomous(name = "Test Trajectory Stalling", group = "Mechanism Test")
public class TestTrajectoryStalling extends AbstractAuto {
    private static final Pose2d start = new Pose2d(0, 0, 0);

    @Override
    protected Pose2d getStartPose() {
        return start;
    }

    @Override
    protected void onRun() {
        scorePurple();
    }

    // Example
    private void scorePurple() {
        schedule.addAction(robot.drivetrain.actionBuilder(start)
                .lineToX(10)
                .build());

        schedule.addAction(new Actions.RunnableAction(() -> {
            MainTeleOp.gamepadEx1.readButtons();
            return MainTeleOp.gamepadEx1.isDown(GamepadKeys.Button.A);
        }));

        schedule.addAction(robot.drivetrain.actionBuilder(new Pose2d(10, 0, 0))
                .lineToX(0)
                .build());

        schedule.run();
    }
}
