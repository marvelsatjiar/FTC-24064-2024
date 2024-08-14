package org.firstinspires.ftc.teamcode.robot.centerstage.opmode;

import static org.firstinspires.ftc.teamcode.robot.centerstage.opmode.MainTeleOp.robot;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.auto.Actions;

// CenterStage
@Autonomous(name = "Test Trajectory Stalling", group = "Mechanism Test")
public class TestTrajectoryStalling extends AbstractAuto {
    @Override
    protected Pose2d getStartPose() {
        return new Pose2d(0, 0, 0);
    }

    @Override
    protected Action onRun() {
        return waitForPress();
    }

    // Example
    private Action waitForPress() {
        return robot.drivetrain.actionBuilder(getStartPose())
                .lineToX(10)
                .stopAndAdd(new Actions.RunnableAction(() -> {
                    MainTeleOp.gamepadEx1.readButtons();
                    return !MainTeleOp.gamepadEx1.isDown(GamepadKeys.Button.A);
                }))
                .lineToX(0)
                .build();
    }
}
