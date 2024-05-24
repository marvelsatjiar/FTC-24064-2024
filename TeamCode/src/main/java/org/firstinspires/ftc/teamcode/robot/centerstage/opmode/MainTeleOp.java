package org.firstinspires.ftc.teamcode.robot.centerstage.opmode;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.A;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.B;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_UP;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.RIGHT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.X;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.Y;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Trigger.LEFT_TRIGGER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Trigger.RIGHT_TRIGGER;

import static org.firstinspires.ftc.teamcode.robot.centerstage.opmode.AbstractAuto.BACKWARD;
import static org.firstinspires.ftc.teamcode.robot.centerstage.opmode.AbstractAuto.FORWARD;
import static java.lang.Math.pow;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.centerstage.subsystem.Memory;
import org.firstinspires.ftc.teamcode.robot.centerstage.subsystem.Robot;
import org.firstinspires.ftc.teamcode.util.LoopUtil;

@TeleOp(group = "24064 Main")
public final class MainTeleOp extends LinearOpMode {
    static Robot robot;
    public static GamepadEx gamepadEx1;
    public static GamepadEx gamepadEx2;
    public static MultipleTelemetry mTelemetry;

    public static boolean keyPressed(int gamepad, GamepadKeys.Button button) {
        return (gamepad == 2 ? gamepadEx2 : gamepadEx1).wasJustPressed(button);
    }

    @Override
    public void runOpMode() {
        boolean isHanging = false;

        mTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        gamepadEx1 = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);

        robot = new Robot(hardwareMap);

        Pose2d pose = Memory.AUTO_END_POSE;
        if (pose != null) {
            robot.drivetrain.pose = new Pose2d(pose.position, pose.heading.toDouble() - (Memory.IS_RED ? FORWARD : BACKWARD));
        }

        robot.purplePixel.setActivated(true);

        waitForStart();

        while (opModeIsActive()) {
            // Read sensors + gamepads:
            robot.readSensors();
            gamepadEx1.readButtons();
            gamepadEx2.readButtons();

            // Gamepad 1
            // Change the heading of the drivetrain in field-centric mode
            double x = gamepadEx1.getRightX();

            if (gamepadEx1.isDown(RIGHT_BUMPER)) {
                robot.drivetrain.setDrivePowers(
                        new PoseVelocity2d(
                                new Vector2d(
                                        gamepadEx1.getLeftY() * 0.2,
                                        gamepadEx1.getLeftX() * 0.2
                                ),
                                x * 0.2
                        )
                );
            } else {
                robot.drivetrain.setDrivePowers(
                    new PoseVelocity2d(
                            new Vector2d(
                                    gamepadEx1.getLeftY(),
                                    gamepadEx1.getLeftX()
                            ),
                            x
                    )
                );
            }
            if (keyPressed(1, X) && robot.launcherClamp.isActivated()) robot.launcher.toggle();
            if (keyPressed(1, A)) robot.launcherClamp.toggle();
            if (keyPressed(1, DPAD_UP)) isHanging = !isHanging;
//            if (keyPressed(1, B)) robot.deployableRoller.toggle();

            // Gamepad 2
            double stick = pow(gamepadEx2.getLeftY(), 3);
            if (stick != 0) robot.lift.setWithStick(stick);
            if (keyPressed(2, B)) robot.arm.toggleFlap();
            if (robot.lift.getSetPoint() >= 0) {
                if (keyPressed(2, Y)) robot.arm.toggleArm();
            }

            // Shared
            // The intake power takes precedent to the first player
            double trigger1 = gamepadEx1.getTrigger(RIGHT_TRIGGER) - gamepadEx1.getTrigger(LEFT_TRIGGER);
            double trigger2 = gamepadEx2.getTrigger(RIGHT_TRIGGER) - gamepadEx2.getTrigger(LEFT_TRIGGER);
            double intake = trigger1 != 0 ? trigger1 : trigger2;
            if (!isHanging) robot.rollers.intake(intake);
            robot.rollers.setDeployableWithTrigger(intake);

            robot.drivetrain.updatePoseEstimate();

            if (!isHanging) robot.run();
            else robot.hang(trigger1);

            robot.printTelemetry();
            mTelemetry.addData("Loop time (hertz)", LoopUtil.getLoopTimeInHertz());
            mTelemetry.update();
        }
    }
}