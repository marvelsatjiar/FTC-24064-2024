package org.firstinspires.ftc.teamcode.robot.centerstage.opmode;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.B;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_DOWN;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_UP;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.LEFT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.RIGHT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.X;
import static org.firstinspires.ftc.teamcode.robot.centerstage.opmode.MainTeleOp.mTelemetry;
import static org.firstinspires.ftc.teamcode.robot.centerstage.opmode.MainTeleOp.robot;
import static java.lang.Math.toRadians;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.centerstage.subsystem.Memory;
import org.firstinspires.ftc.teamcode.robot.centerstage.subsystem.Robot;
import org.firstinspires.ftc.teamcode.sensor.vision.PropSensor;
import org.firstinspires.ftc.teamcode.util.LoopUtil;

@Config
public abstract class AbstractAuto extends LinearOpMode {
    protected PropSensor propSensor;

    public static final double
            LEFT = toRadians(180),
            FORWARD = toRadians(90),
            RIGHT = toRadians(0),
            BACKWARD = toRadians(270);

    protected final void getAllianceSideData() {
        MainTeleOp.gamepadEx1 = new GamepadEx(gamepad1);

        // Get gamepad 1 button input and save "right" and "red" booleans for autonomous configuration:
        while (opModeInInit() && !(MainTeleOp.gamepadEx1.isDown(RIGHT_BUMPER) && MainTeleOp.gamepadEx1.isDown(LEFT_BUMPER))) {
            MainTeleOp.gamepadEx1.readButtons();
            if (MainTeleOp.keyPressed(1, DPAD_UP)) Memory.IS_BACKBOARD_SIDE = true;
            if (MainTeleOp.keyPressed(1, DPAD_DOWN)) Memory.IS_BACKBOARD_SIDE = false;
            if (MainTeleOp.keyPressed(1, B)) Memory.IS_RED = true;
            if (MainTeleOp.keyPressed(1, X)) Memory.IS_RED = false;
            mTelemetry.addLine("| B - RED | X - BLUE |");
            mTelemetry.addLine("| D-pad-down - AUDIENCE | D-pad-up - BACKBOARD |");
            mTelemetry.addLine();
            mTelemetry.addLine("Selected " + (Memory.IS_RED ? "RED" : "BLUE") + " " + (Memory.IS_BACKBOARD_SIDE ? "BACKBOARD" : "AUDIENCE"));
            mTelemetry.addLine("Press both shoulder buttons to confirm!");
            mTelemetry.update();
        }
    }

    protected final int getPropSensorData() {
        propSensor = new PropSensor(hardwareMap, Memory.IS_RED);

        while (!propSensor.getIsOpened()) {
            mTelemetry.addLine("Confirmed " + (Memory.IS_RED ? "RED" : "BLUE") + " " + (Memory.IS_BACKBOARD_SIDE ? "BACKBOARD" : "AUDIENCE"));
            mTelemetry.addLine("Camera is not open");
            mTelemetry.update();
        }

        int randomization = 0;
        while (!isStarted() && !isStopRequested()) {
            randomization = propSensor.propPosition();
            mTelemetry.addData("Predicted Prop Placement", randomization);
            mTelemetry.update();
            sleep(50);
        }

        propSensor.getCamera().closeCameraDeviceAsync(() -> {
            mTelemetry.addLine("Camera closed");
            mTelemetry.update();
        });

        return randomization;
    }

    protected final void update() {
        robot.readSensors();
        robot.run();
        mTelemetry.addData("Loop time (hertz)", LoopUtil.getLoopTimeInHertz());
        mTelemetry.update();
    }

    @Override
    public final void runOpMode() {
        robot = new Robot(hardwareMap);
        mTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        onInit();
        getAllianceSideData();
        Memory.RANDOMIZATION = getPropSensorData();

        if (isStopRequested()) return;

        resetRuntime();
        robot.drivetrain.pose = getStartPose();

        Actions.runBlocking(
                new ParallelAction(
                        onRun(),
                        new org.firstinspires.ftc.teamcode.auto.Actions.RunnableAction(() -> {
                            update();
                            return opModeIsActive();
                        })
                )
        );

        Memory.AUTO_END_POSE = robot.drivetrain.pose;
    }

    protected void onInit() {}
    protected abstract Pose2d getStartPose();
    protected abstract Action onRun();
}