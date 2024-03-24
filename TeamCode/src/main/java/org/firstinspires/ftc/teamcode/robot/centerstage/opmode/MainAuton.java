package org.firstinspires.ftc.teamcode.robot.centerstage.opmode;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.B;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_DOWN;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_UP;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.LEFT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.RIGHT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.X;
import static org.firstinspires.ftc.teamcode.robot.centerstage.Robot.mTelemetry;
import static java.lang.Math.toRadians;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.centerstage.Robot;
import org.firstinspires.ftc.teamcode.sensor.vision.PropSensor;

@Config
@Autonomous(group = "24064 Main", preselectTeleOp = "MainTeleOp")
public final class MainAuton extends LinearOpMode {
    static Robot robot;
    static PropSensor propSensor;
    public static GamepadEx gamepadEx1, gamepadEx2;
    public static Pose2d autonEndPose = null;

    public static boolean
            isRed = false,
            isBackboardSide = true;

    public static final double
            LEFT = toRadians(180),
            FORWARD = toRadians(90),
            RIGHT = toRadians(0),
            BACKWARD = toRadians(270);

    public static boolean keyPressed(int gamepad, GamepadKeys.Button button) {
        return (gamepad == 2 ? gamepadEx2 : gamepadEx1).wasJustPressed(button);
    }

    public static void getAllianceSideData(LinearOpMode opMode) {
        gamepadEx1 = new GamepadEx(opMode.gamepad1);

        // Get gamepad 1 button input and save "right" and "red" booleans for autonomous configuration:
        while (opMode.opModeInInit() && !(gamepadEx1.isDown(RIGHT_BUMPER) && gamepadEx1.isDown(LEFT_BUMPER))) {
            gamepadEx1.readButtons();
            if (keyPressed(1, DPAD_UP)) isBackboardSide = true;
            if (keyPressed(1, DPAD_DOWN)) isBackboardSide = false;
            if (keyPressed(1, B)) isRed = true;
            if (keyPressed(1, X)) isRed = false;
            mTelemetry.addLine("| B - RED | X - BLUE |");
            mTelemetry.addLine("| D-pad-down - AUDIENCE | D-pad-up - BACKBOARD |");
            mTelemetry.addLine();
            mTelemetry.addLine("Selected " + (isRed ? "RED" : "BLUE") + " " + (isBackboardSide ? "BACKBOARD" : "AUDIENCE"));
            mTelemetry.addLine("Press both shoulder buttons to confirm!");
            mTelemetry.update();
        }
    }

    public static int getPropSensorData(LinearOpMode opMode) {
        propSensor = new PropSensor(opMode.hardwareMap, isRed);

        while (!propSensor.getIsOpened()) {
            mTelemetry.addLine("Confirmed " + (isRed ? "RED" : "BLUE") + " " + (isBackboardSide ? "BACKBOARD" : "AUDIENCE"));
            mTelemetry.addLine("Camera is not open");
            mTelemetry.update();
        }

        int randomization = 0;
        while (!opMode.isStarted() && !opMode.isStopRequested()) {
            randomization = propSensor.propPosition();
            mTelemetry.addData("Predicted Prop Placement", randomization);
            mTelemetry.update();
            opMode.sleep(50);
        }

        propSensor.getCamera().closeCameraDeviceAsync(() -> {
            mTelemetry.addLine("Camera closed");
            mTelemetry.update();
        });

        return randomization;
    }

    @Override
    public void runOpMode() {
        robot = new Robot(hardwareMap, telemetry, new Pose2d(0, 0, 0));

        getAllianceSideData(this);
        int randomization = getPropSensorData(this);

//        robot.drivetrain.pose = ...;

        while (opModeIsActive()) {
            robot.readSensors();

            robot.drivetrain.updatePoseEstimate();
            robot.run();
        }

//        autonEndPose = ...;
    }
}