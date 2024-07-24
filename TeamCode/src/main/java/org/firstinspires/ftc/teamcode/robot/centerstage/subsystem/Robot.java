package org.firstinspires.ftc.teamcode.robot.centerstage;

import static org.firstinspires.ftc.teamcode.robot.centerstage.Arm.TIME_DEPOSIT_1_PIXEL;
import static org.firstinspires.ftc.teamcode.util.SimpleServoPivot.getGoBildaServo;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.drivetrain.MecanumDrive;
import org.firstinspires.ftc.teamcode.util.BulkReader;
import org.firstinspires.ftc.teamcode.util.SimpleServoPivot;

/**
 * Gets all the classes for the robot and calls them with their right parameters
 */
@Config
public final class Robot {
    public static MultipleTelemetry mTelemetry;
    public static double maxVoltage = 13;
    public final MecanumDrive drivetrain;
    public final Arm arm;
    public final Lift lift;
    public final SimpleServoPivot launcher;
    public final SimpleServoPivot launcherClamp;
    public final DistanceSensor leftDistanceSensor;
    public final DistanceSensor rightDistanceSensor;
    public final Rollers rollers;
    public final SimpleServoPivot purplePixel;
    private final BulkReader bulkReader;
    private final ElapsedTime depositTimer = new ElapsedTime();
    private final ElapsedTime retractTimer = new ElapsedTime();
    public int depositedPixels;

    public static double
            ANGLE_DRONE_LOAD = 140,
            ANGLE_DRONE_LAUNCH = 0,
            ANGLE_DRONE_CLAMP = 90,
            ANGLE_DRONE_UNCLAMPED = 0,
            ANGLE_PURPLE_PIXEL_UNDEPLOYED = 0,
            ANGLE_PURPLE_PIXEL_DEPLOYED = 90;


    public Robot(HardwareMap hardwareMap, Telemetry telemetry) {
        this(hardwareMap, telemetry, new Pose2d(0.0, 0.0, 0.0));
    }

    /**
     * Instantiates a new robot.
     * @param hardwareMap A constant map that holds all the parts for config in code
     */
    public Robot(HardwareMap hardwareMap, Telemetry telemetry, Pose2d pose) {
        mTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        bulkReader = new BulkReader(hardwareMap);

        drivetrain = new MecanumDrive(hardwareMap, pose);
        arm = new Arm(hardwareMap);
        lift = new Lift(hardwareMap);
        rollers = new Rollers(hardwareMap);

        launcher = new SimpleServoPivot(ANGLE_DRONE_LOAD, ANGLE_DRONE_LAUNCH, getGoBildaServo(hardwareMap, "launcher"));
        launcherClamp = new SimpleServoPivot(ANGLE_DRONE_CLAMP, ANGLE_DRONE_UNCLAMPED, getGoBildaServo(hardwareMap, "launcher clamp"));

        leftDistanceSensor = hardwareMap.get(DistanceSensor.class, "left distance");
        rightDistanceSensor = hardwareMap.get(DistanceSensor.class, "right distance");

        purplePixel = new SimpleServoPivot(ANGLE_PURPLE_PIXEL_UNDEPLOYED, ANGLE_PURPLE_PIXEL_DEPLOYED,
                getGoBildaServo(hardwareMap, "purple placer")
        );
    }

    public void readSensors() {
        bulkReader.bulkRead();
    }

    public void deposit() {
        depositTimer.reset();
        arm.toggleArm();
        if (depositTimer.milliseconds() >= 1500) arm.toggleFlap();
    }

    public void retract() {
        if (depositedPixels >= 2) {
            retractTimer.reset();
            arm.toggleArm();
            if (retractTimer.milliseconds() >= 1500) lift.retract();
        }
    }

    public void hang(double motorPower) {
        arm.run();
        lift.run(motorPower, false);
    }

    public void run() {
        if (lift.getSetPoint() == -1) {
            arm.setFlap(rollers.intakePower() == 0);
        } else {
            if (arm.flapTimerCondition && arm.flapTimer.seconds() >= TIME_DEPOSIT_1_PIXEL) {
                arm.setFlap(true);
                arm.flapTimerCondition = false;
            }
        }

        rollers.run();
        purplePixel.run();
        launcherClamp.run();
        launcher.run();
        lift.run();
        arm.run();
    }

    /**
     * Print telemetry data for user debugging
     */
    public void printTelemetry() {
        arm.printTelemetry();
        mTelemetry.addLine();
        lift.printTelemetry();
        mTelemetry.addLine();
        rollers.printTelemetry();
        mTelemetry.addLine();
        mTelemetry.addLine();
        mTelemetry.addLine();
        lift.printNumericalTelemetry();
    }
}
