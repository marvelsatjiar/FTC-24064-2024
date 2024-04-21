package org.firstinspires.ftc.teamcode.auto.estimator;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.control.filter.dualfilter.ComplementaryFilter;
import org.firstinspires.ftc.teamcode.control.gainmatrices.ComplementaryGains;
import org.firstinspires.ftc.teamcode.sensor.vision.AprilTagSensor;

public class AprilTagEstimator extends Estimator implements IAprilTagEstimator {
    public static class Params {
        public String webcamName = "Webcam 1";
        // For example, 0.4 means localizer will be weighted 40% while April Tag estimate will be weighted 60%
        public double localizerTrust = 0.4;
        // Distance from camera lens to the middle of the drivetrain (inches)
        public Vector2d offset = new Vector2d(0, -7.5);
        public boolean isBackFacing = true;
    }

    public static Params PARAMS = new Params();

    private AprilTagSensor aprilTag;
    private final HardwareMap hardwareMap;

    private final ComplementaryFilter filter = new ComplementaryFilter(new ComplementaryGains(PARAMS.localizerTrust));

    public AprilTagEstimator(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    @Override
    public Pose2d estimate(Pose2d localizerPose) {
        Pose2d pose = localizerPose;
        Pose2d tagEstimate = null;

        if (aprilTag != null) {
            tagEstimate = aprilTag.getPoseEstimate(localizerPose.heading.toDouble());
        }

        if (tagEstimate != null) {
            pose = new Pose2d(
                    new Vector2d(
                            filter.calculate(localizerPose.position.x, tagEstimate.position.x),
                            filter.calculate(localizerPose.position.y, tagEstimate.position.y)
                    ),
                    localizerPose.heading
            );
        }

        return pose;
    }

    /**
     * Create AprilTagSensor after estimator creation. This is useful for applications where the camera must be used for other purposes before sensing April Tags
     */
    @Override
    public void createAprilTagSensor() {
        aprilTag = new AprilTagSensor(hardwareMap, PARAMS.isBackFacing, PARAMS.offset, PARAMS.webcamName);
    }
}
