package org.firstinspires.ftc.teamcode.robot.drivetrain;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.control.filters.dualfilter.ComplementaryFilter;
import org.firstinspires.ftc.teamcode.control.gainmatrices.ComplementaryGains;
import org.firstinspires.ftc.teamcode.roadrunner.message.PoseMessage;
import org.firstinspires.ftc.teamcode.sensor.vision.AprilTagSensor;

// Pose correction works with any localizer, an advantage of using a separate drive class
public class AprilTagDrive extends MecanumDrive {
    public static class Params {
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

    public AprilTagDrive(HardwareMap hardwareMap, Pose2d pose) {
        super(hardwareMap, pose);

        this.hardwareMap = hardwareMap;
    }

    /**
     * Create AprilTagSensor after drivetrain creation. This is useful for applications where the camera must be used for other purposes before sensing April Tags
     */
    public void createAprilTagSensor() {
        aprilTag = new AprilTagSensor(hardwareMap, PARAMS.isBackFacing, PARAMS.offset, "Webcam 1");
    }

    @Override
    public PoseVelocity2d updatePoseEstimate() {
        Twist2dDual<Time> twist = localizer.update();

        Pose2d localizerPose = pose.plus(twist.value());

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
        } else {
            pose = localizerPose;
        }

        poseHistory.add(pose);
        while (poseHistory.size() > 100) {
            poseHistory.removeFirst();
        }

        estimatedPoseWriter.write(new PoseMessage(pose));

        return twist.velocity().value();
    }
}
