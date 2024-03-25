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

// TODO Make into a localizer
public class AprilTagDrive extends MecanumDrive {
    // For example, 0.3 means odometry will be weighted 40% while April Tag estimate will be weighted 70%
    private static final double ODO_TRUST_COEF = 0.4;

    // ---

    private AprilTagSensor aprilTag;
    private HardwareMap hardwareMap;

    private final ComplementaryFilter filter = new ComplementaryFilter(new ComplementaryGains(ODO_TRUST_COEF));

    public AprilTagDrive(HardwareMap hardwareMap, Pose2d pose) {
        super(hardwareMap, pose);

        this.hardwareMap = hardwareMap;
    }

    /**
     * Create AprilTagSensor after drivetrain creation. This is useful for applications where the camera must be used for other purposes before sensing April Tags
     */
    public void createAprilTagSensor() {
        aprilTag = new AprilTagSensor(hardwareMap);
    }

    @Override
    public PoseVelocity2d updatePoseEstimate() {
        Twist2dDual<Time> twist = localizer.update();

        Pose2d localizerPose = pose.plus(twist.value());

        Pose2d estimate = null;
        if (aprilTag != null) {
            estimate = aprilTag.getPoseEstimate();
        }

        if (estimate != null) {
            pose = new Pose2d(
                    new Vector2d(
                            filter.calculate(localizerPose.position.x, estimate.position.x),
                            filter.calculate(localizerPose.position.y, estimate.position.y)
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
